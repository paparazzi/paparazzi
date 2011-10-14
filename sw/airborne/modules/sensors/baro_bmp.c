/*
 * $Id: baro_bmp.c $
 *
 * Copyright (C) 2010 Martin Mueller
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file baro_bmp.c
 *  \brief Bosch BMP085 I2C sensor interface
 *
 *   This reads the values for pressure and temperature from the Bosch BMP085 sensor through I2C.
 */


#include "baro_bmp.h"

#include "sys_time.h"
#include "mcu_periph/i2c.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#ifndef SENSOR_SYNC_SEND
#warning set SENSOR_SYNC_SEND to use baro_bmp
#endif

#ifndef BMP_I2C_DEV
#define BMP_I2C_DEV i2c0
#endif

#define BMP085_SLAVE_ADDR 0xEE

struct i2c_transaction bmp_trans;

uint8_t  baro_bmp_status;
uint32_t baro_bmp_pressure;
uint16_t baro_bmp_temperature;

int16_t  bmp_ac1, bmp_ac2, bmp_ac3;
uint16_t bmp_ac4, bmp_ac5, bmp_ac6;
int16_t  bmp_b1, bmp_b2;
int16_t  bmp_mb, bmp_mc, bmp_md;
int32_t  bmp_up, bmp_ut;

void baro_bmp_init( void ) {
  baro_bmp_status = BARO_BMP_UNINIT;
  /* read calibration values */
  bmp_trans.buf[0] = BMP085_EEPROM_AC1;
  I2CTransceive(BMP_I2C_DEV, bmp_trans, BMP085_SLAVE_ADDR, 1, 22);
}

void baro_bmp_periodic( void ) {
  if (baro_bmp_status == BARO_BMP_IDLE) {
    /* start temp measurement (once) */
    bmp_trans.buf[0] = BMP085_CTRL_REG;
    bmp_trans.buf[1] = BMP085_START_TEMP;
    I2CTransmit(BMP_I2C_DEV, bmp_trans, BMP085_SLAVE_ADDR, 2);
    baro_bmp_status = BARO_BMP_START_TEMP;
  }
  else if (baro_bmp_status == BARO_BMP_START_TEMP) {
    /* read temp measurement */
    bmp_trans.buf[0] = BMP085_DAT_MSB;
    I2CTransceive(BMP_I2C_DEV, bmp_trans, BMP085_SLAVE_ADDR, 1, 2);
    baro_bmp_status = BARO_BMP_READ_TEMP;
  }
  else if (baro_bmp_status == BARO_BMP_START_PRESS) {
    /* read press measurement */
    bmp_trans.buf[0] = BMP085_DAT_MSB;
    I2CTransceive(BMP_I2C_DEV, bmp_trans, BMP085_SLAVE_ADDR, 1, 3);
    baro_bmp_status = BARO_BMP_READ_PRESS;
  }
}

void baro_bmp_event( void ) {

  if (bmp_trans.status == I2CTransSuccess) {

    if (baro_bmp_status == BARO_BMP_UNINIT) {
      /* get calibration data */
      bmp_ac1 = (bmp_trans.buf[0] << 8) | bmp_trans.buf[1];
      bmp_ac2 = (bmp_trans.buf[2] << 8) | bmp_trans.buf[3];
      bmp_ac3 = (bmp_trans.buf[4] << 8) | bmp_trans.buf[5];
      bmp_ac4 = (bmp_trans.buf[6] << 8) | bmp_trans.buf[7];
      bmp_ac5 = (bmp_trans.buf[8] << 8) | bmp_trans.buf[9];
      bmp_ac6 = (bmp_trans.buf[10] << 8) | bmp_trans.buf[11];
      bmp_b1  = (bmp_trans.buf[12] << 8) | bmp_trans.buf[13];
      bmp_b2  = (bmp_trans.buf[14] << 8) | bmp_trans.buf[15];
      bmp_mb  = (bmp_trans.buf[16] << 8) | bmp_trans.buf[17];
      bmp_mc  = (bmp_trans.buf[18] << 8) | bmp_trans.buf[19];
      bmp_md  = (bmp_trans.buf[20] << 8) | bmp_trans.buf[21];
      baro_bmp_status = BARO_BMP_IDLE;
    }
    else if (baro_bmp_status == BARO_BMP_READ_TEMP) {
      /* get uncompensated temperature */
      bmp_ut = (bmp_trans.buf[0] << 8) | bmp_trans.buf[1];
      /* start high res pressure measurement */
      bmp_trans.buf[0] = BMP085_CTRL_REG;
      bmp_trans.buf[1] = BMP085_START_P3;
      I2CTransmit(BMP_I2C_DEV, bmp_trans, BMP085_SLAVE_ADDR, 2);
      baro_bmp_status = BARO_BMP_START_PRESS;
    }
    else if (baro_bmp_status == BARO_BMP_READ_PRESS) {
      int32_t  bmp_p, bmp_t;
      int32_t  bmp_x1, bmp_x2, bmp_x3;
      int32_t  bmp_b3, bmp_b5, bmp_b6;
      uint32_t bmp_b4, bmp_b7;

      /* get uncompensated pressure, oss=3 */
      bmp_up = (bmp_trans.buf[0] << 11) |
               (bmp_trans.buf[1] << 3)  |
               (bmp_trans.buf[2] >> 5);
      /* start temp measurement */
      bmp_trans.buf[0] = BMP085_CTRL_REG;
      bmp_trans.buf[1] = BMP085_START_TEMP;
      I2CTransmit(BMP_I2C_DEV, bmp_trans, BMP085_SLAVE_ADDR, 2);
      baro_bmp_status = BARO_BMP_START_TEMP;

      /* compensate temperature */
      bmp_x1 = (bmp_ut - bmp_ac6) * bmp_ac5 / (1<<15);
      bmp_x2 = bmp_mc * (1<<11) / (bmp_x1 + bmp_md);
      bmp_b5 = bmp_x1 + bmp_x2;
      bmp_t  = (bmp_b5 + 8) / (1<<4);

      /* compensate pressure */
      bmp_b6 = bmp_b5 - 4000;
      bmp_x1 = (bmp_b2 * (bmp_b6 * bmp_b6 / (1<<12))) / (1<<11);
      bmp_x2 = bmp_ac2 *bmp_b6 / (1<<11);
      bmp_x3 = bmp_x1 + bmp_x2;
      bmp_b3 = (((bmp_ac1 * 4 + bmp_x3) << 3) + 2) / 4;
      bmp_x1 = bmp_ac3 * bmp_b6 / (1<<13);
      bmp_x2 = (bmp_b1 * (bmp_b6 * bmp_b6 / (1<<12))) / (1<<16);
      bmp_x3 = ((bmp_x1 + bmp_x2) +2) / (1<<2);
      bmp_b4 = bmp_ac4 * (uint32_t)(bmp_x3 + 32768) / (1<<15);
      bmp_b7 = ((uint32_t)bmp_up - bmp_b3) * (50000>>3);
      if (bmp_b7 < 0x80000000)
        bmp_p = (bmp_b7 * 2) / bmp_b4;
      else
        bmp_p = (bmp_b7 / bmp_b4) * 2;
      bmp_x1 = (bmp_p / (1<<8)) * (bmp_p / (1<<8));
      bmp_x1 = (bmp_x1 * 3038) / (1<<16);
      bmp_x2 = (-7357 * bmp_p) / (1<<16);
      bmp_p = bmp_p + (bmp_x1 + bmp_x2 + 3791) / (1<<4);

      baro_bmp_temperature = bmp_t;
      baro_bmp_pressure = bmp_p;
#ifdef SENSOR_SYNC_SEND
      DOWNLINK_SEND_BMP_STATUS(DefaultChannel, &bmp_up, &bmp_ut, &bmp_p, &bmp_t);
#endif
    }
  }
}
