/*
 * Copyright (C) 2010 Martin Mueller
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 */

/** @file peripherals/bmp085.c
 *  Bosch BMP085 driver interface.
 */

#include "peripherals/bmp085.h"
#include "peripherals/bmp085_regs.h"


static int32_t bmp085_compensated_temperature(struct Bmp085Calib *calib, int32_t raw)
{
  int32_t x1 = (raw - calib->ac6) * calib->ac5 / (1 << 15);
  int32_t x2 = (calib->mc << 11) / (x1 + calib->md);
  calib->b5 = x1 + x2;
  return (calib->b5 + 8) >> 4;
}



/** Apply temp calibration and sensor calibration to raw measurement to get Pa
 * (from BMP085 datasheet)
 */
static int32_t bmp085_compensated_pressure(struct Bmp085Calib *calib, int32_t raw)
{
  int32_t b6 = calib->b5 - 4000;
  int32_t x1 = (calib->b2 * (b6 * b6 >> 12)) >> 11;
  int32_t x2 = calib->ac2 * b6 >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = (((calib->ac1 * 4 + x3) << BMP085_OSS) + 2) / 4;
  x1 = calib->ac3 * b6 >> 13;
  x2 = (calib->b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = (calib->ac4 * (uint32_t)(x3 + 32768)) >> 15;
  uint32_t b7 = (raw - b3) * (50000 >> BMP085_OSS);
  int32_t p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  return p + ((x1 + x2 + 3791) >> 4);
}

/**
 * Dummy function to always return TRUE on EndOfConversion check.
 * Ensure proper timing trough frequency of bmp085_periodic instead!
 */
static bool bmp085_eoc_true(void)
{
  return true;
}


void bmp085_read_eeprom_calib(struct Bmp085 *bmp)
{
  if (bmp->status == BMP085_STATUS_UNINIT && bmp->i2c_trans.status == I2CTransDone) {
    bmp->initialized = false;
    bmp->i2c_trans.buf[0] = BMP085_EEPROM_AC1;
    i2c_transceive(bmp->i2c_p, &(bmp->i2c_trans), bmp->i2c_trans.slave_addr, 1, 22);
  }
}


void bmp085_init(struct Bmp085 *bmp, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  bmp->i2c_p = i2c_p;

  /* slave address */
  bmp->i2c_trans.slave_addr = addr;
  /* set initial status: Success or Done */
  bmp->i2c_trans.status = I2CTransDone;

  bmp->data_available = false;
  bmp->initialized = false;
  bmp->status = BMP085_STATUS_UNINIT;

  /* by default assign EOC function that always returns TRUE
   * ensure proper timing through frequeny of bmp_periodic!
   */
  bmp->eoc = &bmp085_eoc_true;
}

/**
 * Start new measurement if idle or read temp/pressure.
 * Should run at < 40Hz unless eoc check function is provided.
 * At ultra high resolution (oss = 3) conversion time is max 25.5ms.
 */
void bmp085_periodic(struct Bmp085 *bmp)
{
  switch (bmp->status) {
    case BMP085_STATUS_IDLE:
      /* start temp measurement */
      bmp->i2c_trans.buf[0] = BMP085_CTRL_REG;
      bmp->i2c_trans.buf[1] = BMP085_START_TEMP;
      i2c_transmit(bmp->i2c_p, &bmp->i2c_trans, bmp->i2c_trans.slave_addr, 2);
      bmp->status = BMP085_STATUS_START_TEMP;
      break;

    case BMP085_STATUS_START_TEMP:
      /* read temp measurement */
      if (bmp->eoc()) {
        bmp->i2c_trans.buf[0] = BMP085_DAT_MSB;
        i2c_transceive(bmp->i2c_p, &bmp->i2c_trans, bmp->i2c_trans.slave_addr, 1, 2);
        bmp->status = BMP085_STATUS_READ_TEMP;
      }
      break;

    case BMP085_STATUS_START_PRESS:
      /* read press measurement */
      if (bmp->eoc()) {
        bmp->i2c_trans.buf[0] = BMP085_DAT_MSB;
        i2c_transceive(bmp->i2c_p, &bmp->i2c_trans, bmp->i2c_trans.slave_addr, 1, 3);
        bmp->status = BMP085_STATUS_READ_PRESS;
      }
      break;

    default:
      break;
  }
}

void bmp085_event(struct Bmp085 *bmp)
{
  if (bmp->i2c_trans.status == I2CTransSuccess) {
    switch (bmp->status) {
      case BMP085_STATUS_UNINIT:
        bmp->calib.ac1 = (bmp->i2c_trans.buf[0] << 8) | bmp->i2c_trans.buf[1];
        bmp->calib.ac2 = (bmp->i2c_trans.buf[2] << 8) | bmp->i2c_trans.buf[3];
        bmp->calib.ac3 = (bmp->i2c_trans.buf[4] << 8) | bmp->i2c_trans.buf[5];
        bmp->calib.ac4 = (bmp->i2c_trans.buf[6] << 8) | bmp->i2c_trans.buf[7];
        bmp->calib.ac5 = (bmp->i2c_trans.buf[8] << 8) | bmp->i2c_trans.buf[9];
        bmp->calib.ac6 = (bmp->i2c_trans.buf[10] << 8) | bmp->i2c_trans.buf[11];
        bmp->calib.b1  = (bmp->i2c_trans.buf[12] << 8) | bmp->i2c_trans.buf[13];
        bmp->calib.b2  = (bmp->i2c_trans.buf[14] << 8) | bmp->i2c_trans.buf[15];
        bmp->calib.mb  = (bmp->i2c_trans.buf[16] << 8) | bmp->i2c_trans.buf[17];
        bmp->calib.mc  = (bmp->i2c_trans.buf[18] << 8) | bmp->i2c_trans.buf[19];
        bmp->calib.md  = (bmp->i2c_trans.buf[20] << 8) | bmp->i2c_trans.buf[21];
        bmp->status = BMP085_STATUS_IDLE;
        bmp->initialized = true;
        break;

      case BMP085_STATUS_READ_TEMP:
        /* get uncompensated temperature */
        bmp->ut = (bmp->i2c_trans.buf[0] << 8) | bmp->i2c_trans.buf[1];
        bmp->temperature = bmp085_compensated_temperature(&(bmp->calib), bmp->ut);
        /* start high res pressure measurement */
        bmp->i2c_trans.buf[0] = BMP085_CTRL_REG;
        bmp->i2c_trans.buf[1] = BMP085_START_P3;
        i2c_transmit(bmp->i2c_p, &bmp->i2c_trans, bmp->i2c_trans.slave_addr, 2);
        bmp->status = BMP085_STATUS_START_PRESS;
        break;

      case BMP085_STATUS_READ_PRESS:
        /* get uncompensated pressure */
        bmp->up = (bmp->i2c_trans.buf[0] << 16) |
                  (bmp->i2c_trans.buf[1] << 8) |
                  bmp->i2c_trans.buf[2];
        bmp->up = bmp->up >> (8 - BMP085_OSS);
        bmp->pressure = bmp085_compensated_pressure(&(bmp->calib), bmp->up);
        bmp->data_available = true;
        bmp->status = BMP085_STATUS_IDLE;
        break;

      default:
        break;
    }
  } else if (bmp->i2c_trans.status == I2CTransFailed) {
    /* try again */
    if (bmp->initialized) {
      bmp->status = BMP085_STATUS_IDLE;
    } else {
      bmp->status = BMP085_STATUS_UNINIT;
    }
    bmp->i2c_trans.status = I2CTransDone;
  }
}
