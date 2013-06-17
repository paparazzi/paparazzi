/*
 * Copyright (C) 2011 Martin Mueller <martinmm@pfump.org>
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

/**
 * @file modules/sensors/baro_ms5611_i2c.c
 * Measurement Specialties (Intersema) MS5611-01BA pressure/temperature sensor interface for I2C.
 *
 */


#include "modules/sensors/baro_ms5611_i2c.h"

#include "mcu_periph/sys_time.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/nav.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#ifndef MS5611_I2C_DEV
#define MS5611_I2C_DEV i2c0
#endif

/* address can be 0xEC or 0xEE (CSB\ low = 0xEE) */
#ifndef MS5611_SLAVE_ADDR
#define MS5611_SLAVE_ADDR 0xEE
#endif


struct i2c_transaction ms5611_trans;
uint8_t ms5611_status;
uint16_t ms5611_c[PROM_NB];
uint32_t ms5611_d1, ms5611_d2;
int32_t prom_cnt;
float fbaroms, ftempms;
float baro_ms5611_alt;
bool_t baro_ms5611_enabled;
bool_t baro_ms5611_valid;
float baro_ms5611_r;
float baro_ms5611_sigma2;

static int8_t baro_ms5611_crc(uint16_t* prom) {
  int32_t i, j;
  uint32_t res = 0;
  uint8_t crc = prom[7] & 0xF;
  prom[7] &= 0xFF00;
  for (i = 0; i < 16; i++) {
    if (i & 1) res ^= ((prom[i>>1]) & 0x00FF);
    else res ^= (prom[i>>1]>>8);
    for (j = 8; j > 0; j--) {
      if (res & 0x8000) res ^= 0x1800;
      res <<= 1;
    }
  }
  prom[7] |= crc;
  if (crc == ((res >> 12) & 0xF)) return 0;
  else return -1;
}

void baro_ms5611_init(void) {
  baro_ms5611_enabled = TRUE;
  baro_ms5611_valid = FALSE;
  ms5611_status = MS5611_UNINIT;
  baro_ms5611_r = BARO_MS5611_R;
  baro_ms5611_sigma2 = BARO_MS5611_SIGMA2;
  prom_cnt = 0;
}

void baro_ms5611_periodic( void ) {
  if (sys_time.nb_sec > 1) {
    if (ms5611_status >= MS5611_IDLE) {
      /* start D1 conversion */
      ms5611_status = MS5611_CONV_D1;
      ms5611_trans.buf[0] = MS5611_START_CONV_D1;
      i2c_transmit(&MS5611_I2C_DEV, &ms5611_trans, MS5611_SLAVE_ADDR, 1);
      RunOnceEvery((4*30), DOWNLINK_SEND_MS5611_COEFF(DefaultChannel, DefaultDevice,
          &ms5611_c[0], &ms5611_c[1], &ms5611_c[2], &ms5611_c[3],
          &ms5611_c[4], &ms5611_c[5], &ms5611_c[6], &ms5611_c[7]));
    }
    else if (ms5611_status == MS5611_UNINIT) {
      /* reset sensor */
      ms5611_status = MS5611_RESET;
      ms5611_trans.buf[0] = MS5611_SOFT_RESET;
      i2c_transmit(&MS5611_I2C_DEV, &ms5611_trans, MS5611_SLAVE_ADDR, 1);
    }
    else if (ms5611_status == MS5611_RESET_OK) {
      /* start getting prom data */
      ms5611_status = MS5611_PROM;
      ms5611_trans.buf[0] = MS5611_PROM_READ | (prom_cnt << 1);
      i2c_transceive(&MS5611_I2C_DEV, &ms5611_trans, MS5611_SLAVE_ADDR, 1, 2);
    }
  }
}

void baro_ms5611_d1( void ) {
  if (sys_time.nb_sec > 1) {
    if (ms5611_status == MS5611_CONV_D1_OK) {
      /* read D1 adc */
      ms5611_status = MS5611_ADC_D1;
      ms5611_trans.buf[0] = MS5611_ADC_READ;
      i2c_transceive(&MS5611_I2C_DEV, &ms5611_trans, MS5611_SLAVE_ADDR, 1, 3);
    }
  }
}

void baro_ms5611_d2( void ) {
  if (sys_time.nb_sec > 1) {
    if (ms5611_status == MS5611_CONV_D2_OK) {
      /* read D2 adc */
      ms5611_status = MS5611_ADC_D2;
      ms5611_trans.buf[0] = MS5611_ADC_READ;
      i2c_transceive(&MS5611_I2C_DEV, &ms5611_trans, MS5611_SLAVE_ADDR, 1, 3);
    }
  }
}

void baro_ms5611_event( void ) {
  if (ms5611_trans.status == I2CTransSuccess) {
    switch (ms5611_status) {

    case MS5611_RESET:
      ms5611_status = MS5611_RESET_OK;
      ms5611_trans.status = I2CTransDone;
      break;

    case MS5611_PROM:
      /* read prom data */
      ms5611_c[prom_cnt++] = (ms5611_trans.buf[0] << 8) | ms5611_trans.buf[1];
      if (prom_cnt < PROM_NB) {
        /* get next prom data */
        ms5611_trans.buf[0] = MS5611_PROM_READ | (prom_cnt << 1);
        i2c_transceive(&MS5611_I2C_DEV, &ms5611_trans, MS5611_SLAVE_ADDR, 1, 2);
      }
      else {
        /* done reading prom */
        ms5611_trans.status = I2CTransDone;
        /* check prom crc */
        if (baro_ms5611_crc(ms5611_c) == 0) {
          DOWNLINK_SEND_MS5611_COEFF(DefaultChannel, DefaultDevice,
              &ms5611_c[0], &ms5611_c[1], &ms5611_c[2], &ms5611_c[3],
              &ms5611_c[4], &ms5611_c[5], &ms5611_c[6], &ms5611_c[7]);
          ms5611_status = MS5611_IDLE;
        }
        else {
          /* checksum error, try again */
          baro_ms5611_init();
        }
      }
      break;

    case MS5611_CONV_D1:
      ms5611_status = MS5611_CONV_D1_OK;
      ms5611_trans.status = I2CTransDone;
      break;

    case  MS5611_ADC_D1:
      /* read D1 (pressure) */
      ms5611_d1 = (ms5611_trans.buf[0] << 16) |
                  (ms5611_trans.buf[1] << 8) |
                  ms5611_trans.buf[2];
      /* start D2 conversion */
      ms5611_status = MS5611_CONV_D2;
      ms5611_trans.buf[0] = MS5611_START_CONV_D2;
      i2c_transmit(&MS5611_I2C_DEV, &ms5611_trans, MS5611_SLAVE_ADDR, 1);
      break;

    case MS5611_CONV_D2:
      ms5611_status = MS5611_CONV_D2_OK;
      ms5611_trans.status = I2CTransDone;
      break;

    case  MS5611_ADC_D2: {
      int64_t dt, baroms, tempms, off, sens, t2, off2, sens2;
      /* read D2 (temperature) */
      ms5611_d2 = (ms5611_trans.buf[0] << 16) |
                  (ms5611_trans.buf[1] << 8) |
                  ms5611_trans.buf[2];
      ms5611_status = MS5611_IDLE;
      ms5611_trans.status = I2CTransDone;

      /* difference between actual and ref temperature */
      dt = ms5611_d2 - (int64_t)ms5611_c[5] * (1<<8);
      /* actual temperature */
      tempms = 2000 + ((int64_t)dt * ms5611_c[6]) / (1<<23);
      /* offset at actual temperature */
      off = ((int64_t)ms5611_c[2] * (1<<16)) + ((int64_t)ms5611_c[4] * dt) / (1<<7);
      /* sensitivity at actual temperature */
      sens = ((int64_t)ms5611_c[1] * (1<<15)) + ((int64_t)ms5611_c[3] * dt) / (1<<8);
      /* second order temperature compensation */
      if (tempms < 2000) {
        t2 = (dt*dt) / (1<<31);
        off2 = 5 * ((int64_t)(tempms-2000)*(tempms-2000)) / (1<<1);
        sens2 = 5 * ((int64_t)(tempms-2000)*(tempms-2000)) / (1<<2);
        if (tempms < -1500) {
          off2 = off2 + 7 * (int64_t)(tempms+1500)*(tempms+1500);
          sens2 = sens2 + 11 * ((int64_t)(tempms+1500)*(tempms+1500)) / (1<<1);
        }
        tempms = tempms - t2;
        off = off - off2;
        sens = sens - sens2;
      }
      /* temperature compensated pressure */
      baroms = (((int64_t)ms5611_d1 * sens) / (1<<21) - off) / (1<<15);

      float tmp_float = baroms/101325.0; //pressure at sea level
      tmp_float = pow(tmp_float,0.190295); //eleva pressao ao expoente
      baro_ms5611_alt = 44330*(1.0-tmp_float); //altitude above MSL

      baro_ms5611_valid = TRUE;

#ifdef SENSOR_SYNC_SEND
      ftempms = tempms / 100.;
      fbaroms = baroms / 100.;
      DOWNLINK_SEND_BARO_MS5611(DefaultChannel, DefaultDevice,
                                &ms5611_d1, &ms5611_d2, &fbaroms, &ftempms);
#endif

      break;
    }

    default:
      ms5611_trans.status = I2CTransDone;
      break;
    }
  }
}
