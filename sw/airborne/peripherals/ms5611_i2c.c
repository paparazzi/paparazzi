/*
 * Copyright (C) 2011 Martin Mueller <martinmm@pfump.org>
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

/**
 * @file peripherals/ms5611_i2c.c
 * Measurement Specialties (Intersema) MS5611-01BA pressure/temperature sensor interface for I2C.
 *
 */


#include "peripherals/ms5611_i2c.h"


void ms5611_i2c_init(struct Ms5611_I2c *ms, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  ms->i2c_p = i2c_p;

  /* slave address */
  ms->i2c_trans.slave_addr = addr;
  /* set initial status: Success or Done */
  ms->i2c_trans.status = I2CTransDone;

  ms->data_available = FALSE;
  ms->initialized = FALSE;
  ms->status = MS5611_STATUS_UNINIT;
  ms->prom_cnt = 0;
}

void ms5611_i2c_start_configure(struct Ms5611_I2c *ms)
{
  if (ms->status == MS5611_STATUS_UNINIT) {
    ms->initialized = FALSE;
    ms->prom_cnt = 0;
    ms->i2c_trans.buf[0] = MS5611_SOFT_RESET;
    i2c_transmit(ms->i2c_p, &(ms->i2c_trans), ms->i2c_trans.slave_addr, 1);
    ms->status = MS5611_STATUS_RESET;
  }
}

void ms5611_i2c_start_conversion(struct Ms5611_I2c *ms)
{
  if (ms->status == MS5611_STATUS_IDLE &&
      ms->i2c_trans.status == I2CTransDone) {
    /* start D1 conversion */
    ms->i2c_trans.buf[0] = MS5611_START_CONV_D1;
    i2c_transmit(ms->i2c_p, &(ms->i2c_trans), ms->i2c_trans.slave_addr, 1);
    ms->status = MS5611_STATUS_CONV_D1;
  }
}

/**
 * Periodic function to ensure proper delay after triggering reset or conversion.
 * Should run at 100Hz max.
 * Typical conversion time is 8.22ms at max resolution.
 */
void ms5611_i2c_periodic_check(struct Ms5611_I2c *ms)
{
  switch (ms->status) {
    case MS5611_STATUS_RESET:
      ms->status = MS5611_STATUS_RESET_OK;
      break;
    case MS5611_STATUS_RESET_OK:
      if (ms->i2c_trans.status == I2CTransDone) {
        /* start getting prom data */
        ms->i2c_trans.buf[0] = MS5611_PROM_READ | (ms->prom_cnt << 1);
        i2c_transceive(ms->i2c_p, &(ms->i2c_trans), ms->i2c_trans.slave_addr, 1, 2);
        ms->status = MS5611_STATUS_PROM;
      }
      break;
    case MS5611_STATUS_CONV_D1:
      ms->status = MS5611_STATUS_CONV_D1_OK;
      break;
    case MS5611_STATUS_CONV_D1_OK:
      if (ms->i2c_trans.status == I2CTransDone) {
        /* read D1 adc */
        ms->i2c_trans.buf[0] = MS5611_ADC_READ;
        i2c_transceive(ms->i2c_p, &(ms->i2c_trans), ms->i2c_trans.slave_addr, 1, 3);
        ms->status = MS5611_STATUS_ADC_D1;
      }
      break;
    case MS5611_STATUS_CONV_D2:
      ms->status = MS5611_STATUS_CONV_D2_OK;
      break;
    case MS5611_STATUS_CONV_D2_OK:
      if (ms->i2c_trans.status == I2CTransDone) {
        /* read D2 adc */
        ms->i2c_trans.buf[0] = MS5611_ADC_READ;
        i2c_transceive(ms->i2c_p, &(ms->i2c_trans), ms->i2c_trans.slave_addr, 1, 3);
        ms->status = MS5611_STATUS_ADC_D2;
      }
      break;
    default:
      break;
  }
}

void ms5611_i2c_event(struct Ms5611_I2c *ms) {
  if (ms->initialized) {
    if (ms->i2c_trans.status == I2CTransFailed) {
      ms->status = MS5611_STATUS_IDLE;
      ms->i2c_trans.status = I2CTransDone;
    }
    else if (ms->i2c_trans.status == I2CTransSuccess) {
      // Successfull reading
      switch (ms->status) {

        case MS5611_STATUS_ADC_D1:
          /* read D1 (pressure) */
          ms->data.d1 = (ms->i2c_trans.buf[0] << 16) |
                        (ms->i2c_trans.buf[1] << 8) |
                         ms->i2c_trans.buf[2];
          ms->i2c_trans.status = I2CTransDone;
          if (ms->data.d1 == 0) {
            /* if value is zero, it was read to soon and is invalid, back to idle */
            ms->status = MS5611_STATUS_IDLE;
          }
          else {
            /* start D2 conversion */
            ms->i2c_trans.buf[0] = MS5611_START_CONV_D2;
            i2c_transmit(ms->i2c_p, &(ms->i2c_trans), ms->i2c_trans.slave_addr, 1);
            ms->status = MS5611_STATUS_CONV_D2;
          }
          break;

        case MS5611_STATUS_ADC_D2:
          /* read D2 (temperature) */
          ms->data.d2 = (ms->i2c_trans.buf[0] << 16) |
                        (ms->i2c_trans.buf[1] << 8) |
                         ms->i2c_trans.buf[2];
          ms->i2c_trans.status = I2CTransDone;
          if (ms->data.d2 == 0) {
            /* if value is zero, it was read to soon and is invalid, back to idle */
            ms->status = MS5611_STATUS_IDLE;
          }
          else {
            /* calculate temp and pressure from measurements and set available if valid */
            if (ms5611_calc(&(ms->data)))
              ms->data_available = TRUE;
            ms->status = MS5611_STATUS_IDLE;
          }
          break;

        default:
          ms->i2c_trans.status = I2CTransDone;
          break;
      }
    }
  }
  else if (ms->status != MS5611_STATUS_UNINIT) { // Configuring but not yet initialized
    switch (ms->i2c_trans.status) {

      case I2CTransFailed:
        /* try again */
        ms->status = MS5611_STATUS_UNINIT;
        ms->i2c_trans.status = I2CTransDone;
        break;

      case I2CTransSuccess:
        if (ms->status == MS5611_STATUS_PROM) {
          /* read prom data */
          ms->data.c[ms->prom_cnt++] = (ms->i2c_trans.buf[0] << 8) |
                                        ms->i2c_trans.buf[1];
          ms->i2c_trans.status = I2CTransDone;
          if (ms->prom_cnt < PROM_NB) {
            /* get next prom data */
            ms->i2c_trans.buf[0] = MS5611_PROM_READ | (ms->prom_cnt << 1);
            i2c_transceive(ms->i2c_p, &(ms->i2c_trans), ms->i2c_trans.slave_addr, 1, 2);
          }
          else {
            /* done reading prom, check prom crc */
            if (ms5611_prom_crc_ok(ms->data.c)) {
              ms->initialized = TRUE;
              ms->status = MS5611_STATUS_IDLE;
            }
            else {
              /* checksum error, try again */
              ms->status = MS5611_STATUS_UNINIT;
            }
          }
        }
        break;

      default:
        ms->i2c_trans.status = I2CTransDone;
        break;
    }
  }
}
