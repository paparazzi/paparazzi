/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file peripherals/lis3mdl.c
 *
 * PNI RM3100 3-axis magnetometer driver interface (I2C).
 */

#include "peripherals/rm3100.h"

#define RM3100_ADDR_POLL    0x00
#define RM3100_ADDR_CMM     0x01
#define RM3100_ADDR_CCX     0x04
#define RM3100_ADDR_CCY     0x06
#define RM3100_ADDR_CCZ     0x08
#define RM3100_ADDR_TMRC    0x0B
#define RM3100_ADDR_MX      0x24
#define RM3100_ADDR_MY      0x27
#define RM3100_ADDR_MZ      0x2A
#define RM3100_ADDR_BIST    0x33
#define RM3100_ADDR_STATUS  0x34
#define RM3100_ADDR_HSHAKE  0x35
#define RM3100_ADDR_REVID   0x36

#define RM3100_CCX_DEFAULT_MSB  0x00
#define RM3100_CCX_DEFAULT_LSB  0xC8
#define RM3100_CCY_DEFAULT_MSB  RM3100_CCX_DEFAULT_MSB
#define RM3100_CCY_DEFAULT_LSB  RM3100_CCX_DEFAULT_LSB
#define RM3100_CCZ_DEFAULT_MSB  RM3100_CCX_DEFAULT_MSB
#define RM3100_CCZ_DEFAULT_LSB  RM3100_CCX_DEFAULT_LSB
#define RM3100_CMM_DEFAULT      0x70	// No continuous mode
#define RM3100_CONTINUOUS_MODE  (1 << 0)
#define RM3100_POLLING_MODE     (0 << 0)
#define RM3100_BIST_SELFTEST    0x8F
#define RM3100_BIST_DEFAULT     0x00
#define RM3100_BIST_XYZ_OK      ((1 << 4) | (1 << 5) | (1 << 6))
#define RM3100_STATUS_DRDY      (1 << 7)
#define RM3100_POLL_XYZ         0x70
#define RM3100_RM3100_REVID     0x22

void rm3100_init(struct Rm3100 *mag, struct i2c_periph *i2c_p, uint8_t addr, uint8_t data_rate)
{
  /* set i2c_peripheral */
  mag->i2c_p = i2c_p;
  /* set i2c address */
  mag->i2c_trans.slave_addr = addr;
  mag->i2c_trans.status = I2CTransDone;
  /* store data rate */
  mag->data_rate = data_rate;

  mag->initialized = false;
  mag->status = RM3100_CONF_UNINIT;
  mag->data_available = false;
}

// Configure
void rm3100_configure(struct Rm3100 *mag)
{
  // Only configure when not busy
  if (mag->i2c_trans.status != I2CTransSuccess && mag->i2c_trans.status != I2CTransFailed
      && mag->i2c_trans.status != I2CTransDone) {
    return;
  }

  // Only when succesfull continue with next
  if (mag->i2c_trans.status == I2CTransSuccess) {
    mag->status++;
  }

  mag->i2c_trans.status = I2CTransDone;
  switch (mag->status) {

    case RM3100_CONF_UNINIT:
      /* prepare config request */
      mag->i2c_trans.buf[0] = RM3100_ADDR_CCX; // start at CCR X reg
      mag->i2c_trans.buf[1] = RM3100_CCX_DEFAULT_MSB;
      mag->i2c_trans.buf[2] = RM3100_CCX_DEFAULT_LSB;
      mag->i2c_trans.buf[3] = RM3100_CCY_DEFAULT_MSB;
      mag->i2c_trans.buf[4] = RM3100_CCY_DEFAULT_LSB;
      mag->i2c_trans.buf[5] = RM3100_CCZ_DEFAULT_MSB;
      mag->i2c_trans.buf[6] = RM3100_CCZ_DEFAULT_LSB;
      // send config request
      i2c_transmit(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 7);
      break;

    case RM3100_CONF_CCR_DONE:
      /* prepare config request */
      mag->i2c_trans.buf[0] = RM3100_ADDR_TMRC; // start at TMRC reg
      mag->i2c_trans.buf[1] = mag->data_rate;
      // send config request
      i2c_transmit(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 2);
      break;

    case RM3100_CONF_TMRC_DONE:
      /* prepare config request */
      mag->i2c_trans.buf[0] = RM3100_ADDR_CMM; // start at CMM reg
      mag->i2c_trans.buf[1] = RM3100_CONTINUOUS_MODE | RM3100_POLL_XYZ;
      // send config request
      i2c_transmit(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 2);
      break;

    case RM3100_CONF_CCM_DONE:
      mag->status = RM3100_STATUS_IDLE;
      mag->initialized = true;
      break;

    default:
      break;
  }
}

void rm3100_read(struct Rm3100 *mag)
{
  if (mag->status != RM3100_STATUS_IDLE) {
    return;
  }

  // get 3 x 3bytes data = 9
  mag->i2c_trans.buf[0] = RM3100_ADDR_MX;
  i2c_transceive(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 1, 9);
  mag->status = RM3100_STATUS_MEAS;
}

// Get raw value
// 24 bits signed 2's complement format
// return as a signed 32 bits integer
static int32_t rm3100_get_raw_from_buf(const volatile uint8_t *buf, uint8_t idx)
{
  uint32_t raw = (uint32_t)((buf[idx] << 16) | (buf[idx+1] << 8) | buf[idx+2]);
  uint8_t shift = 32 - 24;
  return (int32_t)(raw << shift) >> shift;
}

void rm3100_event(struct Rm3100 *mag)
{
  if (!mag->initialized) {
    return;
  }

  switch (mag->status) {

    case RM3100_STATUS_MEAS:
      if (mag->i2c_trans.status == I2CTransSuccess) {
        // Copy the data
        mag->data.vect.x = rm3100_get_raw_from_buf(mag->i2c_trans.buf, 0);
        mag->data.vect.y = rm3100_get_raw_from_buf(mag->i2c_trans.buf, 3);
        mag->data.vect.z = rm3100_get_raw_from_buf(mag->i2c_trans.buf, 6);
        mag->data_available = true;
        // End reading, back to idle
        mag->status = RM3100_STATUS_IDLE;
      }
      break;

    default:
      if (mag->i2c_trans.status == I2CTransSuccess || mag->i2c_trans.status == I2CTransFailed) {
        // Goto idle
        mag->i2c_trans.status = I2CTransDone;
        mag->status = RM3100_STATUS_IDLE;
      }
      break;
  }
}

