/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * ST LIS3MDL 3-axis magnetometer driver interface (I2C).
 */

#include "peripherals/lis3mdl.h"

#define LIS3MDL_ENABLE_AUTO_INC   (0x1<<7)

#define LIS3MDL_STATUS_ZYXOR      0b10000000
#define LIS3MDL_STATUS_ZOR        0b01000000
#define LIS3MDL_STATUS_YOR        0b00100000
#define LIS3MDL_STATUS_XOR        0b00010000
#define LIS3MDL_STATUS_ZYXDA      0b00001000
#define LIS3MDL_STATUS_ZDA        0b00000100
#define LIS3MDL_STATUS_YDA        0b00000010
#define LIS3MDL_STATUS_XDA        0b00000001

#define LIS3MDL_DEVICE_ID         0b00111101

#define LIS3MDL_REG_WHO_AM_I      0x0F
#define LIS3MDL_REG_CTL_1         0x20
#define LIS3MDL_REG_CTL_2         0x21
#define LIS3MDL_REG_CTL_3         0x22
#define LIS3MDL_REG_CTL_4         0x23
#define LIS3MDL_REG_STATUS        0x27
#define LIS3MDL_REG_OUT_X_L       0x28
#define LIS3MDL_REG_OUT_X_H       0x29
#define LIS3MDL_REG_OUT_Y_L       0x2A
#define LIS3MDL_REG_OUT_Y_H       0x2B
#define LIS3MDL_REG_OUT_Z_L       0x2C
#define LIS3MDL_REG_OUT_Z_H       0x2D
#define LIS3MDL_REG_OUT_TEMP_L    0x2E
#define LIS3MDL_REG_OUT_TEMP_H    0x2F

#define LIS3MDL_REG_CTL_1_TEMP_EN 0b10000000
#define LIS3MDL_REG_CTL_2_RESET   0b00000100

void lis3mdl_init(struct Lis3mdl *mag, struct i2c_periph *i2c_p, uint8_t addr, uint8_t data_rate, uint8_t scale, uint8_t mode, uint8_t perf)
{
  /* set i2c_peripheral */
  mag->i2c_p = i2c_p;
  /* set i2c address */
  mag->i2c_trans.slave_addr = addr;
  mag->i2c_trans.status = I2CTransDone;

  /* prepare config request */
  mag->i2c_trans.buf[0] = LIS3MDL_REG_CTL_1 | LIS3MDL_ENABLE_AUTO_INC;
  mag->i2c_trans.buf[1] = (data_rate << 2) | (perf << 5);
  mag->i2c_trans.buf[2] = (scale << 5);
  mag->i2c_trans.buf[3] = mode;
  mag->i2c_trans.buf[4] = (perf << 2);
  mag->i2c_trans.buf[5] = 0;

  mag->initialized = false;
  mag->status = LIS3MDL_CONF_UNINIT;
  mag->data_available = false;
}

// Configure
void lis3mdl_configure(struct Lis3mdl *mag)
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

    case LIS3MDL_CONF_UNINIT:
      // send config request
      i2c_transmit(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 6);
      break;

    case LIS3MDL_CONF_REG:
      mag->status = LIS3MDL_STATUS_IDLE;
      mag->initialized = true;
      break;

    default:
      break;
  }
}

void lis3mdl_read(struct Lis3mdl *mag)
{
  if (mag->status != LIS3MDL_STATUS_IDLE) {
    return;
  }

  mag->i2c_trans.buf[0] = LIS3MDL_REG_STATUS | LIS3MDL_ENABLE_AUTO_INC;
  i2c_transceive(mag->i2c_p, &(mag->i2c_trans), mag->i2c_trans.slave_addr, 1, 7);
  mag->status = LIS3MDL_STATUS_MEAS;
}

// Get raw value
#define Int16FromBuf(_buf,_idx) ((int16_t)(_buf[_idx] | (_buf[_idx+1] << 8)))

void lis3mdl_event(struct Lis3mdl *mag)
{
  if (!mag->initialized) {
    return;
  }

  switch (mag->status) {

    case LIS3MDL_STATUS_MEAS:
      if (mag->i2c_trans.status == I2CTransSuccess) {
        // Read status and error bytes
        const bool dr = mag->i2c_trans.buf[0] & LIS3MDL_STATUS_ZYXDA; // data ready
        if (dr > 0) {
          // Copy the data
          mag->data.vect.x = Int16FromBuf(mag->i2c_trans.buf, 1);
          mag->data.vect.y = Int16FromBuf(mag->i2c_trans.buf, 3);
          mag->data.vect.z = Int16FromBuf(mag->i2c_trans.buf, 5);
          mag->data_available = true;
        }
        // End reading, back to idle
        mag->status = LIS3MDL_STATUS_IDLE;
      }
      break;

    default:
      if (mag->i2c_trans.status == I2CTransSuccess || mag->i2c_trans.status == I2CTransFailed) {
        // Goto idle
        mag->i2c_trans.status = I2CTransDone;
        mag->status = LIS3MDL_STATUS_IDLE;
      }
      break;
  }
}

