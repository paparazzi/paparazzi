/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file peripherals/ak8963.c
 *
 * Driver for the AKM AK8963 magnetometer.
 */

#include "peripherals/ak8963.h"
#include "std.h"

/**
 * Initialize AK8963 struct
 */
void ak8963_init(struct Ak8963 *ak, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  ak->i2c_p = i2c_p;
  /* set i2c address */
  ak->i2c_trans.slave_addr = addr;
  ak->i2c_trans.status = I2CTransDone;
  ak->initialized = FALSE;
  ak->init_status = AK_CONF_UNINIT;
  ak->data_available = FALSE;
}

void ak8963_configure(struct Ak8963 *ak)
{
  // Only configure when not busy
  if (ak->i2c_trans.status != I2CTransSuccess && ak->i2c_trans.status != I2CTransFailed && ak->i2c_trans.status != I2CTransDone) {
    return;
  }

  // Only when succesfull continue with next
  if (ak->i2c_trans.status == I2CTransSuccess) {
    ak->init_status++;
  }

  ak->i2c_trans.status = I2CTransDone;
  switch (ak->init_status) {

      // Soft Reset the device
    case AK_CONF_UNINIT:
      ak->i2c_trans.buf[0] = AK8963_REG_CNTL2;
      ak->i2c_trans.buf[1] = 1;
      i2c_transmit(ak->i2c_p, &(ak->i2c_trans), ak->i2c_trans.slave_addr, 2);
      break;

      // Set it to continious measuring mode 2
    case AK_CONF_MODE:
      ak->i2c_trans.buf[0] = AK8963_REG_CNTL1;
      ak->i2c_trans.buf[1] = AK8963_CNTL1_CM_2;
      i2c_transmit(ak->i2c_p, &(ak->i2c_trans), ak->i2c_trans.slave_addr, 2);
      break;

      // Initialization done
    default:
      ak->initialized = TRUE;
      break;
  }
}

void ak8963_read(struct Ak8963 *ak)
{
  if (ak->status != AK_STATUS_IDLE) {
    return;
  }

  // Read the status register
  ak->i2c_trans.buf[0] = AK8963_REG_ST1;
  i2c_transceive(ak->i2c_p, &(ak->i2c_trans), ak->i2c_trans.slave_addr, 1, 1);
}

#define Int16FromBuf(_buf,_idx) ((int16_t)(_buf[_idx] | (_buf[_idx+1] << 8)))
void ak8963_event(struct Ak8963 *ak)
{
  if (!ak->initialized) {
    return;
  }

  switch (ak->status) {
    case AK_STATUS_IDLE:
      // When DRDY start reading
      if (ak->i2c_trans.status == I2CTransSuccess && ak->i2c_trans.buf[0] & 1) {
        ak->i2c_trans.buf[0] = AK8963_REG_HXL;
        i2c_transceive(ak->i2c_p, &(ak->i2c_trans), ak->i2c_trans.slave_addr, 1, 6);
        ak->status++;
      }
      break;

    case AK_STATUS_READ:
      if (ak->i2c_trans.status == I2CTransSuccess) {
        // Copy the data
        ak->data.vect.x = Int16FromBuf(ak->i2c_trans.buf, 0);
        ak->data.vect.y = Int16FromBuf(ak->i2c_trans.buf, 2);
        ak->data.vect.z = Int16FromBuf(ak->i2c_trans.buf, 4);
        ak->data_available = TRUE;

        // Read second status register to be ready for reading again
        ak->i2c_trans.buf[0] = AK8963_REG_ST2;
        i2c_transceive(ak->i2c_p, &(ak->i2c_trans), ak->i2c_trans.slave_addr, 1, 1);
        ak->status++;
        break;
      }

      break;

    default:
      if (ak->i2c_trans.status == I2CTransSuccess || ak->i2c_trans.status == I2CTransFailed) {
        // Goto idle
        ak->i2c_trans.status = I2CTransDone;
        ak->status = AK_STATUS_IDLE;
        // check overrun
        //if (bit_is_set(ak->i2c_trans.buf[0], 3)) {
        //  ak->data_available = FALSE;
        //} else {
        //  ak->data_available = TRUE;
        //}
      }
      break;
  }
}

