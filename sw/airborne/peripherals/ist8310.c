/*
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file peripherals/ist8310.c
 *
 * Driver for the Isentek IST8310 magnetometer.
 */

#include "peripherals/ist8310.h"
#include "std.h"

/**
 * Initialize IST8310 struct
 */
void ist8310_init(struct IST8310 *ist, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  ist->i2c_p = i2c_p;
  /* set i2c address */
  ist->i2c_trans.slave_addr = addr;
  ist->i2c_trans.status = I2CTransDone;
  ist->initialized = false;
  ist->init_status = IST_CONF_UNINIT;
  ist->data_available = false;
}

void ist8310_configure(struct IST8310 *ist)
{
  // Only configure when not busy
  if (ist->i2c_trans.status != I2CTransSuccess && ist->i2c_trans.status != I2CTransFailed
      && ist->i2c_trans.status != I2CTransDone) {
    return;
  }

  // Only when succesfull continue with next
  if (ist->i2c_trans.status == I2CTransSuccess) {
    ist->init_status++;
  }

  ist->i2c_trans.status = I2CTransDone;
  switch (ist->init_status) {

    // Reset the device
    case IST_CONF_UNINIT:
      //ist->i2c_trans.buf[0] = IST8310_REG_CNTL2;
      //ist->i2c_trans.buf[1] = IST8310_CNTL2_DREN | IST8310_CNTL2_DRP;
      //i2c_transmit(ist->i2c_p, &(ist->i2c_trans), ist->i2c_trans.slave_addr, 2);
      ist->i2c_trans.status = I2CTransSuccess;
      break;

    // Configure for 50 Hz output
    case IST_CONF_CNTL1:
      ist->i2c_trans.buf[0] = IST8310_REG_CNTL1;
      ist->i2c_trans.buf[1] = IST8310_CNTL1_ODR_SINGLE;
      i2c_transmit(ist->i2c_p, &(ist->i2c_trans), ist->i2c_trans.slave_addr, 2);
      break;

    // Average 16 samples
    case IST_CONF_CNTL3:
      ist->i2c_trans.buf[0] = IST8310_REG_CNTL3;
      ist->i2c_trans.buf[1] = IST8310_CNTL3_SAMPAVG_16;
      i2c_transmit(ist->i2c_p, &(ist->i2c_trans), ist->i2c_trans.slave_addr, 2);
      break;

    // Set reset pulse duration to normal
    case IST_CONF_CNTL4:
      ist->i2c_trans.buf[0] = IST8310_REG_CNTL4;
      ist->i2c_trans.buf[1] = IST8310_CNTL4_SRPD;
      i2c_transmit(ist->i2c_p, &(ist->i2c_trans), ist->i2c_trans.slave_addr, 2);
      break;

    // Initialization done
    default:
      ist->initialized = true;
      break;
  }
}

void ist8310_read(struct IST8310 *ist)
{
  if (ist->status != IST_STATUS_IDLE) {
    return;
  }

  // Start a single read
  ist->i2c_trans.buf[0] = IST8310_REG_CNTL1;
  ist->i2c_trans.buf[1] = IST8310_CNTL1_ODR_SINGLE;
  i2c_transceive(ist->i2c_p, &(ist->i2c_trans), ist->i2c_trans.slave_addr, 2, 0);
}

#define Int16FromBuf(_buf,_idx) ((int16_t)(_buf[_idx] | (_buf[_idx+1] << 8)))
void ist8310_event(struct IST8310 *ist)
{
  if (!ist->initialized) {
    return;
  }

  switch (ist->status) {
    case IST_STATUS_IDLE:
      // When succesfully send single read
      if (ist->i2c_trans.status == I2CTransSuccess) {
        ist->i2c_trans.buf[0] = IST8310_REG_DATA_XL;
        i2c_transceive(ist->i2c_p, &(ist->i2c_trans), ist->i2c_trans.slave_addr, 1, 6);
        ist->status++;
      }
      break;

    case IST_STATUS_READ:
      if (ist->i2c_trans.status == I2CTransSuccess) {
        // Copy the data
        ist->data.vect.x = Int16FromBuf(ist->i2c_trans.buf, 0);
        ist->data.vect.y = Int16FromBuf(ist->i2c_trans.buf, 2);
        ist->data.vect.z = Int16FromBuf(ist->i2c_trans.buf, 4);
        ist->data_available = true;

        ist->i2c_trans.status = I2CTransDone;
        ist->status = IST_STATUS_IDLE;
      } else if(ist->i2c_trans.status == I2CTransFailed) {
        ist->i2c_trans.status = I2CTransDone;
        ist->status = IST_STATUS_IDLE;
      }
      break;

    default:
      // Goto idle
      ist->i2c_trans.status = I2CTransDone;
      ist->status = IST_STATUS_IDLE;
      break;
  }
}

