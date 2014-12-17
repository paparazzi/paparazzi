/*
 *
 * Copyright (C) 2011 Gautier Hattenberger
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

/** @file peripherals/mpl3115.c
 *  Driver for MPL3115A2 baro sensor.
 */

#include "peripherals/mpl3115.h"
#include "std.h"


void mpl3115_init(struct Mpl3115 *mpl, struct i2c_periph *i2c_p, uint8_t addr)
{

  /* set i2c_peripheral */
  mpl->i2c_p = i2c_p;

  /* slave address */
  mpl->trans.slave_addr = addr;

  mpl->trans.status = I2CTransDone;
  mpl->req_trans.status = I2CTransDone;
  mpl->initialized = FALSE;
  mpl->init_status = MPL_CONF_UNINIT;

  /* by default disable raw mode and set pressure mode */
  mpl->raw_mode = FALSE;
  mpl->alt_mode = FALSE;

  mpl->pressure = 0;
  mpl->temperature = 0;
  mpl->altitude = 0.;
}

// Configuration function called once before normal use
static void mpl3115_send_config(struct Mpl3115 *mpl)
{
  switch (mpl->init_status) {
    case MPL_CONF_PT_DATA:
      mpl->trans.buf[0] = MPL3115_REG_PT_DATA_CFG;
      mpl->trans.buf[1] = MPL3115_PT_DATA_CFG;
      i2c_transmit(mpl->i2c_p, &mpl->trans, mpl->trans.slave_addr, 2);
      mpl->init_status++;
      break;
    case MPL_CONF_CTRL1:
      mpl->trans.buf[0] = MPL3115_REG_CTRL_REG1;
      mpl->trans.buf[1] = ((MPL3115_OVERSAMPLING << 3) | (mpl->raw_mode << 6) |
                           (mpl->alt_mode << 7));
      i2c_transmit(mpl->i2c_p, &mpl->trans, mpl->trans.slave_addr, 2);
      mpl->init_status++;
      break;
    case MPL_CONF_DONE:
      mpl->initialized = TRUE;
      mpl->trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Configure
void mpl3115_configure(struct Mpl3115 *mpl)
{
  if (mpl->init_status == MPL_CONF_UNINIT) {
    mpl->init_status++;
    if (mpl->trans.status == I2CTransSuccess || mpl->trans.status == I2CTransDone) {
      mpl3115_send_config(mpl);
    }
  }
}

// Normal reading
void mpl3115_read(struct Mpl3115 *mpl)
{
  // ask for a reading and then prepare next conversion
  if (mpl->initialized && mpl->trans.status == I2CTransDone) {
    mpl->trans.buf[0] = MPL3115_REG_STATUS;
    i2c_transceive(mpl->i2c_p, &mpl->trans, mpl->trans.slave_addr, 1, 6);
    if (mpl->req_trans.status == I2CTransDone) {
      mpl->req_trans.buf[0] = MPL3115_REG_CTRL_REG1;
      mpl->req_trans.buf[1] = ((MPL3115_OVERSAMPLING << 3) | (mpl->raw_mode << 6) |
                               (mpl->alt_mode << 7) | MPL3115_OST_BIT);
      i2c_transmit(mpl->i2c_p, &mpl->req_trans, mpl->trans.slave_addr, 2);
    }
  }
}

void mpl3115_event(struct Mpl3115 *mpl)
{
  if (mpl->initialized) {
    if (mpl->trans.status == I2CTransFailed) {
      mpl->trans.status = I2CTransDone;
    } else if (mpl->trans.status == I2CTransSuccess) {
      // Successfull reading and new pressure data available
      if (mpl->trans.buf[0] & (1 << 2)) {
        if (mpl->raw_mode) {
          // New data available
          mpl->pressure = (((uint32_t)mpl->trans.buf[1] << 16) |
                           ((uint16_t)mpl->trans.buf[2] << 8) |
                           mpl->trans.buf[3]);
          mpl->temperature = ((int16_t)mpl->trans.buf[4] << 8) | mpl->trans.buf[5];
        } else { // not in raw mode
          uint32_t tmp = (((uint32_t)mpl->trans.buf[1] << 16) |
                          ((uint16_t)mpl->trans.buf[2] << 8) |
                          mpl->trans.buf[3]);
          if (mpl->alt_mode) {
            mpl->altitude = (float)(tmp >> 4) / (1 << 4);
          } else { // Pressure mode
            mpl->pressure = (tmp >> 4);
          }
          tmp = ((int16_t)mpl->trans.buf[4] << 8) | mpl->trans.buf[5];
          mpl->temperature = (tmp >> 4);
        }
        mpl->data_available = TRUE;
      }
      mpl->trans.status = I2CTransDone;
    }
  } else if (!mpl->initialized && mpl->init_status != MPL_CONF_UNINIT) { // Configuring
    if (mpl->trans.status == I2CTransSuccess || mpl->trans.status == I2CTransDone) {
      mpl->trans.status = I2CTransDone;
      mpl3115_send_config(mpl);
    }
    if (mpl->trans.status == I2CTransFailed) {
      mpl->init_status--;
      mpl->trans.status = I2CTransDone;
      mpl3115_send_config(mpl); // Retry config (TODO max retry)
    }
  }
  if (mpl->req_trans.status == I2CTransSuccess || mpl->req_trans.status == I2CTransFailed) {
    mpl->req_trans.status = I2CTransDone;
  }
}

void mpl3115_periodic(struct Mpl3115 *mpl)
{
  if (mpl->initialized) {
    mpl3115_read(mpl);
  } else {
    mpl3115_configure(mpl);
  }
}
