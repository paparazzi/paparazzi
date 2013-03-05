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

#define MPL_CONF_UNINIT   0
#define MPL_CONF_PT_DATA  1
#define MPL_CONF_CTRL1    2
#define MPL_CONF_DONE     3


// Data ready flag
volatile bool_t mpl3115_data_available;
// Data
uint32_t mpl3115_pressure;
int16_t mpl3115_temperature;
float mpl3115_alt;
// I2C transaction for reading and configuring
struct i2c_transaction mpl3115_trans;
// I2C transaction for conversion request
struct i2c_transaction mpl3115_req_trans;
// Init flag
bool_t mpl3115_initialized;
uint8_t mpl3115_init_status;

void mpl3115_init(void)
{
  mpl3115_trans.status = I2CTransDone;
  mpl3115_req_trans.status = I2CTransDone;
  mpl3115_initialized = FALSE;
  mpl3115_init_status = MPL_CONF_UNINIT;

  mpl3115_pressure = 0;
  mpl3115_temperature = 0;
  mpl3115_alt = 0.;
}

// Configuration function called once before normal use
static void mpl3115_send_config(void)
{
  switch (mpl3115_init_status) {
    case MPL_CONF_PT_DATA:
      mpl3115_trans.buf[0] = MPL3115_REG_PT_DATA_CFG;
      mpl3115_trans.buf[1] = MPL3115_PT_DATA_CFG;
      i2c_transmit(&MPL3115_I2C_DEV, &mpl3115_trans, MPL3115_I2C_ADDR, 2);
      mpl3115_init_status++;
      break;
    case MPL_CONF_CTRL1:
      mpl3115_trans.buf[0] = MPL3115_REG_CTRL_REG1;
      mpl3115_trans.buf[1] = MPL3115_CTRL_REG1;
      i2c_transmit(&MPL3115_I2C_DEV, &mpl3115_trans, MPL3115_I2C_ADDR, 2);
      mpl3115_init_status++;
      break;
    case MPL_CONF_DONE:
      mpl3115_initialized = TRUE;
      mpl3115_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Configure
void mpl3115_configure(void)
{
  if (mpl3115_init_status == MPL_CONF_UNINIT) {
    mpl3115_init_status++;
    if (mpl3115_trans.status == I2CTransSuccess || mpl3115_trans.status == I2CTransDone) {
      mpl3115_send_config();
    }
  }
}

// Normal reading
void mpl3115_read(void)
{
  // ask for a reading and then prepare next conversion
  if (mpl3115_initialized && mpl3115_trans.status == I2CTransDone) {
    mpl3115_trans.buf[0] = MPL3115_REG_STATUS;
    i2c_transceive(&MPL3115_I2C_DEV, &mpl3115_trans, MPL3115_I2C_ADDR, 1, 6);
    if (mpl3115_req_trans.status == I2CTransDone) {
      mpl3115_req_trans.buf[0] = MPL3115_REG_CTRL_REG1;
      mpl3115_req_trans.buf[1] = MPL3115_CTRL_REG1 | MPL3115_OST_BIT;
      i2c_transmit(&MPL3115_I2C_DEV, &mpl3115_req_trans, MPL3115_I2C_ADDR, 2);
    }
  }
}

void mpl3115_event(void)
{
  if (mpl3115_initialized) {
    if (mpl3115_trans.status == I2CTransFailed) {
      mpl3115_trans.status = I2CTransDone;
    }
    else if (mpl3115_trans.status == I2CTransSuccess) {
      // Successfull reading and new pressure data available
      if (mpl3115_trans.buf[0] & (1<<2)) {
#if MPL3115_RAW_OUTPUT
        // New data available
        mpl3115_pressure = ((uint32_t)mpl3115_trans.buf[1]<<16)|((uint16_t)mpl3115_trans.buf[2]<<8)|mpl3115_trans.buf[3];
        mpl3115_temperature = ((int16_t)mpl3115_trans.buf[4]<<8)|mpl3115_trans.buf[5];
        mpl3115_data_available = TRUE;
#else // Not in raw mode
#if MPL3115_ALT_MODE
        uint32_t tmp = ((uint32_t)mpl3115_trans.buf[1]<<16)|((uint16_t)mpl3115_trans.buf[2]<<8)|mpl3115_trans.buf[3];
        mpl3115_alt = (float)(tmp>>4)/(1<<4);
        tmp = ((int16_t)mpl3115_trans.buf[4]<<8)|mpl3115_trans.buf[5];
        mpl3115_temperature = (tmp>>4);
        mpl3115_data_available = TRUE;
#else // Pressure mode
        uint32_t tmp = ((uint32_t)mpl3115_trans.buf[1]<<16)|((uint16_t)mpl3115_trans.buf[2]<<8)|mpl3115_trans.buf[3];
        mpl3115_pressure = (tmp>>4);
        tmp = ((int16_t)mpl3115_trans.buf[4]<<8)|mpl3115_trans.buf[5];
        mpl3115_temperature = (tmp>>4);
        mpl3115_data_available = TRUE;
#endif // end alt mode
#endif // end raw mode
      }
      mpl3115_trans.status = I2CTransDone;
    }
  }
  else if (!mpl3115_initialized && mpl3115_init_status != MPL_CONF_UNINIT) { // Configuring
    if (mpl3115_trans.status == I2CTransSuccess || mpl3115_trans.status == I2CTransDone) {
      mpl3115_trans.status = I2CTransDone;
      mpl3115_send_config();
    }
    if (mpl3115_trans.status == I2CTransFailed) {
      mpl3115_init_status--;
      mpl3115_trans.status = I2CTransDone;
      mpl3115_send_config(); // Retry config (TODO max retry)
    }
  }
  if (mpl3115_req_trans.status == I2CTransSuccess || mpl3115_req_trans.status == I2CTransFailed) {
    mpl3115_req_trans.status = I2CTransDone;
  }
}

