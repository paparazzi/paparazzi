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

/* Driver for LSM303DLx using I2C
 */

#include "peripherals/lsm303dl.extra_i2c.h"
#include "std.h"

#define LSM_CONF_UNINIT 0
#define LSM_CONF_REG1   1
#define LSM_CONF_REG4   2
#define LSM_CONF_REG2   3
#define LSM_CONF_DONE   4


// Data ready flag
volatile bool_t lsm303dl_data_available;
// Data vector
struct Int16Vect3 lsm303dl_data;
// I2C transaction structure
struct i2c_transaction lsm303dl_i2c_trans;
// Init flag
bool_t lsm303dl_initialized;
uint8_t lsm303dl_init_status;

// TODO IRQ handling

void lsm303dl_init(void)
{
  lsm303dl_i2c_trans.status = I2CTransDone;
  lsm303dl_i2c_trans.slave_addr = LSM303DL_ADDR;
  lsm303dl_initialized = FALSE;
  lsm303dl_init_status = LSM_CONF_UNINIT;
}

// Configuration function called once before normal use
static void lsm303dl_send_config(void)
{
  switch (lsm303dl_init_status) {
    case LSM_CONF_REG1:
      lsm303dl_i2c_trans.buf[0] = LSM303DL_REG_CTRL_REG1_A;
      lsm303dl_i2c_trans.buf[1] = LSM303DL_CTRL_REG1_A;
      I2CTransmit(LSM303DL_I2C_DEVICE, lsm303dl_i2c_trans, LSM303DL_I2C_ADDR, 2);
      lsm303dl_init_status++;
      break;
    case LSM_CONF_REG4:
      lsm303dl_i2c_trans.buf[0] = LSM303DL_REG_CTRL_REG4_A;
      lsm303dl_i2c_trans.buf[1] = LSM303DL_CTRL_REG4_A;
      I2CTransmit(LSM303DL_I2C_DEVICE, lsm303dl_i2c_trans, LSM303DL_I2C_ADDR, 2);
      lsm303dl_init_status++;
      break;
    case LSM_CONF_REG2:
      lsm303dl_i2c_trans.buf[0] = LSM303DL_REG_CTRL_REG2_A;
      lsm303dl_i2c_trans.buf[1] = LSM303DL_CTRL_REG2_A;
      I2CTransmit(LSM303DL_I2C_DEVICE, lsm303dl_i2c_trans, LSM303DL_I2C_ADDR, 2);
      lsm303dl_init_status++;
      break;
    case LSM_CONF_DONE:
      lsm303dl_initialized = TRUE;
      lsm303dl_i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Configure
void lsm303dl_configure(void)
{
  if (lsm303dl_init_status == LSM_CONF_UNINIT) {
    lsm303dl_init_status++;
    if (lsm303dl_i2c_trans.status == I2CTransSuccess || lsm303dl_i2c_trans.status == I2CTransDone) {
      lsm303dl_send_config();
    }
  }
}

// Normal reading
void lsm303dl_read(void)
{
  if (lsm303dl_initialized && lsm303dl_i2c_trans.status == I2CTransDone) {
    lsm303dl_i2c_trans.buf[0] = LSM303DL_REG_OUT_X_L_A;
    I2CTransceive(LSM303DL_I2C_DEVICE, lsm303dl_i2c_trans, LSM303DL_I2C_ADDR, 1, 6);
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void lsm303dl_event(void)
{
  if (lsm303dl_initialized) {
    if (lsm303dl_i2c_trans.status == I2CTransFailed) {
      lsm303dl_i2c_trans.status = I2CTransDone;
    }
    else if (lsm303dl_i2c_trans.status == I2CTransSuccess) {
    // Successfull reading
      lsm303dl_data.x = Int16FromBuf(lsm303dl_i2c_trans.buf,0);
      lsm303dl_data.y = Int16FromBuf(lsm303dl_i2c_trans.buf,2);
      lsm303dl_data.z = Int16FromBuf(lsm303dl_i2c_trans.buf,4);
      lsm303dl_data_available = TRUE;
      lsm303dl_i2c_trans.status = I2CTransDone;
    }
  }
  else if (!lsm303dl_initialized && lsm303dl_init_status != LSM303DL_CONF_UNINIT) { // Configuring
    if (lsm303dl_i2c_trans.status == I2CTransSuccess || lsm303dl_i2c_trans.status == I2CTransDone) {
      lsm303dl_i2c_trans.status = I2CTransDone;
      lsm303dl_send_config();
    }
    if (lsm303dl_i2c_trans.status == I2CTransFailed) {
      lsm303dl_init_status--;
      lsm303dl_i2c_trans.status = I2CTransDone;
      lsm303dl_send_config(); // Retry config (TODO max retry)
    }
  }
}


