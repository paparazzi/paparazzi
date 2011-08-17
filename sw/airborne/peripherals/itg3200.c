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

/* Driver for ITG3200
 */

#include "peripherals/itg3200.extra.h"
#include "std.h"

#define ITG_CONF_UNINIT 0
#define ITG_CONF_SD     1
#define ITG_CONF_DF     2
#define ITG_CONF_INT    3
#define ITG_CONF_PWR    4
#define ITG_CONF_DONE   5


// Data ready flag
volatile bool_t itg3200_data_available;
// Data vector
struct Int32Rates itg3200_data;
// I2C transaction structure
struct i2c_transaction itg3200_i2c_trans;
// Init flag
bool_t itg3200_initialized;
uint8_t itg3200_init_status;

// TODO IRQ handling

void itg3200_init(void)
{
  itg3200_i2c_trans.status = I2CTransDone;
  itg3200_i2c_trans.slave_addr = ITG3200_ADDR;
  itg3200_initialized = FALSE;
  itg3200_init_status = ITG_CONF_UNINIT;
}

// Configuration function called once before normal use
static void itg3200_send_config(void)
{
  switch (itg3200_init_status) {
    case ITG_CONF_SD:
      itg3200_i2c_trans.buf[0] = ITG3200_REG_SMPLRT_DIV;
      itg3200_i2c_trans.buf[1] = ITG3200_SMPLRT_DIV;
      I2CTransmit(ITG3200_I2C_DEVICE, itg3200_i2c_trans, ITG3200_I2C_ADDR, 2);
      break;
    case ITG_CONF_DF:
      itg3200_i2c_trans.buf[0] = ITG3200_REG_DLPF_FS;
      itg3200_i2c_trans.buf[1] = ITG3200_DLPF_FS;
      I2CTransmit(ITG3200_I2C_DEVICE, itg3200_i2c_trans, ITG3200_I2C_ADDR, 2);
      break;
    case ITG_CONF_INT:
      itg3200_i2c_trans.buf[0] = ITG3200_REG_INT_CFG;
      itg3200_i2c_trans.buf[1] = ITG3200_INT_CFG;
      I2CTransmit(ITG3200_I2C_DEVICE, itg3200_i2c_trans, ITG3200_I2C_ADDR, 2);
      break;
    case ITG_CONF_PWR:
      itg3200_i2c_trans.buf[0] = ITG3200_REG_PWR_MGM;
      itg3200_i2c_trans.buf[1] = ITG3200_PWR_MGM;
      I2CTransmit(ITG3200_I2C_DEVICE, itg3200_i2c_trans, ITG3200_I2C_ADDR, 2);
      break;
    case ITG_CONF_DONE:
      itg3200_initialized = TRUE;
      itg3200_i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

void itg3200_periodic(void)
{
  if (!itg3200_initialized) {
    // Configure
    if (itg3200_i2c_trans.status == I2CTransSuccess || itg3200_i2c_trans.status == I2CTransDone) {
      itg3200_init_status++;
      itg3200_send_config();
    }
    if (itg3200_i2c_trans.status == I2CTransFailed) {
      itg3200_send_config(); // Retry config
    }
  }
  else {
    // Normal reading
    if (itg3200_i2c_trans.status == I2CTransDone) {
      itg3200_i2c_trans.buf[0] = ITG3200_REG_INT_STATUS;
      I2CTransceive(ITG3200_I2C_DEVICE, itg3200_i2c_trans, ITG3200_I2C_ADDR, 1, 9);
    }
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void itg3200_event(void)
{
  if (itg3200_initialized) {
    if (itg3200_i2c_trans.status == I2CTransFailed) {
      itg3200_i2c_trans.status = I2CTransDone;
    }
    else if (itg3200_i2c_trans.status == I2CTransSuccess) {
      // Successfull reading and new data available
      if (itg3200_i2c_trans.buf[0] & 0x01) {
        // New data available
        itg3200_data.p = Int16FromBuf(itg3200_i2c_trans.buf,3);
        itg3200_data.q = Int16FromBuf(itg3200_i2c_trans.buf,5);
        itg3200_data.r = Int16FromBuf(itg3200_i2c_trans.buf,7);
        itg3200_data_available = TRUE;
      }
      itg3200_i2c_trans.status = I2CTransDone;
    }
  }
}

