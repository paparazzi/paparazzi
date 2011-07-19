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

/* Driver for ADXL345 using I2C
 */

#include "peripherals/adxl345.extra_i2c.h"
#include "std.h"

#define ADXL_CONF_UNINIT 0
#define ADXL_CONF_RATE   1
#define ADXL_CONF_POWER  2
#define ADXL_CONF_INT    3
#define ADXL_CONF_FORMAT 4
#define ADXL_CONF_DONE   5


// Data ready flag
volatile bool_t adxl345_data_available;
// Data vector
struct Int16Vect3 adxl345_data;
// I2C transaction structure
struct i2c_transaction adxl345_i2c_trans;
// Init flag
bool_t adxl345_initialized;
uint8_t adxl345_init_status;

// TODO IRQ handling

void adxl345_init(void)
{
  adxl345_i2c_trans.status = I2CTransDone;
  adxl345_i2c_trans.slave_addr = ADXL345_ADDR;
  adxl345_initialized = FALSE;
  adxl345_init_status = ADXL_CONF_UNINIT;
}

// Configuration function called once before normal use
static void adxl345_send_config(void)
{
  switch (adxl345_init_status) {
    case ADXL_CONF_RATE:
      adxl345_i2c_trans.buf[0] = ADXL345_REG_BW_RATE;
      adxl345_i2c_trans.buf[1] = ADXL345_BW_RATE;
      I2CTransmit(ADXL345_I2C_DEVICE, adxl345_i2c_trans, ADXL345_I2C_ADDR, 2);
      break;
    case ADXL_CONF_POWER:
      adxl345_i2c_trans.buf[0] = ADXL345_REG_POWER_CTL;
      adxl345_i2c_trans.buf[1] = ADXL345_POWER_CTL;
      I2CTransmit(ADXL345_I2C_DEVICE, adxl345_i2c_trans, ADXL345_I2C_ADDR, 2);
      break;
    case ADXL_CONF_INT:
      adxl345_i2c_trans.buf[0] = ADXL345_REG_INT_ENABLE;
      adxl345_i2c_trans.buf[1] = ADXL345_INT_ENABLE;
      I2CTransmit(ADXL345_I2C_DEVICE, adxl345_i2c_trans, ADXL345_I2C_ADDR, 2);
      break;
    case ADXL_CONF_FORMAT:
      adxl345_i2c_trans.buf[0] = ADXL345_REG_DATA_FORMAT;
      adxl345_i2c_trans.buf[1] = ADXL345_DATA_FORMAT;
      I2CTransmit(ADXL345_I2C_DEVICE, adxl345_i2c_trans, ADXL345_I2C_ADDR, 2);
      break;
    case ADXL_CONF_DONE:
      adxl345_initialized = TRUE;
      adxl345_i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

void adxl345_periodic(void)
{
  if (!adxl345_initialized) {
    // Configure
    if (adxl345_i2c_trans.status == I2CTransSuccess || adxl345_i2c_trans.status == I2CTransDone) {
      adxl345_init_status++;
      adxl345_send_config();
    }
    if (adxl345_i2c_trans.status == I2CTransFailed) {
      adxl345_send_config(); // Retry config
    }
  }
  else {
    // Normal reading
    if (adxl345_i2c_trans.status == I2CTransDone){
      adxl345_i2c_trans.buf[0] = ADXL345_REG_DATA_X0;
      I2CTransceive(ADXL345_I2C_DEVICE, adxl345_i2c_trans, ADXL345_I2C_ADDR, 1, 6);
    }
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void adxl345_event(void)
{
  if (adxl345_initialized) {
    if (adxl345_i2c_trans.status == I2CTransFailed) {
      adxl345_i2c_trans.status = I2CTransDone;
    }
    else if (adxl345_i2c_trans.status == I2CTransSuccess) {
    // Successfull reading
      adxl345_data.x = Int16FromBuf(adxl345_i2c_trans.buf,0);
      adxl345_data.y = Int16FromBuf(adxl345_i2c_trans.buf,2);
      adxl345_data.z = Int16FromBuf(adxl345_i2c_trans.buf,4);
      adxl345_data_available = TRUE;
      adxl345_i2c_trans.status = I2CTransDone;
    }
  }
}


