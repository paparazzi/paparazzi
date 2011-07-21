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

/* Driver for HMC5843 and HMC5883
 */

#include "peripherals/hmc58xx.h"
#include "std.h"

#define HMC_CONF_UNINIT 0
#define HMC_CONF_CRA    1
#define HMC_CONF_CRB    2
#define HMC_CONF_MODE   3
#define HMC_CONF_DONE   4


// Data ready flag
volatile bool_t hmc58xx_data_available;
// Data vector
struct Int16Vect3 hmc58xx_data;
// I2C transaction structure
struct i2c_transaction hmc58xx_i2c_trans;
// Init flag
bool_t hmc58xx_initialized;
uint8_t hmc58xx_init_status;

// TODO IRQ handling

void hmc58xx_init(void)
{
  hmc58xx_i2c_trans.status = I2CTransDone;
  hmc58xx_i2c_trans.slave_addr = HMC58XX_ADDR;
  hmc58xx_initialized = FALSE;
  hmc58xx_init_status = HMC_CONF_UNINIT;
}

// Configuration function called once before normal use
static void hmc58xx_send_config(void)
{
  switch (hmc58xx_init_status) {
    case HMC_CONF_CRA:
      hmc58xx_i2c_trans.buf[0] = HMC58XX_REG_CFGA;
      hmc58xx_i2c_trans.buf[1] = HMC58XX_CRA;
      I2CTransmit(HMC58XX_I2C_DEVICE, hmc58xx_i2c_trans, HMC58XX_ADDR, 2);
      break;
    case HMC_CONF_CRB:
      hmc58xx_i2c_trans.buf[0] = HMC58XX_REG_CFGB;
      hmc58xx_i2c_trans.buf[1] = HMC58XX_CRB;
      I2CTransmit(HMC58XX_I2C_DEVICE, hmc58xx_i2c_trans, HMC58XX_ADDR, 2);
      break;
    case HMC_CONF_MODE:
      hmc58xx_i2c_trans.buf[0] = HMC58XX_REG_MODE;
      hmc58xx_i2c_trans.buf[1] = HMC58XX_MD;
      I2CTransmit(HMC58XX_I2C_DEVICE, hmc58xx_i2c_trans, HMC58XX_ADDR, 2);
      break;
    case HMC_CONF_DONE:
      hmc58xx_initialized = TRUE;
      hmc58xx_i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

void hmc58xx_periodic(void)
{
  if (!hmc58xx_initialized) {
    // Configure
    if (hmc58xx_i2c_trans.status == I2CTransSuccess || hmc58xx_i2c_trans.status == I2CTransDone) {
      hmc58xx_init_status++;
      hmc58xx_send_config();
    }
    if (hmc58xx_i2c_trans.status == I2CTransFailed) {
      hmc58xx_send_config(); // Retry config
    }
  }
  else {
    // Normal reading
    if (hmc58xx_i2c_trans.status == I2CTransDone){
      hmc58xx_i2c_trans.buf[0] = HMC58XX_REG_DATXM;
      I2CTransceive(HMC58XX_I2C_DEVICE, hmc58xx_i2c_trans, HMC58XX_ADDR, 1, 6);
    }
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void hmc58xx_event(void)
{
  if (hmc58xx_initialized) {
    if (hmc58xx_i2c_trans.status == I2CTransFailed) {
      hmc58xx_i2c_trans.status = I2CTransDone;
    }
    else if (hmc58xx_i2c_trans.status == I2CTransSuccess) {
      hmc58xx_data.x = Int16FromBuf(hmc58xx_i2c_trans.buf,0);
      hmc58xx_data.y = Int16FromBuf(hmc58xx_i2c_trans.buf,4);
      hmc58xx_data.z = Int16FromBuf(hmc58xx_i2c_trans.buf,2);
      hmc58xx_data_available = TRUE;
      hmc58xx_i2c_trans.status = I2CTransDone;
    }
  }
}

