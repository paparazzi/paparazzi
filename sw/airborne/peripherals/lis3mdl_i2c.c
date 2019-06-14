/*
 * Copyright (C) 2019 Alexis Cornard <alexiscornard@gmail.com>
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
 * @file peripherals/lis3mdl_i2c.c
 *
 * Driver for LIS3MDL magnetometer I2C.
 */

#include "peripherals/lis3mdl_i2c.h"
#include "std.h"
#include <stdio.h>


void lis3mdl_i2c_init(struct Lis3mdl_I2c *lis, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  lis->i2c_p = i2c_p;

  /* set i2c address */
  lis->i2c_trans.slave_addr = addr;
  lis->i2c_trans.status = I2CTransDone;
  /* set default config options */
  lis3mdl_set_default_config(&(lis->config));
  lis->initialized = false;
  lis->data_available =  false;
  lis->init_status = LIS3MDL_CONF_UNINIT;
}


static void lis3mdl_i2c_tx_reg(struct Lis3mdl_I2c *lis, uint8_t reg, uint8_t val)
{
  lis->i2c_trans.type = I2CTransTx;
  lis->i2c_trans.buf[0] = reg;
  lis->i2c_trans.buf[1] = val;
  lis->i2c_trans.len_r = 0;
  lis->i2c_trans.len_w = 2;
  i2c_submit(lis->i2c_p, &(lis->i2c_trans));
}

// Configuration function called once before normal use
static void lis3mdl_i2c_send_config(struct Lis3mdl_I2c *lis)
{
  switch (lis->init_status) {
    case LIS3MDL_CONF_CTRL1:
      lis3mdl_i2c_tx_reg(lis, LIS3MDL_CTRL_REG1, lis->config.ctrl1);
      lis->init_status++;
      break;
    case LIS3MDL_CONF_CTRL2:
      lis3mdl_i2c_tx_reg(lis, LIS3MDL_CTRL_REG2, lis->config.ctrl2);
      lis->init_status++;
    break;
    case LIS3MDL_CONF_CTRL3:
      lis3mdl_i2c_tx_reg(lis, LIS3MDL_CTRL_REG3, lis->config.ctrl3);
      lis->init_status++;
    break;
    case LIS3MDL_CONF_CTRL4:
      lis3mdl_i2c_tx_reg(lis, LIS3MDL_CTRL_REG4, lis->config.ctrl4);
      lis->init_status++;
    break;
    case LIS3MDL_CONF_DONE:
      lis->initialized = true;
      lis->i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Start configuration if not already done
void lis3mdl_i2c_start_configure(struct Lis3mdl_I2c *lis)
{
  if (lis->init_status == LIS3MDL_CONF_UNINIT) {
    lis->init_status++;
    if (lis->i2c_trans.status == I2CTransSuccess || lis->i2c_trans.status == I2CTransDone) {
      lis3mdl_i2c_send_config(lis);
    }
  }
}

// Normal reading
void lis3mdl_i2c_read(struct Lis3mdl_I2c *lis)
{
  if (lis->initialized && lis->i2c_trans.status == I2CTransDone) {
    lis->i2c_trans.buf[0] = LIS3MDL_REG_OUTX_L;
    lis->i2c_trans.type = I2CTransTxRx;
    lis->i2c_trans.len_r = 6;
    lis->i2c_trans.len_w = 1;
    i2c_submit(lis->i2c_p, &(lis->i2c_trans));
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void lis3mdl_i2c_event(struct Lis3mdl_I2c *lis)
{
  if (lis->initialized) {
    if (lis->i2c_trans.status == I2CTransFailed) {
      lis->i2c_trans.status = I2CTransDone;
    } else if (lis->i2c_trans.status == I2CTransSuccess) {
      // Successfull reading
      lis->data.vect.x = Int16FromBuf(lis->i2c_trans.buf, 0);
      lis->data.vect.y = Int16FromBuf(lis->i2c_trans.buf, 2);
      lis->data.vect.z = Int16FromBuf(lis->i2c_trans.buf, 4);

      lis->data_available = true;
      lis->i2c_trans.status = I2CTransDone;
    }
  } else if (lis->init_status != LIS3MDL_CONF_UNINIT) { // Configuring but not yet initialized
    switch(lis->i2c_trans.status){
      case I2CTransFailed:
        lis->init_status--;
      case I2CTransSuccess:
      case I2CTransDone:
        lis3mdl_i2c_send_config(lis);
        if(lis->initialized)
          lis->i2c_trans.status = I2CTransDone;
        break;
      default:
        break;
    }
  }
}

