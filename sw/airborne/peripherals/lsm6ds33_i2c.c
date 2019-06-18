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
 * @file peripherals/lsm6ds33_i2c.c
 *
 * Driver for LSM6DS33 accelerometer and gyrometer using I2C.
 */

#include "peripherals/lsm6ds33_i2c.h"
#include "std.h"
#include <stdio.h>


void lsm6_i2c_init(struct Lsm6_I2c *lsm, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  lsm->i2c_p = i2c_p;
  /* set i2c address */
  lsm->i2c_trans.slave_addr = addr;
  lsm->i2c_trans.status = I2CTransDone;
  /* set default config options */
  lsm6_set_default_config(&(lsm->config));
  lsm->initialized = false;
  lsm->data_available =  false;
  lsm->init_status = LSM6_CONF_UNINIT;
}


static void lsm6_i2c_tx_reg(struct Lsm6_I2c *lsm, uint8_t reg, uint8_t val)
{
  lsm->i2c_trans.type = I2CTransTx;
  lsm->i2c_trans.buf[0] = reg;
  lsm->i2c_trans.buf[1] = val;
  lsm->i2c_trans.len_r = 0;
  lsm->i2c_trans.len_w = 2;
  i2c_submit(lsm->i2c_p, &(lsm->i2c_trans));
}

// Configuration function called once before normal use
static void lsm6_i2c_send_config(struct Lsm6_I2c *lsm)
{
  switch (lsm->init_status) {
    case LSM6_CONF_CTRL1_XL:
      lsm6_i2c_tx_reg(lsm, LSM6_REG_CTRL1_XL, lsm->config.xl);
      lsm->init_status++;
      break;
    case LSM6_CONF_CTRL2_G:
      lsm6_i2c_tx_reg(lsm, LSM6_REG_CTRL2_G, lsm->config.g);
      lsm->init_status++;
      break;
    case LSM6_CONF_CTRL3_C:
      lsm6_i2c_tx_reg(lsm, LSM6_REG_CTRL3_C, lsm->config.c);
      lsm->init_status++;
      break;
    case LSM6_CONF_CTRL3_ORIENT:
      lsm6_i2c_tx_reg(lsm, LSM6_REG_ORIENT_CFG_G, lsm->config.orient);
      lsm->init_status++;
      break;
    case LSM6_CONF_DONE:
      lsm->initialized = true;
      lsm->i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Start configuration if not already done
void lsm6_i2c_start_configure(struct Lsm6_I2c *lsm)
{
  if (lsm->init_status == LSM6_CONF_UNINIT) {
    lsm->init_status++;
    if (lsm->i2c_trans.status == I2CTransSuccess || lsm->i2c_trans.status == I2CTransDone) {
      lsm6_i2c_send_config(lsm);
    }
  }
}

// Normal reading
void lsm6_i2c_read(struct Lsm6_I2c *lsm)
{
  if (lsm->initialized && lsm->i2c_trans.status == I2CTransDone) {
    lsm->i2c_trans.buf[0] = LSM6_REG_OUTX_L_G;
    lsm->i2c_trans.type = I2CTransTxRx;
    lsm->i2c_trans.len_r = 12;
    lsm->i2c_trans.len_w = 1;
    i2c_submit(lsm->i2c_p, &(lsm->i2c_trans));
  }
}



#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void lsm6_i2c_event(struct Lsm6_I2c *lsm)
{
  if (lsm->initialized) {
    if (lsm->i2c_trans.status == I2CTransFailed) {
      lsm->i2c_trans.status = I2CTransDone;
    } else if (lsm->i2c_trans.status == I2CTransSuccess) {
      // Successfull reading
      lsm->data_xl.vect.x = Int16FromBuf(lsm->i2c_trans.buf, 6);
      lsm->data_xl.vect.y = Int16FromBuf(lsm->i2c_trans.buf, 8);
      lsm->data_xl.vect.z = Int16FromBuf(lsm->i2c_trans.buf, 10);
      lsm->data_g.rates.p = Int16FromBuf(lsm->i2c_trans.buf, 0);
      lsm->data_g.rates.q = Int16FromBuf(lsm->i2c_trans.buf, 2);
      lsm->data_g.rates.r = Int16FromBuf(lsm->i2c_trans.buf, 4);
      lsm->data_available = true;
      lsm->i2c_trans.status = I2CTransDone;
    }
  } else if (lsm->init_status != LSM6_CONF_UNINIT) { // Configuring but not yet initialized
    switch(lsm->i2c_trans.status){
      case I2CTransFailed:
        lsm->init_status--;
      case I2CTransSuccess:
      case I2CTransDone:
        lsm6_i2c_send_config(lsm);
        if(lsm->initialized)
          lsm->i2c_trans.status = I2CTransDone;
        break;
      default:
        break;
    }
  }
}

