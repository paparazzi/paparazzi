/*
 * Copyright (C) 2019 Alexis Cornard <alexiscornard@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as publpshed by
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
 * @file peripherals/lps25h_i2c.c
 *
 * Driver for LPS25H barometer I2C.
 */

#include "peripherals/lps25h_i2c.h"
#include "std.h"
#include <stdio.h>
#include <math.h>

void lps25h_i2c_init(struct Lps25h_I2c *lps, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  lps->i2c_p = i2c_p;
  /* set i2c address */
  lps->i2c_trans.slave_addr = addr;
  lps->i2c_trans.status = I2CTransDone;
  /* set default config options */
  lps25h_set_default_config(&(lps->config));
  lps->initialized = false;
  lps->data_available =  false;
  lps->init_status = LPS25H_CONF_UNINIT;
}


static void lps25h_i2c_tx_reg(struct Lps25h_I2c *lps, uint8_t reg, uint8_t val)
{
  lps->i2c_trans.type = I2CTransTx;
  lps->i2c_trans.buf[0] = reg;
  lps->i2c_trans.buf[1] = val;
  lps->i2c_trans.len_r = 0;
  lps->i2c_trans.len_w = 2;
  i2c_submit(lps->i2c_p, &(lps->i2c_trans));
}

// Configuration function called once before normal use
static void lps25h_i2c_send_config(struct Lps25h_I2c *lps)
{
  switch (lps->init_status) {
    case LPS25H_CONF_CTRL1:
      lps25h_i2c_tx_reg(lps, LPS25H_CTRL_REG1, lps->config.ctrl1);
      lps->init_status++;
      break;
    case LPS25H_CONF_DONE:
      lps->initialized = true;
      lps->i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Start configuration if not already done
void lps25h_i2c_start_configure(struct Lps25h_I2c *lps)
{
  if (lps->init_status == LPS25H_CONF_UNINIT) {
    lps->init_status++;
    if (lps->i2c_trans.status == I2CTransSuccess || lps->i2c_trans.status == I2CTransDone) {
      lps25h_i2c_send_config(lps);
    }
  }
}

// Normal reading
void lps25h_i2c_read(struct Lps25h_I2c *lps)
{
  if (lps->initialized && lps->i2c_trans.status == I2CTransDone) {
    lps->i2c_trans.buf[0] = LPS25H_REG_OUT_XL | (1 << 7);
    lps->i2c_trans.type = I2CTransTxRx;
    lps->i2c_trans.len_r = 3;
    lps->i2c_trans.len_w = 1;
    i2c_submit(lps->i2c_p, &(lps->i2c_trans));
  }
}

#define Int32FromBuf(buf, idx) (int32_t)(int8_t)buf[idx+2] << 16 | (uint16_t)buf[idx+1] << 8 | buf[idx];

// The two following functions can be used to check data
float lps25h_readPressureMillibars(int32_t press)
{
  return (float)press / 4096;
}

float pressureToAltMeters(float pressure_mbar, float altimeter_setting_mbar){
  return (1-pow((pressure_mbar/altimeter_setting_mbar), 0.190263)) * 4430.8;
}

void lps25h_i2c_event(struct Lps25h_I2c *lps)
{
  if (lps->initialized) {
    if (lps->i2c_trans.status == I2CTransFailed) {
      lps->i2c_trans.status = I2CTransDone;
    } else if (lps->i2c_trans.status == I2CTransSuccess) {
      lps->data = Int32FromBuf(lps->i2c_trans.buf, 0)
      lps->data_available = true;
      lps->i2c_trans.status = I2CTransDone;
    }
  } else if (lps->init_status != LPS25H_CONF_UNINIT) { // Configuring but not yet initialized
    switch(lps->i2c_trans.status){
      case I2CTransFailed:
        lps->init_status--;
      case I2CTransSuccess:
      case I2CTransDone:
        lps25h_i2c_send_config(lps);
        if(lps->initialized)
          lps->i2c_trans.status = I2CTransDone;
        break;
      default:
        break;
    }
  }
}

