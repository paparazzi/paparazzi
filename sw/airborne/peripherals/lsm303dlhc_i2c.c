/*
 * Copyright (C) 2011 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file peripherals/lsm303dlhc_i2c.c
 *
 * Driver for ST LSM303DLHC 3D accelerometer and magnetometer.
 * UNTESTED
 */

#include "peripherals/lsm303dlhc_i2c.h"
#include "std.h"

/**
 * Initialize Lsm303dlhc struct and set default config options.
 * @param lsm   Lsm303dlhc struct
 * @param i2c_p I2C peripheral to use
 * @param addr  I2C address of Lsm303dlhc
 */
void lsm303dlhc_i2c_init(struct Lsm303dlhc_i2c *lsm, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  lsm->i2c_p = i2c_p;
  /* set i2c address */
  lsm->i2c_trans.slave_addr = addr;
  lsm->i2c_trans.status = I2CTransDone;
  /* set default config options */
  if (addr == LSM303DLHC_ACC_ADDR) {
    lsm303dlhc_acc_set_default_config(&(lsm->config.acc));
    lsm->init_status.acc = LSM_CONF_ACC_UNINIT;
  } else {
    lsm303dlhc_mag_set_default_config(&(lsm->config.mag));
    lsm->init_status.mag = LSM_CONF_MAG_UNINIT;
  }
  lsm->initialized = false;
}

static void lsm303dlhc_i2c_tx_reg(struct Lsm303dlhc_I2c *lsm, uint8_t reg, uint8_t val)
{
  lsm->i2c_trans.type = I2CTransTx;
  lsm->i2c_trans.buf[0] = reg;
  lsm->i2c_trans.buf[1] = val;
  lsm->i2c_trans.len_r = 0;
  lsm->i2c_trans.len_w = 2;
  i2c_submit(lsm->i2c_p, &(lsm->i2c_trans));
}

/// Configuration function called once before normal use
static void lsm303dlhc_i2c_send_config(struct Lsm303dlhc_I2c *lsm)
{
  if (lsm->i2c_trans.slave_addr == LSM303DLHC_ACC_ADDR) {
    switch (lsm->init_status.acc) {
      case LSM_CONF_ACC_CTRL_REG4_A:
        lsm303dlhc_i2c_tx_reg(lsm, LSM303DLHC_REG_CTRL_REG4_A,
                              (lsm->config.acc.scale & LSM303DLHC_FS_MASK) |
                              (lsm->config.acc.hres & LSM303DLHC_DEFAULT_HR));
        lsm->init_status.acc++;
        break;
      case LSM_CONF_ACC_CTRL_REG1_A:
        lsm303dlhc_i2c_tx_reg(lsm, LSM303DLHC_REG_CTRL_REG1_A,
                              (lsm->config.acc.rate & LSM303DLHC_ODR_MASK) |
                              (lsm->config.acc.lp_mode & LSM303DLHC_LPen) |
                              LSM303DLHC_Xen | LSM303DLHC_Yen | LSM303DLHC_Zen);
        lsm->init_status.acc++;
        break;
      case LSM_CONF_ACC_CTRL_REG3_A:
        lsm303dlhc_i2c_tx_reg(lsm, LSM303DLHC_REG_CTRL_REG3_A, LSM303DLHC_I1_DRDY1);
        lsm->init_status.acc++;
        break;
      case LSM_CONF_ACC_DONE:
        lsm->initialized = true;
        lsm->i2c_trans.status = I2CTransDone;
        lsm303dlhc_i2c_read(lsm);
        break;
      default:
        break;
    }
  } else {
    switch (lsm->init_status.mag) {
      case LSM_CONF_MAG_CRA_REG_M:
        lsm303dlhc_i2c_tx_reg(lsm, LSM303DLHC_REG_CRA_REG_M, lsm->config.mag.rate);
        lsm->init_status.mag++;
        break;
      case LSM_CONF_MAG_CRB_REG_M:
        lsm303dlhc_i2c_tx_reg(lsm, LSM303DLHC_REG_CRB_REG_M, lsm->config.mag.gain);
        lsm->init_status.mag++;
        break;
      case LSM_CONF_MAG_MR_REG_M:
        lsm303dlhc_i2c_tx_reg(lsm, LSM303DLHC_REG_MR_REG_M, lsm->config.mag.mode);
        lsm->init_status.mag++;
        break;
      case LSM_CONF_MAG_DONE:
        lsm->initialized = true;
        lsm->i2c_trans.status = I2CTransDone;
        break;
      default:
        break;
    }
  }
}

// Configure
void lsm303dlhc_i2c_start_configure(struct Lsm303dlhc_I2c *lsm)
{
  if (lsm->i2c_trans.slave_addr == LSM303DLHC_ACC_ADDR) {
    if (lsm->init_status.acc == LSM_CONF_ACC_UNINIT) {
      lsm->init_status.acc++;
      if (lsm->i2c_trans.status == I2CTransSuccess || lsm->i2c_trans.status == I2CTransDone) {
        lsm303dlhc_send_config(lsm);
      }
    }
  } else {
    if (lsm->init_status.mag == LSM_CONF_MAG_UNINIT) {
      lsm->init_status.mag++;
      if (lsm->i2c_trans.status == I2CTransSuccess || lsm->i2c_trans.status == I2CTransDone) {
        lsm303dlhc_i2c_send_config(lsm);
      }
    }
  }
}

// Normal reading
void lsm303dlhc_i2c_read(struct Lsm303dlhc *lsm)
{
  if (lsm->i2c_trans.slave_addr == LSM303DLHC_ACC_ADDR) {
    //if ((lsm->init_status.acc == LSM_CONF_ACC_CLR_INT_READ) && (lsm->i2c_trans.status == I2CTransDone)){
    if (!(lsm->initialized) || (lsm->initialized && lsm->i2c_trans.status == I2CTransDone)) {
      lsm->i2c_trans.buf[0] = LSM303DLHC_REG_OUT_X_L_A | 0x80;
      lsm->i2c_trans.type = I2CTransTxRx;
      lsm->i2c_trans.len_r = 6;
      lsm->i2c_trans.len_w = 1;
      i2c_submit(lsm->i2c_p, &(lsm->i2c_trans));
    }
  } else {
    if (lsm->initialized && lsm->i2c_trans.status == I2CTransDone) {
      lsm->i2c_trans.buf[0] = LSM303DLHC_REG_OUT_X_H_M;
      lsm->i2c_trans.type = I2CTransTxRx;
      lsm->i2c_trans.len_r = 6;
      lsm->i2c_trans.len_w = 1;
      i2c_submit(lsm->i2c_p, &(lsm->i2c_trans));
    }
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void lsm303dlhc_i2c_event(struct Lsm303dlhc *lsm)
{
  if (lsm->initialized) {
    if (lsm->i2c_trans.status == I2CTransFailed) {
      lsm->i2c_trans.status = I2CTransDone;
    } else if (lsm->i2c_trans.status == I2CTransSuccess) {
      lsm->data.vect.x = Int16FromBuf(lsm->i2c_trans.buf, 0);
      lsm->data.vect.y = Int16FromBuf(lsm->i2c_trans.buf, 2);
      lsm->data.vect.z = Int16FromBuf(lsm->i2c_trans.buf, 4);
      lsm->data_available = true;
      lsm->i2c_trans.status = I2CTransDone;
    } else {
    }
  } else {
    if (lsm->i2c_trans.slave_addr == LSM303DLHC_ACC_ADDR) {
      if (lsm->init_status.acc != LSM_CONF_ACC_UNINIT) { // Configuring but not yet initialized
        if (lsm->i2c_trans.status == I2CTransSuccess || lsm->i2c_trans.status == I2CTransDone) {
          lsm->i2c_trans.status = I2CTransDone;
          lsm303dlhc_send_config(lsm);
        }
        if (lsm->i2c_trans.status == I2CTransFailed) {
          lsm->init_status.acc--;
          lsm->i2c_trans.status = I2CTransDone;
          lsm303dlhc_send_config(lsm); // Retry config (TODO max retry)
        }
      }
    } else {
      if (lsm->init_status.mag != LSM_CONF_MAG_UNINIT) { // Configuring but not yet initialized
        if (lsm->i2c_trans.status == I2CTransSuccess || lsm->i2c_trans.status == I2CTransDone) {
          lsm->i2c_trans.status = I2CTransDone;
          lsm303dlhc_send_config(lsm);
        }
        if (lsm->i2c_trans.status == I2CTransFailed) {
          lsm->init_status.mag--;
          lsm->i2c_trans.status = I2CTransDone;
          lsm303dlhc_send_config(lsm); // Retry config (TODO max retry)
        }
      }
    }
  }
}
