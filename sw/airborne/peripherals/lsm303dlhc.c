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
 * @file peripherals/lsm303dlhc.c
 *
 * Driver for Honeywell HMC5843 and HMC5883 magnetometers.
 * @todo DRDY/IRQ handling
 */

#include "peripherals/lsm303dlhc.h"
#include "std.h"


/* LSM303DLHC default conf */
#ifndef LSM303DLHC_DEFAULT_ODR
#define LSM303DLHC_DEFAULT_ODR 0x90 //normal 1.344khz, low power 5.376khz
#endif

#ifndef LSM303DLHC_DEFAULT_LP
#define LSM303DLHC_DEFAULT_LP 0x00 //low power disabled
#endif

#ifndef LSM303DLHC_DEFAULT_FS
#define LSM303DLHC_DEFAULT_FS 0x00 // +- 2G
#endif

#ifndef LSM303DLHC_DEFAULT_HR
#define LSM303DLHC_DEFAULT_HR 0x04 // high res enabled
#endif

/* #ifndef LSM303DLHC_DEFAULT_DO */
/* #define LSM303DLHC_DEFAULT_DO 0x6 // Data Output Rate (6 -> 50Hz with HMC5843, 75Hz with HMC5883) */
/* #endif */
/* #ifndef LSM303DLHC_DEFAULT_MS */
/* #define LSM303DLHC_DEFAULT_MS 0x0 // Measurement configuration */
/* #endif */
/* #ifndef LSM303DLHC_DEFAULT_GN */
/* #define LSM303DLHC_DEFAULT_GN 0x1 // Gain configuration (1 -> +- 1 Gauss) */
/* #endif */
/* #ifndef LSM303DLHC_DEFAULT_MD */
/* #define LSM303DLHC_DEFAULT_MD 0x0 // Continious measurement mode */
/* #endif */

static void lsm303dlhc_set_default_config(struct Lsm303dlhcConfig *c)
{
  c->rate = LSM303DLHC_DEFAULT_ODR;
  c->lp_mode = LSM303DLHC_DEFAULT_LP;
  c->scale = LSM303DLHC_DEFAULT_FS;
  c->hres = LSM303DLHC_DEFAULT_HR;

  //  c->meas = LSM303DLHC_DEFAULT_MS;
  //c->gain = LSM303DLHC_DEFAULT_GN;
  //c->mode = LSM303DLHC_DEFAULT_MD;
}

/**
 * Initialize Lsm303dlhc struct and set default config options.
 * @param lsm   Lsm303dlhc struct
 * @param i2c_p I2C periperal to use
 * @param addr  I2C address of Lsm303dlhc
 */
void lsm303dlhc_acc_init(struct Lsm303dlhc *lsm, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  lsm->i2c_p = i2c_p;
  /* set i2c address */
  lsm->i2c_trans.slave_addr = addr;
  lsm->i2c_trans.status = I2CTransDone;
  /* set default config options */
  lsm303dlhc_set_default_config(&(lsm->config));
  lsm->initialized = FALSE;
  lsm->init_status = LSM_CONF_UNINIT;
}

static void lsm303dlhc_i2c_tx_reg(struct Lsm303dlhc *lsm, uint8_t reg, uint8_t val)
{
  lsm->i2c_trans.type = I2CTransTx;
  lsm->i2c_trans.buf[0] = reg;
  lsm->i2c_trans.buf[1] = val;
  lsm->i2c_trans.len_r = 0;
  lsm->i2c_trans.len_w = 2;
  i2c_submit(lsm->i2c_p, &(lsm->i2c_trans));
}

/// Configuration function called once before normal use
static void lsm303dlhc_send_config(struct Lsm303dlhc *lsm)
{
  switch (lsm->init_status) {
  case LSM_CONF_CTRL_REG1_A:
    lsm303dlhc_i2c_tx_reg(lsm, LSM303DLHC_REG_CTRL_REG1_A, (lsm->config.rate & LSM303DLHC_ODR_MASK) | (lsm->config.lp_mode & LSM303DLHC_LPen) | LSM303DLHC_Xen | LSM303DLHC_Yen | LSM303DLHC_Zen);
      lsm->init_status++;
      break;
  case LSM_CONF_CTRL_REG4_A:
    lsm303dlhc_i2c_tx_reg(lsm, LSM303DLHC_REG_CTRL_REG4_A, (lsm->config.scale & LSM303DLHC_FS_MASK) | (lsm->config.hres & LSM303DLHC_HR));
      lsm->init_status++;
      break;
  case LSM_CONF_CTRL_REG3_A:
    lsm303dlhc_i2c_tx_reg(lsm, LSM303DLHC_REG_CTRL_REG3_A, LSM303DLHC_I1_DRDY1);
      lsm->init_status++;
      break;

    /* case LSM_CONF_CRA: */
    /*   lsm303dlhc_i2c_tx_reg(lsm, LSM303DLHC_REG_CFGA, (lsm->config.rate<<2)|(lsm->config.meas)); */
    /*   lsm->init_status++; */
    /*   break; */
    /* case LSM_CONF_CRB: */
    /*   lsm303dlhc_i2c_tx_reg(lsm, LSM303DLHC_REG_CFGB, (lsm->config.gain << 5)); */
    /*   lsm->init_status++; */
    /*   break; */
    /* case LSM_CONF_MODE: */
    /*   lsm303dlhc_i2c_tx_reg(lsm, LSM303DLHC_REG_MODE, lsm->config.mode); */
    /*   lsm->init_status++; */
    /*   break; */
    case LSM_CONF_DONE:
      lsm->initialized = TRUE;
      lsm->i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Configure
void lsm303dlhc_start_configure(struct Lsm303dlhc *lsm)
{
  if (lsm->init_status == LSM_CONF_UNINIT) {
    lsm->init_status++;
    if (lsm->i2c_trans.status == I2CTransSuccess || lsm->i2c_trans.status == I2CTransDone) {
      lsm303dlhc_send_config(lsm);
    }
  }
}

// Normal reading
void lsm303dlhc_read(struct Lsm303dlhc *lsm)
{
  if (lsm->initialized && lsm->i2c_trans.status == I2CTransDone){
    lsm->i2c_trans.buf[0] = LSM303DLHC_REG_OUT_X_L_A | 0x80;
    lsm->i2c_trans.type = I2CTransTxRx;
    lsm->i2c_trans.len_r = 6;
    lsm->i2c_trans.len_w = 1;
    i2c_submit(lsm->i2c_p, &(lsm->i2c_trans));
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void lsm303dlhc_event(struct Lsm303dlhc *lsm)
{
  if (lsm->initialized) {
    if (lsm->i2c_trans.status == I2CTransFailed) {
      lsm->i2c_trans.status = I2CTransDone;
    }
    else if (lsm->i2c_trans.status == I2CTransSuccess) {
      lsm->data.vect.x = Int16FromBuf(lsm->i2c_trans.buf,0);
      lsm->data.vect.y = Int16FromBuf(lsm->i2c_trans.buf,2);
      lsm->data.vect.z = Int16FromBuf(lsm->i2c_trans.buf,4);
      lsm->data_available = TRUE;
      lsm->i2c_trans.status = I2CTransDone;
    }
  }
  else if (lsm->init_status != LSM_CONF_UNINIT) { // Configuring but not yet initialized
    if (lsm->i2c_trans.status == I2CTransSuccess || lsm->i2c_trans.status == I2CTransDone) {
      lsm->i2c_trans.status = I2CTransDone;
      lsm303dlhc_send_config(lsm);
    }
    if (lsm->i2c_trans.status == I2CTransFailed) {
      lsm->init_status--;
      lsm->i2c_trans.status = I2CTransDone;
      lsm303dlhc_send_config(lsm); // Retry config (TODO max retry)
    }
  }
}

