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
 * @file peripherals/itg3200.c
 *
 * Driver for ITG3200.
 * @todo DRDY/IRQ handling
 */

#include "peripherals/itg3200.h"
#include "std.h"

void itg3200_set_default_config(struct Itg3200Config *c) {
  c->smplrt_div = ITG3200_DEFAULT_SMPLRT_DIV;
  c->fs_sel = ITG3200_DEFAULT_FS_SEL;
  c->dlpf_cfg = ITG3200_DEFAULT_DLPF_CFG;
  c->int_cfg = ITG3200_DEFAULT_INT_CFG;
  c->clk_sel = ITG3200_DEFAULT_CLK_SEL;
}


/**
 * Initialize Itg3200 struct and set default config options.
 * @param itg   Itg3200 struct
 * @param i2c_p I2C periperal to use
 * @param addr  I2C address of ITG3200
 */
void itg3200_init(struct Itg3200 *itg, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  itg->i2c_p = i2c_p;
  /* set i2c address */
  itg->i2c_trans.slave_addr = addr;
  itg->i2c_trans.status = I2CTransDone;
  /* set default config options */
  itg3200_set_default_config(&(itg->config));
  itg->initialized = FALSE;
  itg->init_status = ITG_CONF_UNINIT;
}

// Configuration function called once before normal use
static void itg3200_send_config(struct Itg3200 *itg)
{
  switch (itg->init_status) {
    case ITG_CONF_SD:
      itg->i2c_trans.buf[0] = ITG3200_REG_SMPLRT_DIV;
      itg->i2c_trans.buf[1] = itg->config.smplrt_div;
      I2CTransmit(*(itg->i2c_p), itg->i2c_trans, itg->i2c_trans.slave_addr, 2);
      itg->init_status++;
      break;
    case ITG_CONF_DF:
      itg->i2c_trans.buf[0] = ITG3200_REG_DLPF_FS;
      itg->i2c_trans.buf[1] = (itg->config.fs_sel<<3)|(itg->config.dlpf_cfg);
      I2CTransmit(*(itg->i2c_p), itg->i2c_trans, itg->i2c_trans.slave_addr, 2);
      itg->init_status++;
      break;
    case ITG_CONF_INT:
      itg->i2c_trans.buf[0] = ITG3200_REG_INT_CFG;
      itg->i2c_trans.buf[1] = itg->config.int_cfg;
      I2CTransmit(*(itg->i2c_p), itg->i2c_trans, itg->i2c_trans.slave_addr, 2);
      itg->init_status++;
      break;
    case ITG_CONF_PWR:
      itg->i2c_trans.buf[0] = ITG3200_REG_PWR_MGM;
      itg->i2c_trans.buf[1] = itg->config.clk_sel;
      I2CTransmit(*(itg->i2c_p), itg->i2c_trans, itg->i2c_trans.slave_addr, 2);
      itg->init_status++;
      break;
    case ITG_CONF_DONE:
      itg->initialized = TRUE;
      itg->i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Configure
void itg3200_configure(struct Itg3200 *itg)
{
  if (itg->init_status == ITG_CONF_UNINIT) {
    itg->init_status++;
    if (itg->i2c_trans.status == I2CTransSuccess || itg->i2c_trans.status == I2CTransDone) {
      itg3200_send_config(itg);
    }
  }
}

// Normal reading
void itg3200_read(struct Itg3200 *itg)
{
  if (itg->initialized && itg->i2c_trans.status == I2CTransDone) {
    itg->i2c_trans.buf[0] = ITG3200_REG_INT_STATUS;
    I2CTransceive(*(itg->i2c_p), itg->i2c_trans, itg->i2c_trans.slave_addr, 1, 9);
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void itg3200_event(struct Itg3200 *itg)
{
  if (itg->initialized) {
    if (itg->i2c_trans.status == I2CTransFailed) {
      itg->i2c_trans.status = I2CTransDone;
    }
    else if (itg->i2c_trans.status == I2CTransSuccess) {
      // Successfull reading and new data available
      if (itg->i2c_trans.buf[0] & 0x01) {
        // New data available
        itg->data.rates.p = Int16FromBuf(itg->i2c_trans.buf,3);
        itg->data.rates.q = Int16FromBuf(itg->i2c_trans.buf,5);
        itg->data.rates.r = Int16FromBuf(itg->i2c_trans.buf,7);
        itg->data_available = TRUE;
      }
      itg->i2c_trans.status = I2CTransDone;
    }
  }
  else if (!itg->initialized && itg->init_status != ITG_CONF_UNINIT) { // Configuring
    if (itg->i2c_trans.status == I2CTransSuccess || itg->i2c_trans.status == I2CTransDone) {
      itg->i2c_trans.status = I2CTransDone;
      itg3200_send_config(itg);
    }
    if (itg->i2c_trans.status == I2CTransFailed) {
      itg->init_status--;
      itg->i2c_trans.status = I2CTransDone;
      itg3200_send_config(itg); // Retry config (TODO max retry)
    }
  }
}

