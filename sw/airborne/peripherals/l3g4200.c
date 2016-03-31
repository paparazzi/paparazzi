/*
 * Copyright (C) 2011 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
 *               2013 Eduardo Lavratti <agressiva@hotmail.com>
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
 * @file peripherals/l3g4200.c
 *
 * Driver for L3G4200 from ST.
 */

#include "peripherals/l3g4200.h"
#include "std.h"

void l3g4200_set_default_config(struct L3g4200Config *c)
{
  c->ctrl_reg1 = L3G4200_DEFAULT_CTRL_REG1;
  c->ctrl_reg4 = L3G4200_DEFAULT_CTRL_REG4;
  c->ctrl_reg5 = L3G4200_DEFAULT_CTRL_REG5;
}

/**
 * Initialize L3g4200 struct and set default config options.
 * @param l3g   L3g4200 struct
 * @param i2c_p I2C periperal to use
 * @param addr  I2C address of L3G4200
 */

void l3g4200_init(struct L3g4200 *l3g, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  l3g->i2c_p = i2c_p;
  /* set i2c address */
  l3g->i2c_trans.slave_addr = addr;
  l3g->i2c_trans.status = I2CTransDone;
  /* set default config options */
  l3g4200_set_default_config(&(l3g->config));
  l3g->initialized = false;
  l3g->init_status = L3G_CONF_UNINIT;
}

static void l3g4200_i2c_tx_reg(struct L3g4200 *l3g, uint8_t reg, uint8_t val)
{
  l3g->i2c_trans.buf[0] = reg;
  l3g->i2c_trans.buf[1] = val;
  i2c_transmit(l3g->i2c_p, &(l3g->i2c_trans), l3g->i2c_trans.slave_addr, 2);
}

// Configuration function called once before normal use
static void l3g4200_send_config(struct L3g4200 *l3g)
{
  switch (l3g->init_status) {
    case L3G_CONF_REG1:
      l3g4200_i2c_tx_reg(l3g, L3G4200_REG_CTRL_REG1, l3g->config.ctrl_reg1);
      l3g->init_status++;
      break;
    case L3G_CONF_REG4:
      l3g4200_i2c_tx_reg(l3g, L3G4200_REG_CTRL_REG4, l3g->config.ctrl_reg4);
      l3g->init_status++;
      break;
    case L3G_CONF_REG5:
      l3g4200_i2c_tx_reg(l3g, L3G4200_REG_CTRL_REG5, l3g->config.ctrl_reg5);
      l3g->init_status++;
      break;
    case L3G_CONF_DONE:
      l3g->initialized = true;
      l3g->i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Configure
void l3g4200_start_configure(struct L3g4200 *l3g)
{
  if (l3g->init_status == L3G_CONF_UNINIT) {
    l3g->init_status++;
    if (l3g->i2c_trans.status == I2CTransSuccess || l3g->i2c_trans.status == I2CTransDone) {
      l3g4200_send_config(l3g);
    }
  }
}

// Normal reading
void l3g4200_read(struct L3g4200 *l3g)
{
  if (l3g->initialized && l3g->i2c_trans.status == I2CTransDone) {
    l3g->i2c_trans.buf[0] = 0x80 | L3G4200_REG_STATUS_REG;
    i2c_transceive(l3g->i2c_p, &(l3g->i2c_trans), l3g->i2c_trans.slave_addr, 1, 7);
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void l3g4200_event(struct L3g4200 *l3g)
{
  if (l3g->initialized) {
    if (l3g->i2c_trans.status == I2CTransFailed) {
      l3g->i2c_trans.status = I2CTransDone;
    } else if (l3g->i2c_trans.status == I2CTransSuccess) {
      // Successfull reading and new data available
      if (l3g->i2c_trans.buf[0] & 0x08) {
        // New data available
        l3g->data.rates.p = Int16FromBuf(l3g->i2c_trans.buf, 1);
        l3g->data.rates.q = Int16FromBuf(l3g->i2c_trans.buf, 3);
        l3g->data.rates.r = Int16FromBuf(l3g->i2c_trans.buf, 5);
        l3g->data_available = true;
      }
      l3g->i2c_trans.status = I2CTransDone;
    }
  } else if (l3g->init_status != L3G_CONF_UNINIT) { // Configuring but not yet initialized
    if (l3g->i2c_trans.status == I2CTransSuccess || l3g->i2c_trans.status == I2CTransDone) {
      l3g->i2c_trans.status = I2CTransDone;
      l3g4200_send_config(l3g);
    }
    if (l3g->i2c_trans.status == I2CTransFailed) {
      l3g->init_status--;
      l3g->i2c_trans.status = I2CTransDone;
      l3g4200_send_config(l3g); // Retry config (TODO max retry)
    }
  }
}
