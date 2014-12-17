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
 * @file peripherals/adxl345_i2c.c
 *
 * Driver for ADXL345 accelerometer using I2C.
 * @todo IRQ handling
 */

#include "peripherals/adxl345_i2c.h"
#include "std.h"

#define ADXL345_DATA_FORMAT ((adxl->config.int_invert<<5)|(adxl->config.full_res<<3)|(adxl->config.justify_msb<<2)|(adxl->config.range))


void adxl345_i2c_init(struct Adxl345_I2c *adxl, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  adxl->i2c_p = i2c_p;
  /* set i2c address */
  adxl->i2c_trans.slave_addr = addr;
  adxl->i2c_trans.status = I2CTransDone;
  /* set default config options */
  adxl345_set_default_config(&(adxl->config));
  adxl->initialized = FALSE;
  adxl->init_status = ADXL_CONF_UNINIT;
}

static void adxl345_i2c_tx_reg(struct Adxl345_I2c *adxl, uint8_t reg, uint8_t val)
{
  adxl->i2c_trans.type = I2CTransTx;
  adxl->i2c_trans.buf[0] = reg;
  adxl->i2c_trans.buf[1] = val;
  adxl->i2c_trans.len_r = 0;
  adxl->i2c_trans.len_w = 2;
  i2c_submit(adxl->i2c_p, &(adxl->i2c_trans));
}

// Configuration function called once before normal use
static void adxl345_i2c_send_config(struct Adxl345_I2c *adxl)
{
  switch (adxl->init_status) {
    case ADXL_CONF_RATE:
      adxl345_i2c_tx_reg(adxl, ADXL345_REG_BW_RATE, adxl->config.rate);
      adxl->init_status++;
      break;
    case ADXL_CONF_INT:
      adxl345_i2c_tx_reg(adxl, ADXL345_REG_INT_ENABLE, adxl->config.drdy_int_enable);
      adxl->init_status++;
      break;
    case ADXL_CONF_FORMAT:
      adxl345_i2c_tx_reg(adxl, ADXL345_REG_DATA_FORMAT, ADXL345_DATA_FORMAT);
      adxl->init_status++;
      break;
    case ADXL_CONF_ENABLE:
      /* enable measurement, is in standby after power up */
      adxl345_i2c_tx_reg(adxl, ADXL345_REG_POWER_CTL, (0x1 << 3));
      adxl->init_status++;
      break;
    case ADXL_CONF_DONE:
      adxl->initialized = TRUE;
      adxl->i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

/// Start configuration if not already done
void adxl345_i2c_start_configure(struct Adxl345_I2c *adxl)
{
  if (adxl->init_status == ADXL_CONF_UNINIT) {
    adxl->init_status++;
    if (adxl->i2c_trans.status == I2CTransSuccess || adxl->i2c_trans.status == I2CTransDone) {
      adxl345_i2c_send_config(adxl);
    }
  }
}

// Normal reading
void adxl345_i2c_read(struct Adxl345_I2c *adxl)
{
  if (adxl->initialized && adxl->i2c_trans.status == I2CTransDone) {
    adxl->i2c_trans.buf[0] = ADXL345_REG_DATA_X0;
    adxl->i2c_trans.type = I2CTransTxRx;
    adxl->i2c_trans.len_r = 6;
    adxl->i2c_trans.len_w = 1;
    i2c_submit(adxl->i2c_p, &(adxl->i2c_trans));
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void adxl345_i2c_event(struct Adxl345_I2c *adxl)
{
  if (adxl->initialized) {
    if (adxl->i2c_trans.status == I2CTransFailed) {
      adxl->i2c_trans.status = I2CTransDone;
    } else if (adxl->i2c_trans.status == I2CTransSuccess) {
      // Successfull reading
      adxl->data.vect.x = Int16FromBuf(adxl->i2c_trans.buf, 0);
      adxl->data.vect.y = Int16FromBuf(adxl->i2c_trans.buf, 2);
      adxl->data.vect.z = Int16FromBuf(adxl->i2c_trans.buf, 4);
      adxl->data_available = TRUE;
      adxl->i2c_trans.status = I2CTransDone;
    }
  } else if (adxl->init_status != ADXL_CONF_UNINIT) { // Configuring but not yet initialized
    if (adxl->i2c_trans.status == I2CTransSuccess || adxl->i2c_trans.status == I2CTransDone) {
      adxl->i2c_trans.status = I2CTransDone;
      adxl345_i2c_send_config(adxl);
    }
    if (adxl->i2c_trans.status == I2CTransFailed) {
      adxl->init_status--;
      adxl->i2c_trans.status = I2CTransDone;
      adxl345_i2c_send_config(adxl); // Retry config (TODO max retry)
    }
  }
}
