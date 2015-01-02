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
 * @file peripherals/hmc58xx.c
 *
 * Driver for Honeywell HMC5843 and HMC5883 magnetometers.
 * @todo DRDY/IRQ handling
 */

#include "peripherals/hmc58xx.h"
#include "mcu_periph/sys_time.h"
#include "std.h"


/* HMC58XX default conf */
#ifndef HMC58XX_DEFAULT_DO
#define HMC58XX_DEFAULT_DO 0x6 // Data Output Rate (6 -> 50Hz with HMC5843, 75Hz with HMC5883)
#endif
#ifndef HMC58XX_DEFAULT_MS
#define HMC58XX_DEFAULT_MS 0x0 // Measurement configuration
#endif
#ifndef HMC58XX_DEFAULT_GN
#define HMC58XX_DEFAULT_GN 0x1 // Gain configuration (1 -> +- 1 Gauss)
#endif
#ifndef HMC58XX_DEFAULT_MD
#define HMC58XX_DEFAULT_MD 0x0 // Continious measurement mode
#endif

/** HMC58XX startup delay
 *
 *  On startup, the hmc is making a first conversion in single mode.
 *  Trying to configure the mode register before the end of this conversion
 *  seems to void the configuration.
 *  Default conversion rate is 15 Hz (66ms) and worst case is O.75Hz (1.3s).
 *  Let set the default delay to 1.5s afer boot time.
 */
#ifndef HMC58XX_STARTUP_DELAY
#define HMC58XX_STARTUP_DELAY 1.5
#endif

static void hmc58xx_set_default_config(struct Hmc58xxConfig *c)
{
  c->rate = HMC58XX_DEFAULT_DO;
  c->meas = HMC58XX_DEFAULT_MS;
  c->gain = HMC58XX_DEFAULT_GN;
  c->mode = HMC58XX_DEFAULT_MD;
}

/**
 * Initialize Hmc58xx struct and set default config options.
 * @param hmc   Hmc58xx struct
 * @param i2c_p I2C periperal to use
 * @param addr  I2C address of HMC58xx
 */
void hmc58xx_init(struct Hmc58xx *hmc, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  hmc->i2c_p = i2c_p;
  /* set i2c address */
  hmc->i2c_trans.slave_addr = addr;
  hmc->i2c_trans.status = I2CTransDone;
  /* set default config options */
  hmc58xx_set_default_config(&(hmc->config));
  hmc->type = HMC_TYPE_5883;
  hmc->initialized = FALSE;
  hmc->init_status = HMC_CONF_UNINIT;
}

static void hmc58xx_i2c_tx_reg(struct Hmc58xx *hmc, uint8_t reg, uint8_t val)
{
  hmc->i2c_trans.type = I2CTransTx;
  hmc->i2c_trans.buf[0] = reg;
  hmc->i2c_trans.buf[1] = val;
  hmc->i2c_trans.len_r = 0;
  hmc->i2c_trans.len_w = 2;
  i2c_submit(hmc->i2c_p, &(hmc->i2c_trans));
}

/// Configuration function called once before normal use
static void hmc58xx_send_config(struct Hmc58xx *hmc)
{
  switch (hmc->init_status) {
    case HMC_CONF_CRA:
      hmc58xx_i2c_tx_reg(hmc, HMC58XX_REG_CFGA, (hmc->config.rate << 2) | (hmc->config.meas));
      hmc->init_status++;
      break;
    case HMC_CONF_CRB:
      hmc58xx_i2c_tx_reg(hmc, HMC58XX_REG_CFGB, (hmc->config.gain << 5));
      hmc->init_status++;
      break;
    case HMC_CONF_MODE:
      hmc58xx_i2c_tx_reg(hmc, HMC58XX_REG_MODE, hmc->config.mode);
      hmc->init_status++;
      break;
    case HMC_CONF_DONE:
      hmc->initialized = TRUE;
      hmc->i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Configure
void hmc58xx_start_configure(struct Hmc58xx *hmc)
{
  // wait before starting the configuration
  // doing to early may void the mode configuration
  if (hmc->init_status == HMC_CONF_UNINIT && get_sys_time_float() > HMC58XX_STARTUP_DELAY) {
    hmc->init_status++;
    if (hmc->i2c_trans.status == I2CTransSuccess || hmc->i2c_trans.status == I2CTransDone) {
      hmc58xx_send_config(hmc);
    }
  }
}

// Normal reading
void hmc58xx_read(struct Hmc58xx *hmc)
{
  if (hmc->initialized && hmc->i2c_trans.status == I2CTransDone) {
    hmc->i2c_trans.buf[0] = HMC58XX_REG_DATXM;
    hmc->i2c_trans.type = I2CTransTxRx;
    hmc->i2c_trans.len_r = 6;
    hmc->i2c_trans.len_w = 1;
    i2c_submit(hmc->i2c_p, &(hmc->i2c_trans));
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void hmc58xx_event(struct Hmc58xx *hmc)
{
  if (hmc->initialized) {
    if (hmc->i2c_trans.status == I2CTransFailed) {
      hmc->i2c_trans.status = I2CTransDone;
    } else if (hmc->i2c_trans.status == I2CTransSuccess) {
      if (hmc->type == HMC_TYPE_5843) {
        hmc->data.vect.x = Int16FromBuf(hmc->i2c_trans.buf, 0);
        hmc->data.vect.y = Int16FromBuf(hmc->i2c_trans.buf, 2);
        hmc->data.vect.z = Int16FromBuf(hmc->i2c_trans.buf, 4);
      }
      /* HMC5883 has xzy order of axes in returned data */
      else {
        hmc->data.vect.x = Int16FromBuf(hmc->i2c_trans.buf, 0);
        hmc->data.vect.y = Int16FromBuf(hmc->i2c_trans.buf, 4);
        hmc->data.vect.z = Int16FromBuf(hmc->i2c_trans.buf, 2);
      }
      hmc->data_available = TRUE;
      hmc->i2c_trans.status = I2CTransDone;
    }
  } else if (hmc->init_status != HMC_CONF_UNINIT) { // Configuring but not yet initialized
    if (hmc->i2c_trans.status == I2CTransSuccess || hmc->i2c_trans.status == I2CTransDone) {
      hmc->i2c_trans.status = I2CTransDone;
      hmc58xx_send_config(hmc);
    }
    if (hmc->i2c_trans.status == I2CTransFailed) {
      hmc->init_status--;
      hmc->i2c_trans.status = I2CTransDone;
      hmc58xx_send_config(hmc); // Retry config (TODO max retry)
    }
  }
}

