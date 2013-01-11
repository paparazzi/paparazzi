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
#include "std.h"

#include "peripherals/hmc58xx_regs.h"

/* HMC58XX default conf */
#ifndef HMC58XX_DO
#define HMC58XX_DO 0x6 // Data Output Rate (6 -> 50Hz with HMC5843, 75Hz with HMC5883)
#endif
#ifndef HMC58XX_MS
#define HMC58XX_MS 0x0 // Measurement configuration
#endif
#ifndef HMC58XX_GN
#define HMC58XX_GN 0x1 // Gain configuration (1 -> +- 1 Gauss)
#endif
#ifndef HMC58XX_MD
#define HMC58XX_MD 0x0 // Continious measurement mode
#endif

/* Default I2C device is i2c2 (for lisa) */
#ifndef HMC58XX_I2C_DEV
#define HMC58XX_I2C_DEV i2c2
#endif


void hmc58xx_init(struct Hmc58xx *hmc, bool_t set_default_config)
{
  hmc->i2c_trans.status = I2CTransDone;
  hmc->i2c_trans.slave_addr = HMC58XX_ADDR;
  if (set_default_config) {
    hmc->type = HMC_TYPE_5883;
    hmc->i2c_p = &(HMC58XX_I2C_DEV);
    hmc->config.dor = HMC58XX_DO;
    hmc->config.ms = HMC58XX_MS;
    hmc->config.gn = HMC58XX_GN;
    hmc->config.md = HMC58XX_MD;
  }
  hmc->initialized = FALSE;
  hmc->init_status = HMC_CONF_UNINIT;
}

/// Configuration function called once before normal use
static void hmc58xx_send_config(struct Hmc58xx *hmc)
{
  switch (hmc->init_status) {
    case HMC_CONF_CRA:
      hmc->i2c_trans.buf[0] = HMC58XX_REG_CFGA;
      hmc->i2c_trans.buf[1] = (hmc->config.dor<<2)|(hmc->config.ms);
      I2CTransmit(*(hmc->i2c_p), hmc->i2c_trans, HMC58XX_ADDR, 2);
      hmc->init_status++;
      break;
    case HMC_CONF_CRB:
      hmc->i2c_trans.buf[0] = HMC58XX_REG_CFGB;
      hmc->i2c_trans.buf[1] = hmc->config.gn<<5;
      I2CTransmit(*(hmc->i2c_p), hmc->i2c_trans, HMC58XX_ADDR, 2);
      hmc->init_status++;
      break;
    case HMC_CONF_MODE:
      hmc->i2c_trans.buf[0] = HMC58XX_REG_MODE;
      hmc->i2c_trans.buf[1] = hmc->config.md;
      I2CTransmit(*(hmc->i2c_p), hmc->i2c_trans, HMC58XX_ADDR, 2);
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
void hmc58xx_configure(struct Hmc58xx *hmc)
{
  if (hmc->init_status == HMC_CONF_UNINIT) {
    hmc->init_status++;
    if (hmc->i2c_trans.status == I2CTransSuccess || hmc->i2c_trans.status == I2CTransDone) {
      hmc58xx_send_config(hmc);
    }
  }
}

// Normal reading
void hmc58xx_read(struct Hmc58xx *hmc)
{
  if (hmc->initialized && hmc->i2c_trans.status == I2CTransDone){
    hmc->i2c_trans.buf[0] = HMC58XX_REG_DATXM;
    I2CTransceive(*(hmc->i2c_p), hmc->i2c_trans, HMC58XX_ADDR, 1, 6);
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void hmc58xx_event(struct Hmc58xx *hmc)
{
  if (hmc->initialized) {
    if (hmc->i2c_trans.status == I2CTransFailed) {
      hmc->i2c_trans.status = I2CTransDone;
    }
    else if (hmc->i2c_trans.status == I2CTransSuccess) {
      if (hmc->type == HMC_TYPE_5843) {
        hmc->data.vect.x = Int16FromBuf(hmc->i2c_trans.buf,0);
        hmc->data.vect.y = Int16FromBuf(hmc->i2c_trans.buf,2);
        hmc->data.vect.z = Int16FromBuf(hmc->i2c_trans.buf,4);
      }
      else {
        hmc->data.vect.x = Int16FromBuf(hmc->i2c_trans.buf,0);
        hmc->data.vect.y = Int16FromBuf(hmc->i2c_trans.buf,4);
        hmc->data.vect.z = Int16FromBuf(hmc->i2c_trans.buf,2);
      }
      hmc->data_available = TRUE;
      hmc->i2c_trans.status = I2CTransDone;
    }
  }
  else if (!hmc->initialized && hmc->init_status != HMC_CONF_UNINIT) { // Configuring
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

