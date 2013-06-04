/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file peripherals/mpu60x0.c
 *
 * MPU-60X0 driver common functions (I2C and SPI).
 *
 * Still needs the either the I2C or SPI specific implementation.
 */

#include "peripherals/mpu60x0.h"

void mpu60x0_set_default_config(struct Mpu60x0Config *c)
{
  c->smplrt_div = MPU60X0_DEFAULT_SMPLRT_DIV;
  c->dlpf_cfg = MPU60X0_DEFAULT_DLPF_CFG;
  c->gyro_range = MPU60X0_DEFAULT_FS_SEL;
  c->accel_range = MPU60X0_DEFAULT_AFS_SEL;
  c->i2c_bypass = TRUE;
  c->drdy_int_enable = FALSE;
  c->clk_sel = MPU60X0_DEFAULT_CLK_SEL;
}

void mpu60x0_send_config(Mpu60x0ConfigSet mpu_set, void* mpu, struct Mpu60x0Config* config)
{
  switch (config->init_status) {
    case MPU60X0_CONF_PWR:
      /* switch to gyroX clock by default */
      mpu_set(mpu, MPU60X0_REG_PWR_MGMT_1, ((config->clk_sel)|(0<<6)));
      config->init_status++;
      break;
    case MPU60X0_CONF_SD:
      /* configure sample rate divider */
      mpu_set(mpu, MPU60X0_REG_SMPLRT_DIV, config->smplrt_div);
      config->init_status++;
      break;
    case MPU60X0_CONF_DLPF:
      /* configure digital low pass filter */
      mpu_set(mpu, MPU60X0_REG_CONFIG, config->dlpf_cfg);
      config->init_status++;
      break;
    case MPU60X0_CONF_GYRO:
      /* configure gyro range */
      mpu_set(mpu, MPU60X0_REG_GYRO_CONFIG, (config->gyro_range<<3));
      config->init_status++;
      break;
    case MPU60X0_CONF_ACCEL:
      /* configure accelerometer range */
      mpu_set(mpu, MPU60X0_REG_ACCEL_CONFIG, (config->accel_range<<3));
      config->init_status++;
      break;
    case MPU60X0_CONF_I2C_BYPASS:
      mpu_set(mpu, MPU60X0_REG_INT_PIN_CFG, (config->i2c_bypass<<1));
      config->init_status++;
      break;
    case MPU60X0_CONF_INT_ENABLE:
      /* configure data ready interrupt */
      mpu_set(mpu, MPU60X0_REG_INT_ENABLE, (config->drdy_int_enable<<0));
      config->init_status++;
      break;
    case MPU60X0_CONF_DONE:
      config->initialized = TRUE;
      break;
    default:
      break;
  }
}
