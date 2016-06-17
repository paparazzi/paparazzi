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
 * @file peripherals/mpu9250.c
 *
 * MPU-9250 driver common functions (I2C and SPI).
 *
 * Still needs the either the I2C or SPI specific implementation.
 */

#include "peripherals/mpu9250.h"

void mpu9250_set_default_config(struct Mpu9250Config *c)
{
  c->clk_sel = MPU9250_DEFAULT_CLK_SEL;
  c->smplrt_div = MPU9250_DEFAULT_SMPLRT_DIV;
  c->dlpf_gyro_cfg = MPU9250_DEFAULT_DLPF_GYRO_CFG;
  c->dlpf_accel_cfg = MPU9250_DEFAULT_DLPF_ACCEL_CFG;
  c->gyro_range = MPU9250_DEFAULT_FS_SEL;
  c->accel_range = MPU9250_DEFAULT_AFS_SEL;
  c->drdy_int_enable = false;

  /* Number of bytes to read starting with MPU9250_REG_INT_STATUS
   * By default read only gyro and accel data -> 15 bytes.
   * Increase to include slave data.
   */
  c->nb_bytes = 15;
  c->nb_slaves = 0;
  c->nb_slave_init = 0;

  c->i2c_bypass = false;
}

void mpu9250_send_config(Mpu9250ConfigSet mpu_set, void *mpu, struct Mpu9250Config *config)
{
  switch (config->init_status) {
    case MPU9250_CONF_RESET:
      /* device reset, set register values to defaults */
      mpu_set(mpu, MPU9250_REG_PWR_MGMT_1, (1 << 6));
      config->init_status++;
      break;
    case MPU9250_CONF_USER_RESET:
      /* trigger FIFO, I2C_MST and SIG_COND resets */
      mpu_set(mpu, MPU9250_REG_USER_CTRL, ((1 << MPU9250_FIFO_RESET) |
                                           (1 << MPU9250_I2C_MST_RESET) |
                                           (1 << MPU9250_SIG_COND_RESET)));
      config->init_status++;
      break;
    case MPU9250_CONF_PWR:
      /* switch to gyroX clock by default */
      mpu_set(mpu, MPU9250_REG_PWR_MGMT_1, ((config->clk_sel) | (0 << 6)));
      config->init_status++;
      break;
    case MPU9250_CONF_SD:
      /* configure sample rate divider */
      mpu_set(mpu, MPU9250_REG_SMPLRT_DIV, config->smplrt_div);
      config->init_status++;
      break;
    case MPU9250_CONF_DLPF_GYRO:
      /* configure digital low pass filter for gyroscope */
      mpu_set(mpu, MPU9250_REG_CONFIG, config->dlpf_gyro_cfg);
      config->init_status++;
      break;
    case MPU9250_CONF_DLPF_ACCEL:
      /* configure digital low pass filter fir accelerometer */
      mpu_set(mpu, MPU9250_REG_ACCEL_CONFIG_2, config->dlpf_accel_cfg);
      config->init_status++;
      break;
    case MPU9250_CONF_GYRO:
      /* configure gyro range */
      mpu_set(mpu, MPU9250_REG_GYRO_CONFIG, (config->gyro_range << 3));
      config->init_status++;
      break;
    case MPU9250_CONF_ACCEL:
      /* configure accelerometer range */
      mpu_set(mpu, MPU9250_REG_ACCEL_CONFIG, (config->accel_range << 3));
      config->init_status++;
      break;
    case MPU9250_CONF_I2C_SLAVES:
      /* if any, set MPU for I2C slaves and configure them*/
      if (config->nb_slaves > 0) {
        /* returns TRUE when all slaves are configured */
        if (mpu9250_configure_i2c_slaves(mpu_set, mpu)) {
          config->init_status++;
        }
      } else {
        config->init_status++;
      }
      break;
    case MPU9250_CONF_INT_ENABLE:
      /* configure data ready interrupt */
      mpu_set(mpu, MPU9250_REG_INT_ENABLE, (config->drdy_int_enable << 0));
      config->init_status++;
      break;
    case MPU9250_CONF_DONE:
      config->initialized = true;
      break;
    default:
      break;
  }
}
