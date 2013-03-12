/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 * @file peripherals/mpu60x0.h
 *
 * MPU-60X0 driver common interface (I2C and SPI).
 */

#ifndef MPU60X0_H
#define MPU60X0_H

#include "std.h"

/* Include address and register definition */
#include "peripherals/mpu60x0_regs.h"

/// Default sample rate divider
#define MPU60X0_DEFAULT_SMPLRT_DIV 0
/// Default gyro full scale range +- 2000°/s
#define MPU60X0_DEFAULT_FS_SEL MPU60X0_GYRO_RANGE_2000
/// Default accel full scale range +- 16g
#define MPU60X0_DEFAULT_AFS_SEL MPU60X0_ACCEL_RANGE_16G
/// Default internal sampling (1kHz, 42Hz LP Bandwidth)
#define MPU60X0_DEFAULT_DLPF_CFG MPU60X0_DLPF_42HZ
/// Default interrupt config: RAW_RDY_EN
#define MPU60X0_DEFAULT_INT_CFG 1
/// Default clock: PLL with X gyro reference
#define MPU60X0_DEFAULT_CLK_SEL 1

enum Mpu60x0ConfStatus {
  MPU60X0_CONF_UNINIT,
  MPU60X0_CONF_SD,
  MPU60X0_CONF_CONFIG,
  MPU60X0_CONF_GYRO,
  MPU60X0_CONF_ACCEL,
  MPU60X0_CONF_INT_PIN,
  MPU60X0_CONF_INT_ENABLE,
  MPU60X0_CONF_PWR,
  MPU60X0_CONF_DONE
};

struct Mpu60x0Config {
  uint8_t smplrt_div;                   ///< Sample rate divider
  enum Mpu60x0DLPF dlpf_cfg;            ///< Digital Low Pass Filter
  enum Adxl345GyroRanges gyro_range;    ///< deg/s Range
  enum Adxl345AccelRanges accel_range;  ///< g Range
  bool_t i2c_bypass;                    ///< bypass mpu i2c
  bool_t drdy_int_enable;               ///< Enable Data Ready Interrupt
  uint8_t clk_sel;                      ///< Clock select
  enum Mpu60x0ConfStatus init_status;   ///< init status
  bool_t initialized;                   ///< config done flag
};

static inline void mpu60X0_set_default_config(struct Mpu60x0Config *c)
{
  c->smplrt_div = MPU60X0_DEFAULT_SMPLRT_DIV;
  c->dlpf_cfg = MPU60X0_DEFAULT_DLPF_CFG;
  c->gyro_range = MPU60X0_DEFAULT_FS_SEL;
  c->accel_range = MPU60X0_DEFAULT_AFS_SEL;
  c->i2c_bypass = TRUE;
  c->drdy_int_enable = FALSE;
  c->clk_sel = MPU60X0_DEFAULT_CLK_SEL;
}

/// Configuration function prototype
typedef void (*Mpu60x0ConfigSet)(void* mpu, uint8_t _reg, uint8_t _val);

/// Configuration sequence called once before normal use
static inline void mpu60x0_send_config(Mpu60x0ConfigSet mpu_set, void* mpu, struct Mpu60x0Config* config)
{
  switch (config->init_status) {
    case MPU60X0_CONF_SD:
      mpu_set(mpu, MPU60X0_REG_SMPLRT_DIV, config->smplrt_div);
      config->init_status++;
      break;
    case MPU60X0_CONF_CONFIG:
      mpu_set(mpu, MPU60X0_REG_CONFIG, config->dlpf_cfg);
      config->init_status++;
      break;
    case MPU60X0_CONF_GYRO:
      mpu_set(mpu, MPU60X0_REG_GYRO_CONFIG, (config->gyro_range<<3));
      config->init_status++;
      break;
    case MPU60X0_CONF_ACCEL:
      mpu_set(mpu, MPU60X0_REG_ACCEL_CONFIG, (config->accel_range<<3));
      config->init_status++;
      break;
    case MPU60X0_CONF_INT_PIN:
      mpu_set(mpu, MPU60X0_REG_INT_PIN_CFG, (config->i2c_bypass<<1));
      config->init_status++;
      break;
    case MPU60X0_CONF_INT_ENABLE:
      mpu_set(mpu, MPU60X0_REG_INT_ENABLE, (config->drdy_int_enable<<0));
      config->init_status++;
      break;
    case MPU60X0_CONF_PWR:
      mpu_set(mpu, MPU60X0_REG_PWR_MGMT_1, ((config->config.clk_sel)|(0<<6));
      config->init_status++;
      break;
    case MPU60X0_CONF_DONE:
      config->initialized = TRUE;
      break;
    default:
      break;
  }
}


#endif // MPU60X0_H
