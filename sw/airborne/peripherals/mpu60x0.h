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
/// Default interrupt config: DATA_RDY_EN
#define MPU60X0_DEFAULT_INT_CFG 1
/// Default clock: PLL with X gyro reference
#define MPU60X0_DEFAULT_CLK_SEL 1

// Default number of I2C slaves
#ifndef MPU60X0_I2C_NB_SLAVES
#define MPU60X0_I2C_NB_SLAVES 5
#endif

enum Mpu60x0ConfStatus {
  MPU60X0_CONF_UNINIT,
  MPU60X0_CONF_RESET,
  MPU60X0_CONF_USER_RESET,
  MPU60X0_CONF_PWR,
  MPU60X0_CONF_SD,
  MPU60X0_CONF_DLPF,
  MPU60X0_CONF_GYRO,
  MPU60X0_CONF_ACCEL,
  MPU60X0_CONF_I2C_SLAVES,
  MPU60X0_CONF_INT_ENABLE,
  MPU60X0_CONF_DONE
};

/// Configuration function prototype
typedef void (*Mpu60x0ConfigSet)(void *mpu, uint8_t _reg, uint8_t _val);

/// function prototype for configuration of a single I2C slave
typedef bool (*Mpu60x0I2cSlaveConfigure)(Mpu60x0ConfigSet mpu_set, void *mpu);

struct Mpu60x0I2cSlave {
  Mpu60x0I2cSlaveConfigure configure;
};

struct Mpu60x0Config {
  uint8_t smplrt_div;                   ///< Sample rate divider
  enum Mpu60x0DLPF dlpf_cfg;            ///< Digital Low Pass Filter
  enum Mpu60x0GyroRanges gyro_range;    ///< deg/s Range
  enum Mpu60x0AccelRanges accel_range;  ///< g Range
  bool drdy_int_enable;               ///< Enable Data Ready Interrupt
  uint8_t clk_sel;                      ///< Clock select
  uint8_t nb_bytes;                     ///< number of bytes to read starting with MPU60X0_REG_INT_STATUS
  enum Mpu60x0ConfStatus init_status;   ///< init status
  bool initialized;                   ///< config done flag

  /** Bypass MPU I2C.
   * Only effective if using the I2C implementation.
   */
  bool i2c_bypass;

  uint8_t nb_slaves;                    ///< number of used I2C slaves
  uint8_t nb_slave_init;                ///< number of already configured/initialized slaves
  struct Mpu60x0I2cSlave slaves[MPU60X0_I2C_NB_SLAVES];     ///< I2C slaves
  enum Mpu60x0MstClk i2c_mst_clk;       ///< MPU I2C master clock speed
  uint8_t i2c_mst_delay;                ///< MPU I2C slaves delayed sample rate
};

extern void mpu60x0_set_default_config(struct Mpu60x0Config *c);

/// Configuration sequence called once before normal use
extern void mpu60x0_send_config(Mpu60x0ConfigSet mpu_set, void *mpu, struct Mpu60x0Config *config);

/**
 * Configure I2C slaves of the MPU.
 * This is I2C/SPI implementation specific.
 * @param mpu_set configuration function
 * @param mpu Mpu60x0Spi or Mpu60x0I2c peripheral
 * @return TRUE when all slaves are configured
 */
extern bool mpu60x0_configure_i2c_slaves(Mpu60x0ConfigSet mpu_set, void *mpu);

#endif // MPU60X0_H
