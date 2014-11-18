/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file peripherals/mpu9250.h
 *
 * MPU-60X0 driver common interface (I2C and SPI).
 */

#ifndef MPU9250_H
#define MPU9250_H

#include "std.h"

/* Include address and register definition */
#include "peripherals/mpu9250_regs.h"

/// Default sample rate divider
#define MPU9250_DEFAULT_SMPLRT_DIV 0
/// Default gyro full scale range +- 2000°/s
#define MPU9250_DEFAULT_FS_SEL MPU9250_GYRO_RANGE_1000
/// Default accel full scale range +- 16g
#define MPU9250_DEFAULT_AFS_SEL MPU9250_ACCEL_RANGE_8G
/// Default internal sampling (1kHz, 42Hz LP Bandwidth)
#define MPU9250_DEFAULT_DLPF_ACCEL_CFG MPU9250_DLPF_ACCEL_41HZ
/// Default internal sampling (1kHz, 42Hz LP Bandwidth)
#define MPU9250_DEFAULT_DLPF_GYRO_CFG MPU9250_DLPF_GYRO_41HZ
/// Default interrupt config: DATA_RDY_EN
#define MPU9250_DEFAULT_INT_CFG 1
/// Default clock: PLL with X gyro reference
#define MPU9250_DEFAULT_CLK_SEL 1

enum Mpu9250ConfStatus {
  MPU9250_CONF_UNINIT,
  MPU9250_CONF_RESET,
  MPU9250_CONF_USER_RESET,
  MPU9250_CONF_PWR,
  MPU9250_CONF_SD,
  MPU9250_CONF_DLPF_ACCEL,
  MPU9250_CONF_DLPF_GYRO,
  MPU9250_CONF_GYRO,
  MPU9250_CONF_ACCEL,
  MPU9250_CONF_I2C_SLAVES,
  MPU9250_CONF_INT_ENABLE,
  MPU9250_CONF_DONE
};

/// Configuration function prototype
typedef void (*Mpu9250ConfigSet)(void* mpu, uint8_t _reg, uint8_t _val);

/// function prototype for configuration of a single I2C slave
typedef bool_t (*Mpu9250I2cSlaveConfigure)(Mpu9250ConfigSet mpu_set, void* mpu);

struct Mpu9250I2cSlave {
  Mpu9250I2cSlaveConfigure configure;
};

struct Mpu9250Config {
  uint8_t smplrt_div;                   ///< Sample rate divider
  enum Mpu9250DLPFAccel dlpf_accel_cfg; ///< Digital Low Pass Filter for accelerometer
  enum Mpu9250DLPFGyro dlpf_gyro_cfg;   ///< Digital Low Pass Filter for gyroscope
  enum Mpu9250GyroRanges gyro_range;    ///< deg/s Range
  enum Mpu9250AccelRanges accel_range;  ///< g Range
  bool_t drdy_int_enable;               ///< Enable Data Ready Interrupt
  uint8_t clk_sel;                      ///< Clock select
  uint8_t nb_bytes;                     ///< number of bytes to read starting with MPU9250_REG_INT_STATUS
  enum Mpu9250ConfStatus init_status;   ///< init status
  bool_t initialized;                   ///< config done flag

  /** Bypass MPU I2C.
   * Only effective if using the I2C implementation.
   */
  bool_t i2c_bypass;

  uint8_t nb_slaves;                    ///< number of used I2C slaves
  struct Mpu9250I2cSlave slaves[5];     ///< I2C slaves
  enum Mpu9250MstClk i2c_mst_clk;       ///< MPU I2C master clock speed
  uint8_t i2c_mst_delay;                ///< MPU I2C slaves delayed sample rate
};

extern void mpu9250_set_default_config(struct Mpu9250Config *c);

/// Configuration sequence called once before normal use
extern void mpu9250_send_config(Mpu9250ConfigSet mpu_set, void* mpu, struct Mpu9250Config* config);

/**
 * Configure I2C slaves of the MPU.
 * This is I2C/SPI implementation specific.
 * @param mpu_set configuration function
 * @param mpu Mpu9250Spi or Mpu9250I2c peripheral
 * @return TRUE when all slaves are configured
 */
extern bool_t mpu9250_configure_i2c_slaves(Mpu9250ConfigSet mpu_set, void* mpu);

#endif // MPU9250_H
