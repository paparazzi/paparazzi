/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file subsystems/imu/imu_swing.c
 * Driver for the Swing accelerometer and gyroscope
 */

#include "subsystems/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/i2c.h"


/* defaults suitable for Swing */
#ifndef SWING_MPU_I2C_DEV
#define SWING_MPU_I2C_DEV i2c0
#endif
PRINT_CONFIG_VAR(SWING_MPU_I2C_DEV)

#if !defined SWING_LOWPASS_FILTER && !defined  SWING_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define SWING_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define SWING_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define SWING_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define SWING_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#endif
#endif
PRINT_CONFIG_VAR(SWING_SMPLRT_DIV)
PRINT_CONFIG_VAR(SWING_LOWPASS_FILTER)

PRINT_CONFIG_VAR(SWING_GYRO_RANGE)
PRINT_CONFIG_VAR(SWING_ACCEL_RANGE)

/** Basic Navstik IMU data */
struct ImuSwing imu_swing;

/**
 * Navstik IMU initializtion of the MPU-60x0 and HMC58xx
 */
void imu_swing_init(void)
{
  /* MPU-60X0 */
  mpu60x0_i2c_init(&imu_swing.mpu, &(SWING_MPU_I2C_DEV), MPU60X0_ADDR);
  imu_swing.mpu.config.smplrt_div = SWING_SMPLRT_DIV;
  imu_swing.mpu.config.dlpf_cfg = SWING_LOWPASS_FILTER;
  imu_swing.mpu.config.gyro_range = SWING_GYRO_RANGE;
  imu_swing.mpu.config.accel_range = SWING_ACCEL_RANGE;
}

/**
 * Handle all the periodic tasks of the Navstik IMU components.
 * Read the MPU60x0 every periodic call
 */
void imu_swing_periodic(void)
{
  // Start reading the latest gyroscope data
  mpu60x0_i2c_periodic(&imu_swing.mpu);
}

/**
 * Handle all the events of the Navstik IMU components.
 * When there is data available convert it to the correct axis and save it in the imu structure.
 */
void imu_swing_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  /* MPU-60x0 event taks */
  mpu60x0_i2c_event(&imu_swing.mpu);

  if (imu_swing.mpu.data_available) {
    /* default orientation of the MPU is upside down and in plane mode
     * turn it to have rotorcraft mode by default */
    RATES_ASSIGN(imu.gyro_unscaled,
        -imu_swing.mpu.data_rates.rates.r,
        -imu_swing.mpu.data_rates.rates.q,
        -imu_swing.mpu.data_rates.rates.p);
    VECT3_ASSIGN(imu.accel_unscaled,
        -imu_swing.mpu.data_accel.vect.z,
        -imu_swing.mpu.data_accel.vect.y,
        -imu_swing.mpu.data_accel.vect.x);

    imu_swing.mpu.data_available = false;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);
  }
}

