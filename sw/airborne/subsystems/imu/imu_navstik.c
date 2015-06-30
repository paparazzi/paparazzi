/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file subsystems/imu/imu_navstik.c
 * Driver for the Navstik magnetometer, accelerometer and gyroscope
 */

#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "mcu_periph/i2c.h"


/* defaults suitable for Navstik */
#ifndef NAVSTIK_MAG_I2C_DEV
#define NAVSTIK_MAG_I2C_DEV i2c3
#endif
PRINT_CONFIG_VAR(NAVSTIK_MAG_I2C_DEV)

#ifndef NAVSTIK_MPU_I2C_DEV
#define NAVSTIK_MPU_I2C_DEV i2c1
#endif
PRINT_CONFIG_VAR(NAVSTIK_MPU_I2C_DEV)

#if !defined NAVSTIK_LOWPASS_FILTER && !defined  NAVSTIK_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define NAVSTIK_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define NAVSTIK_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define NAVSTIK_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define NAVSTIK_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#endif
#endif
PRINT_CONFIG_VAR(NAVSTIK_SMPLRT_DIV)
PRINT_CONFIG_VAR(NAVSTIK_LOWPASS_FILTER)

#ifndef NAVSTIK_GYRO_RANGE
#define NAVSTIK_GYRO_RANGE MPU60X0_GYRO_RANGE_1000
#endif
PRINT_CONFIG_VAR(NAVSTIK_GYRO_RANGE)

#ifndef NAVSTIK_ACCEL_RANGE
#define NAVSTIK_ACCEL_RANGE MPU60X0_ACCEL_RANGE_8G
#endif
PRINT_CONFIG_VAR(NAVSTIK_ACCEL_RANGE)

/** Basic Navstik IMU data */
struct ImuNavstik imu_navstik;

/**
 * Navstik IMU initializtion of the MPU-60x0 and HMC58xx
 */
void imu_impl_init(void)
{
  /* MPU-60X0 */
  mpu60x0_i2c_init(&imu_navstik.mpu, &(NAVSTIK_MPU_I2C_DEV), MPU60X0_ADDR_ALT);
  imu_navstik.mpu.config.smplrt_div = NAVSTIK_SMPLRT_DIV;
  imu_navstik.mpu.config.dlpf_cfg = NAVSTIK_LOWPASS_FILTER;
  imu_navstik.mpu.config.gyro_range = NAVSTIK_GYRO_RANGE;
  imu_navstik.mpu.config.accel_range = NAVSTIK_ACCEL_RANGE;

  /* HMC58XX */
  hmc58xx_init(&imu_navstik.hmc, &(NAVSTIK_MAG_I2C_DEV), HMC58XX_ADDR);
}

/**
 * Handle all the periodic tasks of the Navstik IMU components.
 * Read the MPU60x0 every periodic call and the HMC58XX every 10th call.
 */
void imu_periodic(void)
{
  // Start reading the latest gyroscope data
  mpu60x0_i2c_periodic(&imu_navstik.mpu);

  // Read HMC58XX at 50Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(10, hmc58xx_periodic(&imu_navstik.hmc));
}

/**
 * Handle all the events of the Navstik IMU components.
 * When there is data available convert it to the correct axis and save it in the imu structure.
 */
void imu_navstik_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  /* MPU-60x0 event taks */
  mpu60x0_i2c_event(&imu_navstik.mpu);

  if (imu_navstik.mpu.data_available) {
    /* default orientation as should be printed on the pcb, z-down, ICs down */
    RATES_COPY(imu.gyro_unscaled, imu_navstik.mpu.data_rates.rates);
    VECT3_COPY(imu.accel_unscaled, imu_navstik.mpu.data_accel.vect);

    imu_navstik.mpu.data_available = FALSE;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);
  }

  /* HMC58XX event task */
  hmc58xx_event(&imu_navstik.hmc);
  if (imu_navstik.hmc.data_available) {
    imu.mag_unscaled.x =  imu_navstik.hmc.data.vect.y;
    imu.mag_unscaled.y = -imu_navstik.hmc.data.vect.x;
    imu.mag_unscaled.z =  imu_navstik.hmc.data.vect.z;
    imu_navstik.hmc.data_available = FALSE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_BOARD_ID, now_ts, &imu.mag);
  }
}
