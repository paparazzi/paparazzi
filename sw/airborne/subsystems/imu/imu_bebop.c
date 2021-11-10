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
 * @file subsystems/imu/imu_bebop.c
 * Driver for the Bebop (2) magnetometer, accelerometer and gyroscope
 */

#include "subsystems/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/i2c.h"


/* defaults suitable for Bebop */
#ifndef BEBOP_MAG_I2C_DEV
#define BEBOP_MAG_I2C_DEV i2c1
#endif
PRINT_CONFIG_VAR(BEBOP_MAG_I2C_DEV)

#ifndef BEBOP_MPU_I2C_DEV
#define BEBOP_MPU_I2C_DEV i2c2
#endif
PRINT_CONFIG_VAR(BEBOP_MPU_I2C_DEV)

#if !defined BEBOP_LOWPASS_FILTER && !defined  BEBOP_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define BEBOP_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define BEBOP_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define BEBOP_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define BEBOP_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#endif
#endif
PRINT_CONFIG_VAR(BEBOP_SMPLRT_DIV)
PRINT_CONFIG_VAR(BEBOP_LOWPASS_FILTER)

PRINT_CONFIG_VAR(BEBOP_GYRO_RANGE)
PRINT_CONFIG_VAR(BEBOP_ACCEL_RANGE)

struct OrientationReps imu_to_mag_bebop;    ///< IMU to magneto rotation

/** Basic Navstik IMU data */
struct ImuBebop imu_bebop;

/**
 * Navstik IMU initializtion of the MPU-60x0 and HMC58xx
 */
void imu_bebop_init(void)
{
  /* MPU-60X0 */
  mpu60x0_i2c_init(&imu_bebop.mpu, &(BEBOP_MPU_I2C_DEV), MPU60X0_ADDR);
  imu_bebop.mpu.config.smplrt_div = BEBOP_SMPLRT_DIV;
  imu_bebop.mpu.config.dlpf_cfg = BEBOP_LOWPASS_FILTER;
  imu_bebop.mpu.config.gyro_range = BEBOP_GYRO_RANGE;
  imu_bebop.mpu.config.accel_range = BEBOP_ACCEL_RANGE;

  /* AKM8963 */
  ak8963_init(&imu_bebop.ak, &(BEBOP_MAG_I2C_DEV), AK8963_ADDR);

#if BEBOP_VERSION2
  //the magnetometer of the bebop2 is located on the gps board,
  //which is under a slight angle
  struct FloatEulers imu_to_mag_eulers =
  { 0.0, RadOfDeg(8.5), M_PI };
  orientationSetEulers_f(&imu_to_mag_bebop, &imu_to_mag_eulers);
#endif
}

/**
 * Handle all the periodic tasks of the Navstik IMU components.
 * Read the MPU60x0 every periodic call and the HMC58XX every 10th call.
 */
void imu_bebop_periodic(void)
{
  // Start reading the latest gyroscope data
  mpu60x0_i2c_periodic(&imu_bebop.mpu);

  // AKM8963
  ak8963_periodic(&imu_bebop.ak);
}

/**
 * Handle all the events of the Navstik IMU components.
 * When there is data available convert it to the correct axis and save it in the imu structure.
 */
void imu_bebop_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  /* MPU-60x0 event taks */
  mpu60x0_i2c_event(&imu_bebop.mpu);

  if (imu_bebop.mpu.data_available) {
    /* default orientation of the MPU is upside down sor corrigate this here */
    RATES_ASSIGN(imu.gyro_unscaled, imu_bebop.mpu.data_rates.rates.p, -imu_bebop.mpu.data_rates.rates.q,
                 -imu_bebop.mpu.data_rates.rates.r);
    VECT3_ASSIGN(imu.accel_unscaled, imu_bebop.mpu.data_accel.vect.x, -imu_bebop.mpu.data_accel.vect.y,
                 -imu_bebop.mpu.data_accel.vect.z);

    imu_bebop.mpu.data_available = false;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);
  }

  /* AKM8963 event task */
  ak8963_event(&imu_bebop.ak);

  if (imu_bebop.ak.data_available) {
#if BEBOP_VERSION2
    struct Int32Vect3 mag_temp;
    // In the second bebop version the magneto is turned 90 degrees
    VECT3_ASSIGN(mag_temp, imu_bebop.ak.data.vect.x, imu_bebop.ak.data.vect.y, imu_bebop.ak.data.vect.z);
    // Rotate the magneto
    struct Int32RMat *imu_to_mag_rmat = orientationGetRMat_i(&imu_to_mag_bebop);
    int32_rmat_vmult(&imu.mag_unscaled, imu_to_mag_rmat, &mag_temp);
#else //BEBOP regular first verion
    VECT3_ASSIGN(imu.mag_unscaled, -imu_bebop.ak.data.vect.y, imu_bebop.ak.data.vect.x, imu_bebop.ak.data.vect.z);
#endif

    imu_bebop.ak.data_available = false;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_BOARD_ID, now_ts, &imu.mag);
  }
}
