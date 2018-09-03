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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file subsystems/imu/imu_disco.c
 * Driver for the Disco magnetometer, accelerometer and gyroscope
 */

#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "mcu_periph/i2c.h"


/* I2C is hardwired on Disco autopilot */
#define DISCO_MAG_I2C_DEV i2c1
PRINT_CONFIG_VAR(DISCO_MAG_I2C_DEV)
#define DISCO_MPU_I2C_DEV i2c2
PRINT_CONFIG_VAR(DISCO_MPU_I2C_DEV)


#if !defined DISCO_LOWPASS_FILTER && !defined  DISCO_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define DISCO_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define DISCO_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define DISCO_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define DISCO_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#endif
#endif

PRINT_CONFIG_VAR(DISCO_SMPLRT_DIV)
PRINT_CONFIG_VAR(DISCO_LOWPASS_FILTER)

PRINT_CONFIG_VAR(DISCO_GYRO_RANGE)
PRINT_CONFIG_VAR(DISCO_ACCEL_RANGE)

/** Disco IMU data */
struct ImuDisco imu_disco;

/**
 * Disco IMU initializtion of the MPU-60x0 and HMC58xx
 */
void imu_disco_init(void)
{
  /* MPU-60X0 */
  mpu60x0_i2c_init(&imu_disco.mpu, &(DISCO_MPU_I2C_DEV), MPU60X0_ADDR);
  imu_disco.mpu.config.smplrt_div = DISCO_SMPLRT_DIV;
  imu_disco.mpu.config.dlpf_cfg = DISCO_LOWPASS_FILTER;
  imu_disco.mpu.config.gyro_range = DISCO_GYRO_RANGE;
  imu_disco.mpu.config.accel_range = DISCO_ACCEL_RANGE;

  /* AKM8963 */
  ak8963_init(&imu_disco.ak, &(DISCO_MAG_I2C_DEV), AK8963_ADDR);
}

/**
 * Handle all the periodic tasks of the Disco IMU components.
 * Read the MPU60x0 every periodic call and the AKM8963 every 10th call.
 */
void imu_disco_periodic(void)
{
  // Start reading the latest gyroscope data
  mpu60x0_i2c_periodic(&imu_disco.mpu);

  // AKM8963
  ak8963_periodic(&imu_disco.ak);
}

/**
 * Handle all the events of the Disco IMU components.
 * When there is data available convert it to the correct axis and save it in the imu structure.
 */
void imu_disco_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  /* MPU-60x0 event taks */
  mpu60x0_i2c_event(&imu_disco.mpu);

  if (imu_disco.mpu.data_available) {
    /* set correct orientation here */
    RATES_ASSIGN(imu.gyro_unscaled,
        -imu_disco.mpu.data_rates.rates.p,
        -imu_disco.mpu.data_rates.rates.q,
        imu_disco.mpu.data_rates.rates.r);
    VECT3_ASSIGN(imu.accel_unscaled,
        -imu_disco.mpu.data_accel.vect.x,
        -imu_disco.mpu.data_accel.vect.y,
        imu_disco.mpu.data_accel.vect.z);

    imu_disco.mpu.data_available = false;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);
  }

  /* AKM8963 event task */
  ak8963_event(&imu_disco.ak);

  if (imu_disco.ak.data_available) {
    VECT3_ASSIGN(imu.mag_unscaled,
        imu_disco.ak.data.vect.x,
        imu_disco.ak.data.vect.y,
        imu_disco.ak.data.vect.z);

    imu_disco.ak.data_available = false;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_BOARD_ID, now_ts, &imu.mag);
  }
}
