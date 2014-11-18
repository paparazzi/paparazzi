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
 * @file subsystems/imu/imu_mpu9250_i2c.c
 *
 * IMU driver for the MPU9250 using I2C
 *
 */

#include "subsystems/imu.h"

#include "mcu_periph/i2c.h"

#if !defined IMU_MPU9250_GYRO_LOWPASS_FILTER && !defined IMU_MPU9250_ACCEL_LOWPASS_FILTER && !defined  IMU_MPU9250_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 41Hz, Delay 5.9ms
 * Gyroscope: Bandwidth 41Hz, Delay 5.9ms sampling 1kHz
 * Output rate: 100Hz
 */
#define IMU_MPU9250_GYRO_LOWPASS_FILTER MPU9250_DLPF_GYRO_41HZ
#define IMU_MPU9250_ACCEL_LOWPASS_FILTER MPU9250_DLPF_ACCEL_41HZ
#define IMU_MPU9250_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 184Hz, Delay 5.8ms
 * Gyroscope: Bandwidth 250Hz, Delay 0.97ms sampling 8kHz
 * Output rate: 2kHz
 */
#define IMU_MPU9250_GYRO_LOWPASS_FILTER MPU9250_DLPF_GYRO_250HZ
#define IMU_MPU9250_ACCEL_LOWPASS_FILTER MPU9250_DLPF_ACCEL_184HZ
#define IMU_MPU9250_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#else
/* By default, don't go too fast */
#define IMU_MPU9250_SMPLRT_DIV 9
#define IMU_MPU9250_GYRO_LOWPASS_FILTER MPU9250_DLPF_GYRO_41HZ
#define IMU_MPU9250_ACCEL_LOWPASS_FILTER MPU9250_DLPF_ACCEL_41HZ
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#endif
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_SMPLRT_DIV)
PRINT_CONFIG_VAR(IMU_MPU9250_GYRO_LOWPASS_FILTER)
PRINT_CONFIG_VAR(IMU_MPU9250_ACCEL_LOWPASS_FILTER)

#ifndef IMU_MPU9250_GYRO_RANGE
#define IMU_MPU9250_GYRO_RANGE MPU9250_GYRO_RANGE_1000
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_GYRO_RANGE)

#ifndef IMU_MPU9250_ACCEL_RANGE
#define IMU_MPU9250_ACCEL_RANGE MPU9250_ACCEL_RANGE_8G
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_ACCEL_RANGE)

#ifndef IMU_MPU9250_I2C_ADDR
#define IMU_MPU9250_I2C_ADDR MPU9250_ADDR_ALT
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_I2C_ADDR)

// Default channels order
#ifndef IMU_MPU9250_CHAN_X
#define IMU_MPU9250_CHAN_X 0
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_CHAN_X)
#ifndef IMU_MPU9250_CHAN_Y
#define IMU_MPU9250_CHAN_Y 1
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_CHAN_Y)
#ifndef IMU_MPU9250_CHAN_Z
#define IMU_MPU9250_CHAN_Z 2
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_CHAN_Z)


struct ImuMpu9250 imu_mpu9250;

void imu_impl_init(void)
{
  /* MPU9250 */
  mpu9250_i2c_init(&imu_mpu9250.mpu, &(IMU_MPU9250_I2C_DEV), IMU_MPU9250_I2C_ADDR);
  // change the default configuration
  imu_mpu9250.mpu.config.smplrt_div = IMU_MPU9250_SMPLRT_DIV;
  imu_mpu9250.mpu.config.dlpf_gyro_cfg = IMU_MPU9250_GYRO_LOWPASS_FILTER;
  imu_mpu9250.mpu.config.dlpf_accel_cfg = IMU_MPU9250_ACCEL_LOWPASS_FILTER;
  imu_mpu9250.mpu.config.gyro_range = IMU_MPU9250_GYRO_RANGE;
  imu_mpu9250.mpu.config.accel_range = IMU_MPU9250_ACCEL_RANGE;

  imu_mpu9250.gyro_valid = FALSE;
  imu_mpu9250.accel_valid = FALSE;
  imu_mpu9250.mag_valid = FALSE;
}

void imu_periodic(void)
{
  mpu9250_i2c_periodic(&imu_mpu9250.mpu);
}

void imu_mpu9250_event(void)
{
  // If the MPU9250 I2C transaction has succeeded: convert the data
  mpu9250_i2c_event(&imu_mpu9250.mpu);

  if (imu_mpu9250.mpu.data_available) {
    // set channel order
    struct Int32Vect3 accel = {
      (int32_t)(imu_mpu9250.mpu.data_accel.value[IMU_MPU9250_CHAN_X]),
      (int32_t)(imu_mpu9250.mpu.data_accel.value[IMU_MPU9250_CHAN_Y]),
      (int32_t)(imu_mpu9250.mpu.data_accel.value[IMU_MPU9250_CHAN_Z])
    };
    struct Int32Rates rates = {
      (int32_t)(imu_mpu9250.mpu.data_rates.value[IMU_MPU9250_CHAN_X]),
      (int32_t)(imu_mpu9250.mpu.data_rates.value[IMU_MPU9250_CHAN_Y]),
      (int32_t)(imu_mpu9250.mpu.data_rates.value[IMU_MPU9250_CHAN_Z])
    };
    // unscaled vector
    VECT3_COPY(imu.accel_unscaled, accel);
    RATES_COPY(imu.gyro_unscaled, rates);

    imu_mpu9250.mpu.data_available = FALSE;
    imu_mpu9250.gyro_valid = TRUE;
    imu_mpu9250.accel_valid = TRUE;
  }
  // Test if mag data are updated
  if (imu_mpu9250.mpu.akm.data_available) {
    struct Int32Vect3 mag = {
      (int32_t)(imu_mpu9250.mpu.akm.data.value[IMU_MPU9250_CHAN_Y]),
      (int32_t)(imu_mpu9250.mpu.akm.data.value[IMU_MPU9250_CHAN_X]),
      -(int32_t)(imu_mpu9250.mpu.akm.data.value[IMU_MPU9250_CHAN_Z])
    };
    VECT3_COPY(imu.mag_unscaled, mag);
    imu_mpu9250.mag_valid = TRUE;
    imu_mpu9250.mpu.akm.data_available = FALSE;
  }

}

