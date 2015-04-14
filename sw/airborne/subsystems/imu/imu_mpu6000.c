/*
 * Copyright (C) 2013-2015 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/imu/imu_mpu6000.c
 * Driver for IMU with only MPU6000 via SPI.
 */

#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "mcu_periph/spi.h"

/* SPI defaults set in subsystem makefile, can be configured from airframe file */
PRINT_CONFIG_VAR(IMU_MPU_SPI_SLAVE_IDX)
PRINT_CONFIG_VAR(IMU_MPU_SPI_DEV)


/* MPU60x0 gyro/accel internal lowpass frequency */
#if !defined IMU_MPU_LOWPASS_FILTER && !defined  IMU_MPU_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define IMU_MPU_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define IMU_MPU_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define IMU_MPU_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define IMU_MPU_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#else
#error Non-default PERIODIC_FREQUENCY: please define MPU_HMC_LOWPASS_FILTER and MPU_HMC_SMPLRT_DIV.
#endif
#endif
PRINT_CONFIG_VAR(IMU_MPU_LOWPASS_FILTER)
PRINT_CONFIG_VAR(IMU_MPU_SMPLRT_DIV)

#ifndef IMU_MPU_GYRO_RANGE
#define IMU_MPU_GYRO_RANGE MPU60X0_GYRO_RANGE_2000
#endif
PRINT_CONFIG_VAR(IMU_MPU_GYRO_RANGE)

#ifndef IMU_MPU_ACCEL_RANGE
#define IMU_MPU_ACCEL_RANGE MPU60X0_ACCEL_RANGE_16G
#endif
PRINT_CONFIG_VAR(IMU_MPU_ACCEL_RANGE)


struct ImuMpu6000 imu_mpu_spi;

void imu_impl_init(void)
{
  mpu60x0_spi_init(&imu_mpu_spi.mpu, &IMU_MPU_SPI_DEV, IMU_MPU_SPI_SLAVE_IDX);
  // change the default configuration
  imu_mpu_spi.mpu.config.smplrt_div = IMU_MPU_SMPLRT_DIV;
  imu_mpu_spi.mpu.config.dlpf_cfg = IMU_MPU_LOWPASS_FILTER;
  imu_mpu_spi.mpu.config.gyro_range = IMU_MPU_GYRO_RANGE;
  imu_mpu_spi.mpu.config.accel_range = IMU_MPU_ACCEL_RANGE;
}


void imu_periodic(void)
{
  mpu60x0_spi_periodic(&imu_mpu_spi.mpu);
}

void imu_mpu_spi_event(void)
{
  mpu60x0_spi_event(&imu_mpu_spi.mpu);
  if (imu_mpu_spi.mpu.data_available) {
    uint32_t now_ts = get_sys_time_usec();
    RATES_COPY(imu.gyro_unscaled, imu_mpu_spi.mpu.data_rates.rates);
    VECT3_COPY(imu.accel_unscaled, imu_mpu_spi.mpu.data_accel.vect);
    imu_mpu_spi.mpu.data_available = FALSE;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_MPU6000_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_MPU6000_ID, now_ts, &imu.accel);
  }
}
