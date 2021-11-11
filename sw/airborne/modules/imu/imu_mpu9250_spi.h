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
 * @file modules/imu/imu_mpu9250_spi.h
 *
 * IMU driver for the MPU9250 using SPI
 *
 */

#ifndef IMU_MPU9250_SPI_H
#define IMU_MPU9250_SPI_H

#include "std.h"
#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "peripherals/mpu9250_spi.h"


#ifndef IMU_MPU9250_GYRO_RANGE
#define IMU_MPU9250_GYRO_RANGE MPU9250_GYRO_RANGE_1000
#endif

#ifndef IMU_MPU9250_ACCEL_RANGE
#define IMU_MPU9250_ACCEL_RANGE MPU9250_ACCEL_RANGE_8G
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS MPU9250_GYRO_SENS[IMU_MPU9250_GYRO_RANGE]
#define IMU_GYRO_P_SENS_NUM MPU9250_GYRO_SENS_FRAC[IMU_MPU9250_GYRO_RANGE][0]
#define IMU_GYRO_P_SENS_DEN MPU9250_GYRO_SENS_FRAC[IMU_MPU9250_GYRO_RANGE][1]
#define IMU_GYRO_Q_SENS MPU9250_GYRO_SENS[IMU_MPU9250_GYRO_RANGE]
#define IMU_GYRO_Q_SENS_NUM MPU9250_GYRO_SENS_FRAC[IMU_MPU9250_GYRO_RANGE][0]
#define IMU_GYRO_Q_SENS_DEN MPU9250_GYRO_SENS_FRAC[IMU_MPU9250_GYRO_RANGE][1]
#define IMU_GYRO_R_SENS MPU9250_GYRO_SENS[IMU_MPU9250_GYRO_RANGE]
#define IMU_GYRO_R_SENS_NUM MPU9250_GYRO_SENS_FRAC[IMU_MPU9250_GYRO_RANGE][0]
#define IMU_GYRO_R_SENS_DEN MPU9250_GYRO_SENS_FRAC[IMU_MPU9250_GYRO_RANGE][1]
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS MPU9250_ACCEL_SENS[IMU_MPU9250_ACCEL_RANGE]
#define IMU_ACCEL_X_SENS_NUM MPU9250_ACCEL_SENS_FRAC[IMU_MPU9250_ACCEL_RANGE][0]
#define IMU_ACCEL_X_SENS_DEN MPU9250_ACCEL_SENS_FRAC[IMU_MPU9250_ACCEL_RANGE][1]
#define IMU_ACCEL_Y_SENS MPU9250_ACCEL_SENS[IMU_MPU9250_ACCEL_RANGE]
#define IMU_ACCEL_Y_SENS_NUM MPU9250_ACCEL_SENS_FRAC[IMU_MPU9250_ACCEL_RANGE][0]
#define IMU_ACCEL_Y_SENS_DEN MPU9250_ACCEL_SENS_FRAC[IMU_MPU9250_ACCEL_RANGE][1]
#define IMU_ACCEL_Z_SENS MPU9250_ACCEL_SENS[IMU_MPU9250_ACCEL_RANGE]
#define IMU_ACCEL_Z_SENS_NUM MPU9250_ACCEL_SENS_FRAC[IMU_MPU9250_ACCEL_RANGE][0]
#define IMU_ACCEL_Z_SENS_DEN MPU9250_ACCEL_SENS_FRAC[IMU_MPU9250_ACCEL_RANGE][1]
#endif


struct ImuMpu9250 {
  struct Mpu9250_Spi mpu;

  struct spi_transaction wait_slave4_trans;
  volatile uint8_t wait_slave4_tx_buf[1];
  volatile uint8_t wait_slave4_rx_buf[2];
  volatile bool slave4_ready;
};

extern struct ImuMpu9250 imu_mpu9250;

extern void imu_mpu9250_init(void);
extern void imu_mpu9250_periodic(void);
extern void imu_mpu9250_event(void);

#endif /* IMU_MPU9250_SPI_H */
