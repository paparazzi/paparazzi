/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file modules/imu/imu_aspirin_2_spi.h
 * Driver for the Aspirin v2.x IMU using SPI for the MPU6000.
 */

#ifndef IMU_ASPIRIN_2_SPI_H
#define IMU_ASPIRIN_2_SPI_H

#include "std.h"
#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "peripherals/mpu60x0_spi.h"

#ifndef ASPIRIN_2_GYRO_RANGE
#define ASPIRIN_2_GYRO_RANGE MPU60X0_GYRO_RANGE_2000
#endif

#ifndef ASPIRIN_2_ACCEL_RANGE
#define ASPIRIN_2_ACCEL_RANGE MPU60X0_ACCEL_RANGE_16G
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS MPU60X0_GYRO_SENS[ASPIRIN_2_GYRO_RANGE]
#define IMU_GYRO_P_SENS_NUM MPU60X0_GYRO_SENS_FRAC[ASPIRIN_2_GYRO_RANGE][0]
#define IMU_GYRO_P_SENS_DEN MPU60X0_GYRO_SENS_FRAC[ASPIRIN_2_GYRO_RANGE][1]
#define IMU_GYRO_Q_SENS MPU60X0_GYRO_SENS[ASPIRIN_2_GYRO_RANGE]
#define IMU_GYRO_Q_SENS_NUM MPU60X0_GYRO_SENS_FRAC[ASPIRIN_2_GYRO_RANGE][0]
#define IMU_GYRO_Q_SENS_DEN MPU60X0_GYRO_SENS_FRAC[ASPIRIN_2_GYRO_RANGE][1]
#define IMU_GYRO_R_SENS MPU60X0_GYRO_SENS[ASPIRIN_2_GYRO_RANGE]
#define IMU_GYRO_R_SENS_NUM MPU60X0_GYRO_SENS_FRAC[ASPIRIN_2_GYRO_RANGE][0]
#define IMU_GYRO_R_SENS_DEN MPU60X0_GYRO_SENS_FRAC[ASPIRIN_2_GYRO_RANGE][1]
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS MPU60X0_ACCEL_SENS[ASPIRIN_2_ACCEL_RANGE]
#define IMU_ACCEL_X_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[ASPIRIN_2_ACCEL_RANGE][0]
#define IMU_ACCEL_X_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[ASPIRIN_2_ACCEL_RANGE][1]
#define IMU_ACCEL_Y_SENS MPU60X0_ACCEL_SENS[ASPIRIN_2_ACCEL_RANGE]
#define IMU_ACCEL_Y_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[ASPIRIN_2_ACCEL_RANGE][0]
#define IMU_ACCEL_Y_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[ASPIRIN_2_ACCEL_RANGE][1]
#define IMU_ACCEL_Z_SENS MPU60X0_ACCEL_SENS[ASPIRIN_2_ACCEL_RANGE]
#define IMU_ACCEL_Z_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[ASPIRIN_2_ACCEL_RANGE][0]
#define IMU_ACCEL_Z_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[ASPIRIN_2_ACCEL_RANGE][1]
#endif

struct ImuAspirin2Spi {
  struct Mpu60x0_Spi mpu;

  struct spi_transaction wait_slave4_trans;
  volatile uint8_t wait_slave4_tx_buf[1];
  volatile uint8_t wait_slave4_rx_buf[2];
  volatile bool slave4_ready;
};

extern struct ImuAspirin2Spi imu_aspirin2;

extern void imu_aspirin2_init(void);
extern void imu_aspirin2_event(void);
extern void imu_aspirin2_periodic(void);

#endif /* IMU_ASPIRIN_2_H */
