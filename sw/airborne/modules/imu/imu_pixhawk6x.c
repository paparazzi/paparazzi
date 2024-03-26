/*
 * Copyright (C) 2022 Freek van tieen <freek.v.tienen@gmail.com>
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
 * @file modules/imu/imu_cube.c
 * Driver for the IMU's in the Cube autopilots.
 */

#include "generated/modules.h"
#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/spi.h"
#include "peripherals/invensense2.h"
#include "peripherals/invensense3.h"


static struct invensense3_t imu1;
static struct invensense3_t imu2;
static struct invensense2_t imu3;

void imu_pixhawk6x_init(void)
{
  struct Int32RMat rmat;
  struct Int32Eulers eulers;

  /* IMU 1 */
  imu1.abi_id = IMU_PIXHAWK1_ID;
  imu1.parser = INVENSENSE3_PARSER_FIFO;
  imu1.bus = INVENSENSE3_SPI;
  imu1.spi.p = &PIXHAWK6X_IMU1_SPI_DEV;
  imu1.spi.slave_idx = PIXHAWK6X_IMU1_SPI_SLAVE_IDX;

  imu1.gyro_odr = INVENSENSE3_GYRO_ODR_4KHZ;
  imu1.gyro_range = INVENSENSE3_GYRO_RANGE_2000DPS;
  imu1.accel_odr = INVENSENSE3_ACCEL_ODR_4KHZ;
  imu1.accel_range = INVENSENSE3_ACCEL_RANGE_32G;
  imu1.gyro_aaf = 977; // ~ODR/4
  imu1.accel_aaf = 213; // Fixed
  
  invensense3_init(&imu1);

  // Rotation
  eulers.phi = ANGLE_BFP_OF_REAL(0);
  eulers.theta = ANGLE_BFP_OF_REAL(0);
  eulers.psi = ANGLE_BFP_OF_REAL(RadOfDeg(90));
  int32_rmat_of_eulers(&rmat, &eulers);
  imu_set_defaults_gyro(IMU_PIXHAWK1_ID, &rmat, NULL, NULL);
  imu_set_defaults_accel(IMU_PIXHAWK1_ID, &rmat, NULL, NULL);

  /* IMU 2 */
  imu2.abi_id = IMU_PIXHAWK2_ID;
  imu2.parser = INVENSENSE3_PARSER_FIFO;
  imu2.bus = INVENSENSE3_SPI;
  imu2.spi.p = &PIXHAWK6X_IMU2_SPI_DEV;
  imu2.spi.slave_idx = PIXHAWK6X_IMU2_SPI_SLAVE_IDX;

  imu2.gyro_odr = INVENSENSE3_GYRO_ODR_4KHZ;
  imu2.gyro_range = INVENSENSE3_GYRO_RANGE_2000DPS;
  imu2.accel_odr = INVENSENSE3_ACCEL_ODR_4KHZ;
  imu2.accel_range = INVENSENSE3_ACCEL_RANGE_32G;
  imu2.gyro_aaf = 977; // ~ODR/4
  imu2.accel_aaf = 213; // Fixed
  
  invensense3_init(&imu2);

  // Rotation
  eulers.phi = ANGLE_BFP_OF_REAL(0);
  eulers.theta = ANGLE_BFP_OF_REAL(RadOfDeg(180));
  eulers.psi = ANGLE_BFP_OF_REAL(RadOfDeg(90));
  int32_rmat_of_eulers(&rmat, &eulers);
  imu_set_defaults_gyro(IMU_PIXHAWK2_ID, &rmat, NULL, NULL);
  imu_set_defaults_accel(IMU_PIXHAWK2_ID, &rmat, NULL, NULL);

  /* IMU 3 */
  imu3.abi_id = IMU_PIXHAWK3_ID;
  imu3.bus = INVENSENSE2_SPI;
  imu3.spi.p = &PIXHAWK6X_IMU3_SPI_DEV;
  imu3.spi.slave_idx = PIXHAWK6X_IMU3_SPI_SLAVE_IDX;
  imu3.gyro_dlpf = INVENSENSE2_GYRO_DLPF_229HZ;
  imu3.gyro_range = INVENSENSE2_GYRO_RANGE_4000DPS;
  imu3.accel_dlpf = INVENSENSE2_ACCEL_DLPF_265HZ;
  imu3.accel_range = INVENSENSE2_ACCEL_RANGE_30G;
  invensense2_init(&imu3);
  
  // Rotation
  eulers.phi = ANGLE_BFP_OF_REAL(0),
  eulers.theta = ANGLE_BFP_OF_REAL(0);
  eulers.psi = ANGLE_BFP_OF_REAL(RadOfDeg(180));
  int32_rmat_of_eulers(&rmat, &eulers);
  imu_set_defaults_gyro(IMU_PIXHAWK3_ID, &rmat, NULL, NULL);
  imu_set_defaults_accel(IMU_PIXHAWK3_ID, &rmat, NULL, NULL);
}

void imu_pixhawk6x_periodic(void)
{
  invensense3_periodic(&imu1);
  invensense3_periodic(&imu2);
  invensense2_periodic(&imu3);
}

void imu_pixhawk6x_event(void)
{
  invensense3_event(&imu1);
  invensense3_event(&imu2);
  invensense2_event(&imu3);
}
