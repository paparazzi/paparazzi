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

#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/spi.h"
#include "peripherals/invensense2.h"


struct invensense2_t imu1;

void imu_cube_init(void)
{
  struct Int32RMat rmat;
  struct Int32Eulers eulers;

  /* IMU 2 (ICM2094) */
  imu2.abi_id = IMU_CUBE2_ID;
  imu2.bus = INVENSENSE2_SPI;
  imu2.spi.p = &CUBE_IMU2_SPI_DEV;
  imu2.spi.slave_idx = CUBE_IMU2_SPI_SLAVE_IDX;
  imu2.gyro_dlpf = INVENSENSE2_GYRO_DLPF_229HZ;
  imu2.gyro_range = INVENSENSE2_GYRO_RANGE_4000DPS;
  imu2.accel_dlpf = INVENSENSE2_ACCEL_DLPF_265HZ;
  imu2.accel_range = INVENSENSE2_ACCEL_RANGE_30G;
  invensense2_init(&imu2);
  
  // Rotation
  eulers.phi = ANGLE_BFP_OF_REAL(0),
  eulers.theta = ANGLE_BFP_OF_REAL(RadOfDeg(180));
  eulers.psi = ANGLE_BFP_OF_REAL(0);
  int32_rmat_of_eulers(&rmat, &eulers);
  imu_set_defaults_gyro(IMU_CUBE2_ID, &rmat, NULL, NULL);
  imu_set_defaults_accel(IMU_CUBE2_ID, &rmat, NULL, NULL);

  /* IMU 3 (ICM20649) */
  imu3.abi_id = IMU_CUBE3_ID;
  imu3.bus = INVENSENSE2_SPI;
  imu3.spi.p = &CUBE_IMU3_SPI_DEV;
  imu3.spi.slave_idx = CUBE_IMU3_SPI_SLAVE_IDX;
  imu3.gyro_dlpf = INVENSENSE2_GYRO_DLPF_229HZ;
  imu3.gyro_range = INVENSENSE2_GYRO_RANGE_4000DPS;
  imu3.accel_dlpf = INVENSENSE2_ACCEL_DLPF_265HZ;
  imu3.accel_range = INVENSENSE2_ACCEL_RANGE_30G;
  invensense2_init(&imu3);

  // Rotation
  eulers.phi = ANGLE_BFP_OF_REAL(0);
  eulers.theta = ANGLE_BFP_OF_REAL(0);
  eulers.psi = ANGLE_BFP_OF_REAL(RadOfDeg(270));
  int32_rmat_of_eulers(&rmat, &eulers);
  imu_set_defaults_gyro(IMU_CUBE3_ID, &rmat, NULL, NULL);
  imu_set_defaults_accel(IMU_CUBE3_ID, &rmat, NULL, NULL);
}

void imu_cube_periodic(void)
{
  invensense2_periodic(&imu1);
}

void imu_cube_event(void)
{
  invensense2_event(&imu1);
}
