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
#include "peripherals/mpu60x0_spi.h"


static struct invensense2_t imu1;
#if IMU_CUBE_ORANGEPLUS
static struct invensense3_t imu2;
#else
static struct Mpu60x0_Spi imu2;
#endif
static struct invensense2_t imu3;

void imu_cube_init(void)
{
  struct Int32RMat rmat;
  struct Int32Eulers eulers;

  /* IMU 1 (ICM20649 not isolated) */
  imu1.abi_id = IMU_CUBE1_ID;
  imu1.bus = INVENSENSE2_SPI;
  imu1.spi.p = &CUBE_IMU1_SPI_DEV;
  imu1.spi.slave_idx = CUBE_IMU1_SPI_SLAVE_IDX;
  imu1.gyro_dlpf = INVENSENSE2_GYRO_DLPF_229HZ;
  imu1.gyro_range = INVENSENSE2_GYRO_RANGE_4000DPS;
  imu1.accel_dlpf = INVENSENSE2_ACCEL_DLPF_265HZ;
  imu1.accel_range = INVENSENSE2_ACCEL_RANGE_30G;
  invensense2_init(&imu1);

  // Rotation
  eulers.phi = ANGLE_BFP_OF_REAL(0);
  eulers.theta = ANGLE_BFP_OF_REAL(0);
  eulers.psi = ANGLE_BFP_OF_REAL(RadOfDeg(270));
  int32_rmat_of_eulers(&rmat, &eulers);
  imu_set_defaults_gyro(IMU_CUBE1_ID, &rmat, NULL, NULL);
  imu_set_defaults_accel(IMU_CUBE1_ID, &rmat, NULL, NULL);

#if IMU_CUBE_ORANGEPLUS
  /* IMU 2 (ICM20602 isolated) */
  imu2.abi_id = IMU_CUBE2_ID;
  imu2.parser = INVENSENSE3_PARSER_FIFO;
  imu2.bus = INVENSENSE3_SPI;
  imu2.spi.p = &CUBE_IMU2_SPI_DEV;
  imu2.spi.slave_idx = CUBE_IMU2_SPI_SLAVE_IDX;

  imu2.gyro_odr = INVENSENSE3_GYRO_ODR_4KHZ;
  imu2.gyro_range = INVENSENSE3_GYRO_RANGE_500DPS;
  imu2.accel_odr = INVENSENSE3_ACCEL_ODR_4KHZ;
  imu2.accel_range = INVENSENSE3_ACCEL_RANGE_32G;
  imu2.gyro_aaf = 977; // ~ODR/4
  imu2.accel_aaf = 213; // Fixed
  
  invensense3_init(&imu2);

  // Rotation
  eulers.phi = ANGLE_BFP_OF_REAL(RadOfDeg(180)),
  eulers.theta = ANGLE_BFP_OF_REAL(0);
  eulers.psi = ANGLE_BFP_OF_REAL(RadOfDeg(270));
  int32_rmat_of_eulers(&rmat, &eulers);
  imu_set_defaults_gyro(IMU_CUBE2_ID, &rmat, NULL, NULL);
  imu_set_defaults_accel(IMU_CUBE2_ID, &rmat, NULL, NULL);
#else
  /* IMU 2 (ICM20602 isolated) */
  mpu60x0_spi_init(&imu2, &CUBE_IMU2_SPI_DEV, CUBE_IMU2_SPI_SLAVE_IDX);
  // change the default configuration
  imu2.config.smplrt_div = 3;
  imu2.config.dlpf_cfg = MPU60X0_DLPF_256HZ;
  imu2.config.dlpf_cfg_acc = MPU60X0_DLPF_ACC_218HZ; // only for ICM sensors
  imu2.config.gyro_range = MPU60X0_GYRO_RANGE_2000;
  imu2.config.accel_range = MPU60X0_ACCEL_RANGE_16G;

  // Rotation
  eulers.phi = ANGLE_BFP_OF_REAL(RadOfDeg(180)),
  eulers.theta = ANGLE_BFP_OF_REAL(0);
  eulers.psi = ANGLE_BFP_OF_REAL(RadOfDeg(270));
  int32_rmat_of_eulers(&rmat, &eulers);
  imu_set_defaults_gyro(IMU_CUBE2_ID, &rmat, NULL, MPU60X0_GYRO_SENS_FRAC[MPU60X0_GYRO_RANGE_2000]);
  imu_set_defaults_accel(IMU_CUBE2_ID, &rmat, NULL, MPU60X0_ACCEL_SENS_FRAC[MPU60X0_ACCEL_RANGE_16G]);
#endif

  /* IMU 3 (ICM2094 isolated) */
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
  eulers.phi = ANGLE_BFP_OF_REAL(0),
  eulers.theta = ANGLE_BFP_OF_REAL(RadOfDeg(180));
  eulers.psi = ANGLE_BFP_OF_REAL(0);
  int32_rmat_of_eulers(&rmat, &eulers);
  imu_set_defaults_gyro(IMU_CUBE3_ID, &rmat, NULL, NULL);
  imu_set_defaults_accel(IMU_CUBE3_ID, &rmat, NULL, NULL);
}

void imu_cube_periodic(void)
{
  invensense2_periodic(&imu1);
#if IMU_CUBE_ORANGEPLUS
  invensense3_periodic(&imu2);
#else
  mpu60x0_spi_periodic(&imu2);
#endif
  invensense2_periodic(&imu3);
}

void imu_cube_event(void)
{
  invensense2_event(&imu1);

#if IMU_CUBE_ORANGEPLUS
  invensense3_event(&imu2);
#else
  mpu60x0_spi_event(&imu2);
  if (imu2.data_available) {
    uint32_t now_ts = get_sys_time_usec();

    // set channel order
    struct Int32Vect3 accel = {
      (int32_t)(imu2.data_accel.value[1]),
      (int32_t)(imu2.data_accel.value[0]),
      -(int32_t)(imu2.data_accel.value[2])
    };
    struct Int32Rates rates = {
      (int32_t)(imu2.data_rates.value[1]),
      (int32_t)(imu2.data_rates.value[0]),
      -(int32_t)(imu2.data_rates.value[2])
    };

    imu2.data_available = false;

    // Send the scaled values over ABI
    AbiSendMsgIMU_GYRO_RAW(IMU_CUBE2_ID, now_ts, &rates, 1, IMU_CUBE_PERIODIC_FREQ, imu2.temp);
    AbiSendMsgIMU_ACCEL_RAW(IMU_CUBE2_ID, now_ts, &accel, 1, IMU_CUBE_PERIODIC_FREQ, imu2.temp);
  }
#endif

  invensense2_event(&imu3);
}
