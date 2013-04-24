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
 * @file subsystems/imu/imu_aspirin_2.c
 * Driver for the Aspirin v2.x IMU using SPI for the MPU6000.
 */

#include "subsystems/imu.h"

#include "mcu_periph/i2c.h"
#include "mcu_periph/spi.h"


/* defaults suitable for Lisa */
#ifndef ASPIRIN_2_SPI_SLAVE_IDX
#define ASPIRIN_2_SPI_SLAVE_IDX SPI_SLAVE2
#endif
PRINT_CONFIG_VAR(ASPIRIN_2_SPI_SLAVE_IDX)

#ifndef ASPIRIN_2_SPI_DEV
#define ASPIRIN_2_SPI_DEV spi2
#endif
PRINT_CONFIG_VAR(ASPIRIN_2_SPI_DEV)

#ifndef ASPIRIN_2_I2C_DEV
#define ASPIRIN_2_I2C_DEV i2c2
#endif
PRINT_CONFIG_VAR(ASPIRIN_2_I2C_DEV)


/* gyro internal lowpass frequency */
#if !defined ASPIRIN_2_LOWPASS_FILTER && !defined  ASPIRIN_2_SMPLRT_DIV
#define ASPIRIN_2_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define ASPIRIN_2_SMPLRT_DIV 1
PRINT_CONFIG_MSG("Gyro/Accel output rate is 500Hz")
#endif
PRINT_CONFIG_VAR(ASPIRIN_2_LOWPASS_FILTER)
PRINT_CONFIG_VAR(ASPIRIN_2_SMPLRT_DIV)

#ifndef ASPIRIN_2_GYRO_RANGE
#define ASPIRIN_2_GYRO_RANGE MPU60X0_GYRO_RANGE_2000
#endif
PRINT_CONFIG_VAR(ASPIRIN_2_GYRO_RANGE)

#ifndef ASPIRIN_2_ACCEL_RANGE
#define ASPIRIN_2_ACCEL_RANGE MPU60X0_ACCEL_RANGE_16G
#endif
PRINT_CONFIG_VAR(ASPIRIN_2_ACCEL_RANGE)


struct ImuAspirin2 imu_aspirin2;

void imu_impl_init(void)
{
  imu_aspirin2.accel_valid = FALSE;
  imu_aspirin2.gyro_valid = FALSE;
  imu_aspirin2.mag_valid = FALSE;

  mpu60x0_spi_init(&imu_aspirin2.mpu, &(ASPIRIN_2_SPI_DEV), ASPIRIN_2_SPI_SLAVE_IDX);
  // change the default configuration
  imu_aspirin2.mpu.config.smplrt_div = ASPIRIN_2_SMPLRT_DIV;
  imu_aspirin2.mpu.config.dlpf_cfg = ASPIRIN_2_LOWPASS_FILTER;
  imu_aspirin2.mpu.config.gyro_range = ASPIRIN_2_GYRO_RANGE;
  imu_aspirin2.mpu.config.accel_range = ASPIRIN_2_ACCEL_RANGE;
  //imu_aspirin2.mpu.config.i2c_bypass = FALSE;
  //imu_aspirin2.mpu.config.drdy_int_enable = TRUE;

  //hmc58xx_init(&imu_aspirin2.mag_hmc, &(ASPIRIN_2_I2C_DEV), HMC58XX_ADDR);
}


void imu_periodic(void)
{
  mpu60x0_spi_periodic(&imu_aspirin2.mpu);

  // Read HMC58XX at 50Hz (main loop for rotorcraft: 512Hz)
  //RunOnceEvery(10, hmc58xx_periodic(&imu_aspirin2.mag_hmc));
}

void imu_aspirin2_event(void)
{
  mpu60x0_spi_event(&imu_aspirin2.mpu);
  if (imu_aspirin2.mpu.data_available) {
    RATES_COPY(imu.gyro_unscaled, imu_aspirin2.mpu.data_rates.rates);
    VECT3_COPY(imu.accel_unscaled, imu_aspirin2.mpu.data_accel.vect);
    imu_aspirin2.mpu.data_available = FALSE;
    imu_aspirin2.gyro_valid = TRUE;
    imu_aspirin2.accel_valid = TRUE;
  }

#if 0
  /* HMC58XX event task */
  hmc58xx_event(&imu_aspirin2.mag_hmc);
  if (imu_aspirin2.mag_hmc.data_available) {
    imu.mag_unscaled.x =  imu_aspirin2.mag_hmc.data.vect.y;
    imu.mag_unscaled.y = -imu_aspirin2.mag_hmc.data.vect.x;
    imu.mag_unscaled.z =  imu_aspirin2.mag_hmc.data.vect.z;
    imu_aspirin2.mag_hmc.data_available = FALSE;
    imu_aspirin2.mag_valid = TRUE;
  }
#endif
}
