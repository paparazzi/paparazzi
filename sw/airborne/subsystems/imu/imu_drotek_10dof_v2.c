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
 * @file subsystems/imu/imu_drotek_10dof_v2.c
 *
 * Driver for the Drotek 10DOF V2 IMU.
 * MPU6050 + HMC5883 + MS5611
 *
 * @todo MS5611 baro not read yet
 */

#include "subsystems/imu.h"

#include "mcu_periph/i2c.h"

#if !defined DROTEK_2_LOWPASS_FILTER && !defined  DROTEK_2_SMPLRT_DIV
#define DROTEK_2_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define DROTEK_2_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz")
#endif
PRINT_CONFIG_VAR(DROTEK_2_SMPLRT_DIV)
PRINT_CONFIG_VAR(DROTEK_2_LOWPASS_FILTER)

#ifndef DROTEK_2_GYRO_RANGE
#define DROTEK_2_GYRO_RANGE MPU60X0_GYRO_RANGE_1000
#endif
PRINT_CONFIG_VAR(DROTEK_2_GYRO_RANGE)

#ifndef DROTEK_2_ACCEL_RANGE
#define DROTEK_2_ACCEL_RANGE MPU60X0_ACCEL_RANGE_8G
#endif
PRINT_CONFIG_VAR(DROTEK_2_ACCEL_RANGE)

struct ImuDrotek2 imu_drotek2;

void imu_impl_init(void)
{
  /* MPU-60X0 */
  mpu60x0_i2c_init(&imu_drotek2.mpu, &(DROTEK_2_I2C_DEV), MPU60X0_ADDR_ALT);
  // change the default configuration
  imu_drotek2.mpu.config.smplrt_div = DROTEK_2_SMPLRT_DIV;
  imu_drotek2.mpu.config.dlpf_cfg = DROTEK_2_LOWPASS_FILTER;
  imu_drotek2.mpu.config.gyro_range = DROTEK_2_GYRO_RANGE;
  imu_drotek2.mpu.config.accel_range = DROTEK_2_ACCEL_RANGE;

  /* HMC5883 magnetometer */
  hmc58xx_init(&imu_drotek2.hmc, &(DROTEK_2_I2C_DEV), HMC58XX_ADDR);
  /* set callback function to configure mag */
  imu_drotek2.mpu.config.slaves[0].configure = &imu_drotek2_configure_mag_slave;

#if DROTEK_2_MAG_SLAVE
  /* Set MPU I2C master clock to 400kHz */
  imu_drotek2.mpu.config.mst_clk = MPU60X0_MST_CLK_400KHZ;
  /* Enable I2C slave0 delayed sample rate */
  imu_drotek2.mpu.config.mst_delay = 1;
#else
  // use hmc mag via passthrough
  imu_drotek2.mpu.config.i2c_bypass = TRUE;
#endif

  imu_drotek2.gyro_valid = FALSE;
  imu_drotek2.accel_valid = FALSE;
  imu_drotek2.mag_valid = FALSE;
}

void imu_periodic(void)
{
  // Start reading the latest gyroscope data
  mpu60x0_i2c_periodic(&imu_drotek2.mpu);

  // Read HMC58XX at ~50Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(10, hmc58xx_periodic(&imu_drotek2.hmc));

}

void imu_drotek2_event(void)
{
  // If the MPU6050 I2C transaction has succeeded: convert the data
  mpu60x0_i2c_event(&imu_drotek2.mpu);
  if (imu_drotek2.mpu.data_available) {
    memcpy(&imu.gyro_unscaled, &imu_drotek2.mpu.data_rates.rates, sizeof(struct Int32Rates));
    memcpy(&imu.accel_unscaled, &imu_drotek2.mpu.data_accel.vect, sizeof(struct Int32Vect3));
    imu_drotek2.mpu.data_available = FALSE;
    imu_drotek2.gyro_valid = TRUE;
    imu_drotek2.accel_valid = TRUE;
  }

  /* HMC58XX event task */
  hmc58xx_event(&imu_drotek2.hmc);
  if (imu_drotek2.hmc.data_available) {
    VECT3_COPY(imu.mag_unscaled, imu_drotek2.hmc.data.vect);
    imu_drotek2.hmc.data_available = FALSE;
    imu_drotek2.mag_valid = TRUE;
  }
}

/** callback function to configure hmc5883 mag
 * @return TRUE if mag configuration finished
 */
bool_t imu_drotek2_configure_mag_slave(Mpu60x0ConfigSet mpu_set, void* mpu)
{
  hmc58xx_start_configure(&imu_drotek2.hmc);
  if (imu_drotek2.hmc.initialized)
    return TRUE;
  else
    return FALSE;
}
