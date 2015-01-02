/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 * @file boards/apogee/imu_apogee.c
 *
 * Driver for the IMU on the Apogee board.
 *
 * Invensense MPU-6050
 */

#include <math.h>
#include "boards/apogee/imu_apogee.h"
#include "mcu_periph/i2c.h"
#include "led.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"


#if !defined APOGEE_LOWPASS_FILTER && !defined  APOGEE_SMPLRT_DIV
#define APOGEE_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define APOGEE_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz")
#endif
PRINT_CONFIG_VAR(APOGEE_SMPLRT_DIV)
PRINT_CONFIG_VAR(APOGEE_LOWPASS_FILTER)

#ifndef APOGEE_GYRO_RANGE
#define APOGEE_GYRO_RANGE MPU60X0_GYRO_RANGE_1000
#endif
PRINT_CONFIG_VAR(APOGEE_GYRO_RANGE)

#ifndef APOGEE_ACCEL_RANGE
#define APOGEE_ACCEL_RANGE MPU60X0_ACCEL_RANGE_8G
#endif
PRINT_CONFIG_VAR(APOGEE_ACCEL_RANGE)

struct ImuApogee imu_apogee;

// baro config will be done later in bypass mode
bool_t configure_baro_slave(Mpu60x0ConfigSet mpu_set, void *mpu);

bool_t configure_baro_slave(Mpu60x0ConfigSet mpu_set __attribute__((unused)), void *mpu __attribute__((unused)))
{
  return TRUE;
}

void imu_impl_init(void)
{
  /////////////////////////////////////////////////////////////////////
  // MPU-60X0
  mpu60x0_i2c_init(&imu_apogee.mpu, &(IMU_APOGEE_I2C_DEV), MPU60X0_ADDR_ALT);
  // change the default configuration
  imu_apogee.mpu.config.smplrt_div = APOGEE_SMPLRT_DIV;
  imu_apogee.mpu.config.dlpf_cfg = APOGEE_LOWPASS_FILTER;
  imu_apogee.mpu.config.gyro_range = APOGEE_GYRO_RANGE;
  imu_apogee.mpu.config.accel_range = APOGEE_ACCEL_RANGE;
  // set MPU in bypass mode for the baro
  imu_apogee.mpu.config.nb_slaves = 1;
  imu_apogee.mpu.config.slaves[0].configure = &configure_baro_slave;
  imu_apogee.mpu.config.i2c_bypass = TRUE;

  imu_apogee.gyr_valid = FALSE;
  imu_apogee.acc_valid = FALSE;
}

void imu_periodic(void)
{
  // Start reading the latest gyroscope data
  mpu60x0_i2c_periodic(&imu_apogee.mpu);

  //RunOnceEvery(10,imu_apogee_downlink_raw());
}

void imu_apogee_downlink_raw(void)
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice, &imu.gyro_unscaled.p, &imu.gyro_unscaled.q,
                             &imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice, &imu.accel_unscaled.x, &imu.accel_unscaled.y,
                              &imu.accel_unscaled.z);
}


void imu_apogee_event(void)
{
  // If the itg3200 I2C transaction has succeeded: convert the data
  mpu60x0_i2c_event(&imu_apogee.mpu);
  if (imu_apogee.mpu.data_available) {
    RATES_ASSIGN(imu.gyro_unscaled, imu_apogee.mpu.data_rates.rates.p, -imu_apogee.mpu.data_rates.rates.q,
                 -imu_apogee.mpu.data_rates.rates.r);
    VECT3_ASSIGN(imu.accel_unscaled, imu_apogee.mpu.data_accel.vect.x, -imu_apogee.mpu.data_accel.vect.y,
                 -imu_apogee.mpu.data_accel.vect.z);
    imu_apogee.mpu.data_available = FALSE;
    imu_apogee.gyr_valid = TRUE;
    imu_apogee.acc_valid = TRUE;
  }
}

