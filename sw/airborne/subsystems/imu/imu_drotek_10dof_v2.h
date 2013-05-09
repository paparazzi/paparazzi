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
 * @file subsystems/imu/imu_drotek_10dof_v2.h
 *
 * Driver for the Drotek 10DOF V2 IMU.
 * MPU6050 + HMC5883 + MS5611
 */

#ifndef IMU_DROTEK_10DOF_V2_H
#define IMU_DROTEK_10DOF_V2_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "peripherals/mpu60x0_i2c.h"
#include "subsystems/imu/imu_mpu60x0_defaults.h"
#include "peripherals/hmc58xx.h"


#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN  1
#define IMU_MAG_Y_SIGN  1
#define IMU_MAG_Z_SIGN  1
#endif
#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN   1
#define IMU_GYRO_R_SIGN   1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN  1
#define IMU_ACCEL_Z_SIGN  1
#endif


struct ImuDrotek2 {
  volatile bool_t gyro_valid;
  volatile bool_t accel_valid;
  volatile bool_t mag_valid;
  struct Mpu60x0_I2c mpu;
  struct Hmc58xx hmc;
};

extern struct ImuDrotek2 imu_drotek2;

extern void imu_drotek2_event(void);
extern bool_t imu_drotek2_configure_mag_slave(Mpu60x0ConfigSet mpu_set, void* mpu);


static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void)) {
  imu_drotek2_event();
  if (imu_drotek2.gyro_valid) {
    imu_drotek2.gyro_valid = FALSE;
    _gyro_handler();
  }
  if (imu_drotek2.accel_valid) {
    imu_drotek2.accel_valid = FALSE;
    _accel_handler();
  }
  if (imu_drotek2.mag_valid) {
    imu_drotek2.mag_valid = FALSE;
    _mag_handler();
  }
}

#endif /* IMU_DROTEK_10DOF_V2_H */
