/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/imu/imu_aspirin.h
 * Interface for the Aspirin v1.x IMU using I2C for the accelerometer.
 */


#ifndef IMU_ASPIRIN_I2C_H
#define IMU_ASPIRIN_I2C_H

#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "peripherals/itg3200.h"
#include "peripherals/hmc58xx.h"
#include "peripherals/adxl345_i2c.h"

/* include default aspirin sensitivity/channel definitions */
#include "subsystems/imu/imu_aspirin_defaults.h"


struct ImuAspirinI2c {
  volatile uint8_t accel_valid;
  volatile uint8_t gyro_valid;
  volatile uint8_t mag_valid;
  struct Adxl345_I2c acc_adxl;
  struct Itg3200 gyro_itg;
  struct Hmc58xx mag_hmc;
};

extern struct ImuAspirinI2c imu_aspirin;

extern void imu_aspirin_i2c_event(void);

static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void))
{
  imu_aspirin_i2c_event();
  if (imu_aspirin.gyro_valid) {
    imu_aspirin.gyro_valid = FALSE;
    _gyro_handler();
  }
  if (imu_aspirin.accel_valid) {
    imu_aspirin.accel_valid = FALSE;
    _accel_handler();
  }
  if (imu_aspirin.mag_valid) {
    imu_aspirin.mag_valid = FALSE;
    _mag_handler();
  }
}

#endif /* IMU_ASPIRIN_I2C_H */
