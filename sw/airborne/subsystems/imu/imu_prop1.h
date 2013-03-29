/*
 * Copyright (C) 2013 Eduardo Lavratti <agressiva@hotmail.com>
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
 * @file subsystems/imu/imu_prop1.h
 * Driver for the i2c IMU using l3g4200 + adxl345 + hmc5883.
 */


#ifndef IMU_PROP1_H
#define IMU_PROP1_H

#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "peripherals/l3g4200.h"
#include "peripherals/hmc58xx.h"
#include "peripherals/adxl345_i2c.h"

/* include default prop1 sensitivity/channel definitions */
#include "subsystems/imu/imu_prop1_defaults.h"


struct ImuProp1 {
  volatile uint8_t accel_valid;
  volatile uint8_t gyro_valid;
  volatile uint8_t mag_valid;
  struct Adxl345_I2c acc_adxl;
  struct L3g4200 gyro_l3g;
  struct Hmc58xx mag_hmc;
};

extern struct ImuProp1 imu_prop1;
extern void imu_prop1_event(void);

static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void)) {
  imu_prop1_event();
  if (imu_prop1.gyro_valid) {
    imu_prop1.gyro_valid = FALSE;
    _gyro_handler();
  }
  if (imu_prop1.accel_valid) {
    imu_prop1.accel_valid = FALSE;
    _accel_handler();
  }
  if (imu_prop1.mag_valid) {
    imu_prop1.mag_valid = FALSE;
    _mag_handler();
  }
}

#endif /* IMU_PROP1_H */
