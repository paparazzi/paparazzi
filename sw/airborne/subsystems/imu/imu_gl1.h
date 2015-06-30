/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
 *               2013 Eduardo Lavratti <agressiva@hotmail.com>
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
 * @file subsystems/imu/imu_gl1.h
 * Interface for I2c IMU using using L3G4200, ADXL345, HMC5883 and BMP085.
 */


#ifndef IMU_GL1_H
#define IMU_GL1_H

#include "generated/airframe.h"
#include "subsystems/imu.h"

/* include default GL1 sensitivity/channel definitions */
#include "subsystems/imu/imu_gl1_defaults.h"

#include "peripherals/l3g4200.h"
#include "peripherals/hmc58xx.h"
#include "peripherals/adxl345_i2c.h"

struct ImuGL1I2c {
  struct Adxl345_I2c acc_adxl;
  struct L3g4200 gyro_l3g;
  struct Hmc58xx mag_hmc;
};

extern struct ImuGL1I2c imu_gl1;

extern void imu_gl1_i2c_event(void);

#define ImuEvent imu_gl1_i2c_event

#endif /* IMU_GL1_H */
