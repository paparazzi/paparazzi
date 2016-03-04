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
 * @file subsystems/imu/imu_mpu6000.h
 * Driver for pixhawk IMU's.
 * On with spi: L3GD20H + LSM303D and the MPU6000.
 * On i2c: external HMC5883L (through 3dr gps).
 */

#ifndef IMU_PX4FMUV24_H
#define IMU_PX4FMUV24_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "subsystems/imu/imu_mpu60x0_defaults.h"
#include "peripherals/mpu60x0_spi.h"
#include "peripherals/hmc58xx.h"
#include "peripherals/l3gd20_spi.h"
#include "peripherals/lsm303dlhc_spi.h"


struct ImuPX4 {
  struct Mpu60x0_Spi mpu;
  struct Hmc58xx hmc;
  struct L3gd20_Spi l3g;
  struct Lsm303dlhc_Spi lsm_acc;
  struct Lsm303dlhc_Spi lsm_mag;
};

extern struct ImuPX4 imu_px4;

extern void imu_px4_event(void);

#define ImuEvent imu_px4_event

#endif /* IMU_PX4FMUV24_H */
