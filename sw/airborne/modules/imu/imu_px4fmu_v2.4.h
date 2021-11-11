/*
 * Copyright (C) 2013-2016 the paparazzi team
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
 * @file modules/imu/imu_px4fmu_v2.4.h
 * Driver for pixhawk IMU's.
 * L3GD20H + LSM303D (both on spi)
 */

#ifndef IMU_PX4FMUV24_H
#define IMU_PX4FMUV24_H

#include "std.h"
#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "modules/imu/imu_px4_defaults.h"
#include "peripherals/l3gd20_spi.h"
#include "peripherals/lsm303d_spi.h"

#ifndef IMU_PX4_DISABLE_MAG
#if MODULE_HMC58XX_UPDATE_AHRS
#define IMU_PX4_DISABLE_MAG TRUE
#else
#define IMU_PX4_DISABLE_MAG FALSE
#endif
#endif

struct ImuPX4 {
  struct L3gd20_Spi l3g;
  struct Lsm303d_Spi lsm_acc;
  struct Lsm303d_Spi lsm_mag;
};

extern struct ImuPX4 imu_px4;

extern void imu_px4_init(void);
extern void imu_px4_periodic(void);
extern void imu_px4_event(void);

#endif /* IMU_PX4FMUV24_H */
