/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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

#ifndef AHRS_EXTERN_EULER_H
#define AHRS_EXTERN_EULER_H

#include "state.h"
#include "subsystems/ahrs.h"
#include "subsystems/imu.h"

struct AhrsIntExternEuler {
  struct Int32Eulers ltp_to_imu_euler; ///< Rotation from LocalTangentPlane to IMU frame as Euler angles
  struct Int32Quat   ltp_to_imu_quat;  ///< Rotation from LocalTangentPlane to IMU frame as quaternions
  struct Int32Rates  imu_rate;         ///< Rotational velocity in IMU frame
  float mag_offset;
};

extern struct AhrsIntExternEuler ahrs_impl;

#endif /* AHRS_INT_CMPL_EULER_H */
