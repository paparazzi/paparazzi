/*
 * Copyright (C) 2013 Michal Podhradsky
 * Utah State University, http://aggieair.usu.edu/
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
 * @file ahrs_aggienav.h
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef AHRS_AGGIENAV_H
#define AHRS_AGGIENAV_H

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_aligner.h"

#ifdef USE_GX3
#include "subsystems/imu/imu_gx3.h"
#endif

#include "state.h"
#include "led.h"

//AHRS
struct AhrsFloatQuat {
  struct FloatQuat   ltp_to_imu_quat;  ///< Rotation from LocalTangentPlane to IMU frame as quaternions
  float mag_offset;                    ///< Difference between true and magnetic north
};

extern struct AhrsFloatQuat ahrs_impl;

#ifdef AHRS_UPDATE_FW_ESTIMATOR
extern float ins_roll_neutral;
extern float ins_pitch_neutral;
#endif

#endif /* AHRS_AGGIENAV_H*/
