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

/** @file stabilization_attitude_ref_int.h
 *  Rotorcraft attitude reference generation API.
 *  Common to all fixed-point reference generators (euler and quaternion)
 */

#ifndef STABILIZATION_ATTITUDE_REF_INT_H
#define STABILIZATION_ATTITUDE_REF_INT_H

#include "math/pprz_algebra_int.h"

#include "state.h"

extern struct Int32Eulers stab_att_sp_euler;  ///< with #INT32_ANGLE_FRAC
extern struct Int32Quat   stab_att_sp_quat;   ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_ref_euler; ///< with #REF_ANGLE_FRAC
extern struct Int32Quat   stab_att_ref_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Rates  stab_att_ref_rate;  ///< with #REF_RATE_FRAC
extern struct Int32Rates  stab_att_ref_accel; ///< with #REF_ACCEL_FRAC

struct Int32RefModel {
  struct Int32Rates omega;
  struct Int32Rates zeta;
};

extern struct Int32RefModel stab_att_ref_model;

#define REF_ACCEL_FRAC 12
#define REF_RATE_FRAC  16
#define REF_ANGLE_FRAC 20

#define REF_ANGLE_PI      BFP_OF_REAL(3.1415926535897932384626433832795029, REF_ANGLE_FRAC)
#define REF_ANGLE_TWO_PI  BFP_OF_REAL(2.*3.1415926535897932384626433832795029, REF_ANGLE_FRAC)
#define ANGLE_REF_NORMALIZE(_a) {                       \
    while (_a >  REF_ANGLE_PI)  _a -= REF_ANGLE_TWO_PI; \
    while (_a < -REF_ANGLE_PI)  _a += REF_ANGLE_TWO_PI; \
  }


static inline void reset_psi_ref_from_body(void) {
//sp has been set from body using stabilization_attitude_get_yaw_i, use that value
  stab_att_ref_euler.psi = stab_att_sp_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
  stab_att_ref_rate.r = 0;
  stab_att_ref_accel.r = 0;
}

#endif /* STABILIZATION_ATTITUDE_REF_INT_H */
