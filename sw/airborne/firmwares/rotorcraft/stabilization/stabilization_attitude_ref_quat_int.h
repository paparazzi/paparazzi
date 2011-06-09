/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
#ifndef STABILIZATION_ATTITUDE_INT_REF_QUAT_INT_H
#define STABILIZATION_ATTITUDE_INT_REF_QUAT_INT_H

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/quat_setpoint.h"

#include "subsystems/radio_control.h"
#include "math/pprz_algebra_float.h"

#include "stabilization_attitude_ref_int.h"

#define REF_ACCEL_FRAC 12
#define REF_RATE_FRAC  16
#define REF_ANGLE_FRAC 20

#define REF_ANGLE_PI      BFP_OF_REAL(3.1415926535897932384626433832795029, REF_ANGLE_FRAC)
#define REF_ANGLE_TWO_PI  BFP_OF_REAL(2.*3.1415926535897932384626433832795029, REF_ANGLE_FRAC)
#define ANGLE_REF_NORMALIZE(_a) {         \
      while (_a >  REF_ANGLE_PI)  _a -= REF_ANGLE_TWO_PI;     \
      while (_a < -REF_ANGLE_PI)  _a += REF_ANGLE_TWO_PI;     \
    }


#define RC_UPDATE_FREQ 40.
#define ROLL_COEF   (STABILIZATION_ATTITUDE_SP_MAX_P     / MAX_PPRZ)
#define ROLL_COEF_H  (STABILIZATION_ATTITUDE_SP_MAX_P_H     / MAX_PPRZ)
#define PITCH_COEF ( STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ)
#define YAW_COEF  (STABILIZATION_ATTITUDE_SP_MAX_PSI   / MAX_PPRZ)

#define ROLL_COEF_RATE  (-STABILIZATION_ATTITUDE_SP_MAX_P   / MAX_PPRZ)
#define PITCH_COEF_RATE ( STABILIZATION_ATTITUDE_SP_MAX_Q / MAX_PPRZ)
#define YAW_COEF_RATE ( STABILIZATION_ATTITUDE_SP_MAX_R / MAX_PPRZ)

#define DEADBAND_EXCEEDED(VARIABLE, VALUE) ((VARIABLE > VALUE) || (VARIABLE < -VALUE))
#define APPLY_DEADBAND(VARIABLE, VALUE) (DEADBAND_EXCEEDED(VARIABLE, VALUE) ? VARIABLE : 0.0)

#define ROLL_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_ROLL] >  STABILIZATION_ATTITUDE_DEADBAND_A || \
   radio_control.values[RADIO_ROLL] < -STABILIZATION_ATTITUDE_DEADBAND_A)
#define PITCH_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_PITCH] >  STABILIZATION_ATTITUDE_DEADBAND_E || \
   radio_control.values[RADIO_PITCH] < -STABILIZATION_ATTITUDE_DEADBAND_E)
#define YAW_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_YAW] >  STABILIZATION_ATTITUDE_DEADBAND_R || \
   radio_control.values[RADIO_YAW] < -STABILIZATION_ATTITUDE_DEADBAND_R)

#define STABILIZATION_ATTITUDE_READ_RC(_sp, _in_flight) do { stabilization_attitude_read_rc_absolute(_sp, _in_flight); } while(0)
#define STABILIZATION_ATTITUDE_RESET_PSI_REF(_sp) do {} while(0)

void stabilization_attitude_ref_enter(void);
void stabilization_attitude_ref_schedule(uint8_t idx);

#endif /* STABILIZATION_ATTITUDE_INT_REF_QUAT_INT_H */
