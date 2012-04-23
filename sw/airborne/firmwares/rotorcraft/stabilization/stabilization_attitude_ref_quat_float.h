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
#ifndef STABILIZATION_ATTITUDE_FLOAT_REF_QUAT_FLOAT_H
#define STABILIZATION_ATTITUDE_FLOAT_REF_QUAT_FLOAT_H

#include "firmwares/rotorcraft/stabilization.h"

#include "subsystems/radio_control.h"
#include "math/pprz_algebra_float.h"

#include "stabilization_attitude_ref_float.h"

#define RC_UPDATE_FREQ 40.
#define ROLL_COEF   (STABILIZATION_ATTITUDE_FLOAT_SP_MAX_PHI   / MAX_PPRZ)
// FIXME: unused, what was it supposed to be?
//#define ROLL_COEF_H (STABILIZATION_ATTITUDE_FLOAT_SP_MAX_P_H   / MAX_PPRZ)
#define PITCH_COEF  (STABILIZATION_ATTITUDE_FLOAT_SP_MAX_THETA / MAX_PPRZ)
// FIXME: what is this supposed to be??
#define YAW_COEF    (STABILIZATION_ATTITUDE_FLOAT_SP_MAX_PSI   / MAX_PPRZ)

#define DEADBAND_EXCEEDED(VARIABLE, VALUE) ((VARIABLE > VALUE) || (VARIABLE < -VALUE))
#define APPLY_DEADBAND(VARIABLE, VALUE) (DEADBAND_EXCEEDED(VARIABLE, VALUE) ? VARIABLE : 0.0)

#define ROLL_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_ROLL] >  STABILIZATION_ATTITUDE_FLOAT_DEADBAND_A || \
   radio_control.values[RADIO_ROLL] < -STABILIZATION_ATTITUDE_FLOAT_DEADBAND_A)
#define PITCH_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_PITCH] >  STABILIZATION_ATTITUDE_FLOAT_DEADBAND_E || \
   radio_control.values[RADIO_PITCH] < -STABILIZATION_ATTITUDE_FLOAT_DEADBAND_E)
#define YAW_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_YAW] >  STABILIZATION_ATTITUDE_FLOAT_DEADBAND_R || \
   radio_control.values[RADIO_YAW] < -STABILIZATION_ATTITUDE_FLOAT_DEADBAND_R)

void stabilization_attitude_ref_enter(void);
void stabilization_attitude_ref_schedule(uint8_t idx);

#endif /* STABILIZATION_ATTITUDE_FLOAT_REF_QUAT_FLOAT_H */
