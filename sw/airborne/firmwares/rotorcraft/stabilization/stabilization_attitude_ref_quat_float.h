/*
 * $Id: booz_stabilization_attitude_ref_traj_euler.h 3796 2009-07-25 00:01:02Z poine $
 *  
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
#ifndef BOOZ_STABILIZATION_ATTITUDE_REF_QUAT_FLOAT_H
#define BOOZ_STABILIZATION_ATTITUDE_REF_QUAT_FLOAT_H

#include "booz_stabilization.h"

#include "booz_radio_control.h"
#include "math/pprz_algebra_float.h"

#include "stabilization/booz_stabilization_attitude_ref_float.h"

#define RC_UPDATE_FREQ 40.
#define ROLL_COEF   (BOOZ_STABILIZATION_ATTITUDE_SP_MAX_P     / MAX_PPRZ)
#define ROLL_COEF_H  (BOOZ_STABILIZATION_ATTITUDE_SP_MAX_P_H     / MAX_PPRZ)
#define PITCH_COEF ( BOOZ_STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ)
#define YAW_COEF  (BOOZ_STABILIZATION_ATTITUDE_SP_MAX_PSI   / MAX_PPRZ)

#define ROLL_COEF_RATE  (-BOOZ_STABILIZATION_ATTITUDE_SP_MAX_P   / MAX_PPRZ)
#define PITCH_COEF_RATE ( BOOZ_STABILIZATION_ATTITUDE_SP_MAX_Q / MAX_PPRZ)
#define YAW_COEF_RATE ( BOOZ_STABILIZATION_ATTITUDE_SP_MAX_R / MAX_PPRZ)

#define DEADBAND_EXCEEDED(VARIABLE, VALUE) ((VARIABLE > VALUE) || (VARIABLE < -VALUE))
#define APPLY_DEADBAND(VARIABLE, VALUE) (DEADBAND_EXCEEDED(VARIABLE, VALUE) ? VARIABLE : 0.0)

#define ROLL_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_CONTROL_ROLL] >  BOOZ_STABILIZATION_ATTITUDE_DEADBAND_A || \
   radio_control.values[RADIO_CONTROL_ROLL] < -BOOZ_STABILIZATION_ATTITUDE_DEADBAND_A)
#define PITCH_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_CONTROL_PITCH] >  BOOZ_STABILIZATION_ATTITUDE_DEADBAND_E || \
   radio_control.values[RADIO_CONTROL_PITCH] < -BOOZ_STABILIZATION_ATTITUDE_DEADBAND_E)
#define YAW_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_CONTROL_YAW] >  BOOZ_STABILIZATION_ATTITUDE_DEADBAND_R || \
   radio_control.values[RADIO_CONTROL_YAW] < -BOOZ_STABILIZATION_ATTITUDE_DEADBAND_R)

void booz_stabilization_attitude_ref_enter(void);
void booz_stabilization_attitude_ref_schedule(uint8_t idx);

#endif /* BOOZ_STABILIZATION_ATTITUDE_REF_QUAT_FLOAT_H */
