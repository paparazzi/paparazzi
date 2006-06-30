/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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
 *
 */

#include "std.h"
#include "airframe.h"
#include "control_grz.h"
#include "link_imu.h"
#include "paparazzi.h"
#include "radio_control.h"
#include "commands.h"

#define NORMALIZE(n, lower, uper) {		\
    while ( n > uper)  { n -= (uper - lower);}	\
    while ( n < lower) { n += (uper - lower);}	\
}

float   ctl_grz_roll_dot_pgain = -1.9;
float   ctl_grz_roll_dot_igain = 0.;
float   ctl_grz_roll_dot_dgain = 0.;

float   ctl_grz_pitch_dot_pgain = -1.9;
float   ctl_grz_pitch_dot_igain = 0.;
float   ctl_grz_pitch_dot_dgain = 0.;

float   ctl_grz_yaw_dot_pgain = -4.8;
float   ctl_grz_yaw_dot_igain = 0.;
float   ctl_grz_yaw_dot_dgain = 0.;

#define MAX_ROLL_DOT_SP (3./MAX_PPRZ)
static float roll_dot = 0;
static float roll_dot_setpoint = 0.;
//static float roll_dot_sum_err = 0;
static float roll_dot_last_err = 0;

#define MAX_PITCH_DOT_SP (3./MAX_PPRZ)
static float pitch_dot = 0;
static float pitch_dot_setpoint = 0.;
//static float pitch_dot_sum_err = 0;
static float pitch_dot_last_err = 0;

#define MAX_YAW_DOT_SP (1./MAX_PPRZ)
static float yaw_dot = 0;
static float yaw_dot_setpoint = 0.;
//static float yaw_dot_sum_err = 0;
static float yaw_dot_last_err = 0;

static float throttle_setpoint = 0.;

void ctl_grz_set_setpoints_rate( void ) {
  roll_dot_setpoint = rc_values[RADIO_ROLL] * MAX_ROLL_DOT_SP;
  pitch_dot_setpoint = rc_values[RADIO_PITCH] * MAX_PITCH_DOT_SP;
  yaw_dot_setpoint = rc_values[RADIO_YAW] * MAX_YAW_DOT_SP;
  throttle_setpoint = rc_values[RADIO_YAW];
}

void ctl_grz_set_measures( void ) {
  roll_dot = link_imu_state.rates[AXIS_X];
  pitch_dot = link_imu_state.rates[AXIS_Y];
  yaw_dot = link_imu_state.rates[AXIS_Z];
}

void ctl_grz_rate_run ( void ) {
  float err =  roll_dot - roll_dot_setpoint;
  int16_t d_err = err - roll_dot_last_err;
  roll_dot_last_err = err;
  //  roll_dot_sum_err += err;
  //  control_commands[RADIO_ROLL] = TRIM_PPRZ((err + d_err * roll_dot_dgain + roll_dot_sum_err * roll_dot_igain)
  commands[COMMAND_ROLL] = TRIM_PPRZ((int16_t)((err + d_err * ctl_grz_roll_dot_dgain) * ctl_grz_roll_dot_pgain));

  err = pitch_dot - pitch_dot_setpoint;
  d_err = err - pitch_dot_last_err;
  pitch_dot_last_err = err;
  //  pitch_dot_sum_err += err;
  //  control_commands[RADIO_PITCH] = TRIM_PPRZ((err + d_err * pitch_dot_dgain + pitch_dot_sum_err * pitch_dot_igain) * pitch_dot_pgain);
  commands[COMMAND_PITCH] =  TRIM_PPRZ((int16_t)((err + d_err * ctl_grz_pitch_dot_dgain) * ctl_grz_pitch_dot_pgain));

  err = yaw_dot - yaw_dot_setpoint;;
  d_err = err - yaw_dot_last_err;
  yaw_dot_last_err = err;
  //  d_err = err - yaw_dot_last_err;
  //  yaw_dot_last_err = err;
  //  yaw_dot_sum_err += err;
  //  control_commands[RADIO_YAW] = TRIM_PPRZ((err + d_err * yaw_dot_dgain + yaw_dot_sum_err * yaw_dot_igain) * yaw_dot_pgain);
  commands[COMMAND_YAW] =   TRIM_PPRZ((int16_t)((err + d_err * ctl_grz_yaw_dot_dgain) * ctl_grz_yaw_dot_pgain));

  commands[COMMAND_THROTTLE] = throttle_setpoint;

}


