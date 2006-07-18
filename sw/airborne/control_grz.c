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
#include "estimator.h"
#include "fbw_downlink.h"

#define NORMALIZE(n, lower, uper) {		\
    while ( n > uper)  { n -= (uper - lower);}	\
    while ( n < lower) { n += (uper - lower);}	\
}


float trim_roll = -450;
float trim_yaw = -250;
float trim_pitch = 0;

float max_roll_dot_sp  = (1.5/MAX_PPRZ);
float max_pitch_dot_sp = (1.5/MAX_PPRZ);
float max_yaw_dot_sp   = (1.5/MAX_PPRZ);

float max_roll_sp  = (0.4/MAX_PPRZ);
float max_pitch_sp = (0.4/MAX_PPRZ);
/* yaw is incremented acording to stick position */
#define UPDATE_SP_DT 0.016384
float max_yaw_sp   = (0.8/MAX_PPRZ*UPDATE_SP_DT);

float max_v_sp   = (6./MAX_PPRZ);

static float throttle_setpoint = 0.;

float max_z_dot_sp =  (5./MAX_PPRZ);
float ctl_grz_z_dot_pgain = -2000;
float ctl_grz_z_dot_igain = 3.;
float ctl_grz_z_dot_dgain = 8.;
float ctl_grz_z_dot = 0;
float ctl_grz_z_dot_setpoint = 0.;
float ctl_grz_z_dot_sum_err = 0.;
float ctl_grz_throttle_level = 6700.;


float max_z_sp =  (4./MAX_PPRZ);
float ctl_grz_z_pgain = -1.2;
float ctl_grz_z_igain = 0.;
float ctl_grz_z_dgain = 5.;
float ctl_grz_z = 0;
float ctl_grz_z_setpoint = 0.;
float ctl_grz_z_sum_err = 0.;


bool_t flying = FALSE;

#define MIN_Z_DOT -1.
#define MAX_Z_DOT 1.

struct pid roll_pid;
struct pid pitch_pid;
struct pid yaw_pid;

struct pid roll_dot_pid;
struct pid pitch_dot_pid;
struct pid yaw_dot_pid;

struct pid vn_pid;
struct pid ve_pid;

void ctl_grz_init( void ) {

  roll_dot_pid.p_gain = 4700.;
  roll_dot_pid.i_gain = 1.;
  roll_dot_pid.d_gain = 1.875;

  pitch_dot_pid.p_gain = -4700.;
  pitch_dot_pid.i_gain = 1.;
  pitch_dot_pid.d_gain = 1.875;

  yaw_dot_pid.p_gain = -4700.;
  yaw_dot_pid.i_gain = 1.;
  yaw_dot_pid.d_gain = 2.;

  roll_pid.p_gain = -5;
  roll_pid.i_gain = 0.;
  roll_pid.d_gain = 1.;
  roll_pid.saturation = 3.;

  pitch_pid.p_gain = -5;
  pitch_pid.i_gain = 0.;
  pitch_pid.d_gain = 1.;
  pitch_pid.saturation = 3.;

  yaw_pid.p_gain = -1.;
  yaw_pid.i_gain = 0.;
  yaw_pid.d_gain = 0.;
  yaw_pid.saturation = 2.;
  yaw_pid.set_point = 0.; // RadOfDeg(-20);

  vn_pid.p_gain = -0.1;
  vn_pid.i_gain = 0.;
  vn_pid.d_gain = 0.;
  vn_pid.saturation = .2;

  ve_pid.p_gain = -0.2;
  ve_pid.i_gain = 1.;
  ve_pid.d_gain = 1.;
  ve_pid.saturation = .2;
}

/** FIXME: should be done by SetCommandsFromRC(commands); */
void ctl_grz_set_setpoints_rate( void ) {
  roll_dot_pid.set_point = - commands[COMMAND_ROLL_DOT] * max_roll_dot_sp;
  pitch_dot_pid.set_point = commands[COMMAND_PITCH_DOT] * max_pitch_dot_sp;
  yaw_dot_pid.set_point = - commands[COMMAND_YAW_DOT] * max_yaw_dot_sp;
  throttle_setpoint = commands[COMMAND_Z];
}


#define YAW_GAP ((int16_t)(0.05*MAX_PPRZ))
void ctl_grz_set_setpoints_auto1( void ) {
  roll_pid.set_point = - rc_values[RADIO_ROLL] * max_roll_sp;
  pitch_pid.set_point = rc_values[RADIO_PITCH] * max_pitch_sp;
  #if 0
  //  yaw_dot_pid.set_point = - rc_values[RADIO_YAW] * max_yaw_dot_sp;
  /* yaw stick increments yaw setpoint */
  if (rc_values[RADIO_YAW] > YAW_GAP || rc_values[RADIO_YAW] < -YAW_GAP) {
    yaw_pid.set_point += - rc_values[RADIO_YAW] * max_yaw_dot_sp;
    NORMALIZE(yaw_pid.set_point, -M_PI, M_PI);  
  }
  #endif
  //  ctl_grz_z_dot_setpoint = (rc_values[RADIO_THROTTLE] - MAX_PPRZ/2) * max_z_dot_sp;
  throttle_setpoint = rc_values[RADIO_THROTTLE];
  //  ctl_grz_z_setpoint = rc_values[RADIO_THROTTLE] * max_z_sp - 0.5;
}

void ctl_grz_set_setpoints_auto2( void ) {
  roll_pid.set_point = - rc_values[RADIO_ROLL] * max_roll_sp;
  pitch_pid.set_point = rc_values[RADIO_PITCH] * max_pitch_sp;

/*   float vright = - rc_values[RADIO_ROLL] * max_v_sp; */
/*   float vfront = - rc_values[RADIO_PITCH] * max_v_sp; */
  
/*   float alpha = M_PI/2. - yaw_pid.measure; */
/*   float cos_alpha = cos(alpha); */
/*   float sin_alpha = sin(alpha); */
/*   ve_pid.set_point = sin_alpha*vright + cos_alpha*vfront; */
/*   vn_pid.set_point = - cos_alpha*vright + sin_alpha*vfront; */
 
  ctl_grz_z_dot_setpoint = (rc_values[RADIO_THROTTLE] - MAX_PPRZ/2) * max_z_dot_sp; 
  //  ctl_grz_z_setpoint = rc_values[RADIO_THROTTLE] * max_z_sp - 0.5;
  // throttle_setpoint = rc_values[RADIO_THROTTLE];
}

void ctl_grz_set_measures( void ) {
  roll_dot_pid.measure = link_imu_state.rates[AXIS_X];
  pitch_dot_pid.measure = link_imu_state.rates[AXIS_Y];
  yaw_dot_pid.measure = link_imu_state.rates[AXIS_Z];

  roll_pid.measure = link_imu_state.eulers[AXIS_X];
  pitch_pid.measure = link_imu_state.eulers[AXIS_Y];
  yaw_pid.measure = link_imu_state.eulers[AXIS_Z];

  ve_pid.measure = estimator_hspeed_mod*sin(estimator_hspeed_dir);
  vn_pid.measure = estimator_hspeed_mod*cos(estimator_hspeed_dir);
}


void ctl_grz_rate_run ( void ) {
  float err = roll_dot_pid.measure - roll_dot_pid.set_point;
  float d_err = err - roll_dot_pid.last_err;
  roll_dot_pid.last_err = err;
  if (flying) {
    roll_dot_pid.sum_err += err/100;
    Bound(roll_dot_pid.sum_err, -0.2, 0.2);
  } else
    roll_dot_pid.sum_err = 0.;

  commands[COMMAND_ROLL] = TRIM_PPRZ((int16_t)((err + d_err * roll_dot_pid.d_gain + roll_dot_pid.sum_err * roll_dot_pid.i_gain) * roll_dot_pid.p_gain));

  err = pitch_dot_pid.measure - pitch_dot_pid.set_point;
  d_err = err - pitch_dot_pid.last_err;
  pitch_dot_pid.last_err = err;
  if (flying) {
    pitch_dot_pid.sum_err += err/100;
    Bound(pitch_dot_pid.sum_err, -0.2, 0.2);
  } else
    pitch_dot_pid.sum_err = 0;

  commands[COMMAND_PITCH] = TRIM_PPRZ((int16_t)((err + d_err * pitch_dot_pid.d_gain + pitch_dot_pid.sum_err * pitch_dot_pid.i_gain) * pitch_dot_pid.p_gain));
  
  err = yaw_dot_pid.measure - yaw_dot_pid.set_point;
  d_err = err - yaw_dot_pid.last_err;
  yaw_dot_pid.last_err = err;
  if (flying) {
    yaw_dot_pid.sum_err += err/100;
    Bound(yaw_dot_pid.sum_err, -0.2, 0.2);
  } else
    yaw_dot_pid.sum_err = 0;
  commands[COMMAND_YAW] = TRIM_PPRZ((int16_t)((err + d_err * yaw_dot_pid.d_gain  + yaw_dot_pid.sum_err * yaw_dot_pid.i_gain) * yaw_dot_pid.p_gain));

  commands[COMMAND_THROTTLE] = throttle_setpoint;
}



void ctl_grz_attitude_run ( void ) {
  float err = roll_pid.measure - roll_pid.set_point;
  float d_err = err - roll_pid.last_err;
  roll_pid.last_err = err;
  roll_dot_pid.set_point = (err + d_err * roll_pid.d_gain) * roll_pid.p_gain; 
  //  Bound(roll_dot_pid.set_point, roll_pid.saturation, -roll_pid.saturation);

  err = pitch_pid.measure - pitch_pid.set_point;
  d_err = err - pitch_pid.last_err;
  pitch_pid.last_err = err;
  pitch_dot_pid.set_point = (err + d_err * pitch_pid.d_gain) * pitch_pid.p_gain; 
  //  Bound(pitch_dot_pid.set_point, pitch_pid.saturation, -pitch_pid.saturation);
  
  err = yaw_pid.measure - yaw_pid.set_point;
  NORMALIZE(err, -M_PI, M_PI);
  d_err = err - yaw_pid.last_err;
  yaw_pid.last_err = err;
  yaw_dot_pid.set_point = (err + d_err * yaw_pid.d_gain) * yaw_pid.p_gain; 
  //  Bound(yaw_dot_pid.set_point, yaw_pid.saturation, -yaw_pid.saturation);
  
}


void ctl_grz_alt_run( void ) {
  float err =  ctl_grz_z - ctl_grz_z_setpoint;
  static float z_last_err;
  float d_err = err - z_last_err;
  z_last_err = err;
  //  ctl_grz_z_sum_err += err;

  if (ctl_grz_z_setpoint < 0.1)
    ctl_grz_z_dot_setpoint = -10;
  else
    ctl_grz_z_dot_setpoint = Chop(((err + d_err * ctl_grz_z_dgain + ctl_grz_z_sum_err * ctl_grz_z_igain) * ctl_grz_z_pgain), MIN_Z_DOT, MAX_Z_DOT);
}

void ctl_grz_z_dot_run( void ) {
  float err =  ctl_grz_z_dot - ctl_grz_z_dot_setpoint;
  static float z_dot_last_err;
  float d_err = err - z_dot_last_err;
  z_dot_last_err = err;
  
  if (flying) {
    ctl_grz_z_dot_sum_err += err/600;
    Bound(ctl_grz_z_dot_sum_err, -1., 1.);
  } else
    ctl_grz_z_dot_sum_err = 0;

  throttle_setpoint = TRIM_PPRZ(ctl_grz_throttle_level + (int16_t)((err + d_err * ctl_grz_z_dot_dgain + ctl_grz_z_dot_sum_err * ctl_grz_z_dot_igain) * ctl_grz_z_dot_pgain));
}


float north_angle_set_point;
float east_angle_set_point;


void ctl_grz_horiz_speed_run ( void ) {
  float err = ve_pid.measure - ve_pid.set_point;
  float d_err = err - ve_pid.last_err;
  ve_pid.last_err = err;
  ve_pid.sum_err += err/100;
  east_angle_set_point = (err + d_err * ve_pid.d_gain + ve_pid.sum_err * ve_pid.i_gain) * ve_pid.p_gain; 
  Bound(east_angle_set_point, -ve_pid.saturation, ve_pid.saturation);
  Bound(ve_pid.sum_err, -3, 3);

  err = vn_pid.measure - vn_pid.set_point;
  d_err = err - vn_pid.last_err;
  vn_pid.last_err = err;
  vn_pid.sum_err += err/100;
  north_angle_set_point = - ((err + d_err * ve_pid.d_gain + vn_pid.sum_err * ve_pid.i_gain) * ve_pid.p_gain);
  Bound(north_angle_set_point, -vn_pid.saturation, vn_pid.saturation);
  Bound(vn_pid.sum_err, -3, 3);

  float alpha = (M_PI/2. - yaw_pid.measure);
  float cos_alpha = cos(alpha);
  float sin_alpha = sin(alpha);

  roll_pid.set_point = sin_alpha*east_angle_set_point + cos_alpha*north_angle_set_point;
  pitch_pid.set_point = - cos_alpha*east_angle_set_point + sin_alpha*north_angle_set_point;
}

void ctl_grz_reset( void ) {
  ctl_grz_z_dot_sum_err = 0;
  ve_pid.sum_err = 0;
  vn_pid.sum_err = 0;
  roll_dot_pid.sum_err = 0;
  pitch_dot_pid.sum_err = 0;
  yaw_dot_pid.sum_err = 0;
}

