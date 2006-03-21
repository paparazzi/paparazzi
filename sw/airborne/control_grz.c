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

#include "airframe.h"


#include "control_grz.h"
#include "estimator.h"
#include "radio.h"

int16_t setpoint_roll = 0;
float roll_pgain = -0.8;
#define MAX_ROLL_DOT (1. * 32768000. / 8500.)

int16_t setpoint_pitch = 0;
float pitch_pgain = -0.5;
#define MAX_PITCH_DOT (1. * 32768000. / 8500.)

int16_t setpoint_yaw = 0;
float yaw_pgain = -2.5;
#define MAX_YAW_DOT (1. * 32768000. / 8500.)

int16_t setpoint_roll_dot = 0;
float   roll_dot_pgain = -1.9;
float   roll_dot_igain = 0.0;
float   roll_dot_dgain = 0.0;
int16_t roll_dot_sum_err = 0;
int16_t roll_dot_last_err = 0;
pprz_t  command_roll = 0;

int16_t setpoint_pitch_dot = 0;
float   pitch_dot_pgain = -1.9;
float   pitch_dot_igain = 0.0;
float   pitch_dot_dgain = 0.0;
int16_t pitch_dot_sum_err = 0;
int16_t pitch_dot_last_err = 0;
pprz_t  command_pitch = 0;

int16_t setpoint_yaw_dot = 0;
float   yaw_dot_pgain = -4.8;
float   yaw_dot_igain = 0.0;
float   yaw_dot_dgain = 0.0;
int16_t yaw_dot_sum_err = 0;
int16_t yaw_dot_last_err = 0;
pprz_t  command_yaw = 0;

int16_t setpoint_throttle = 0;
pprz_t  command_throttle = 0;

pprz_t control_commands[] = {0, 0, 0, 0};

void control_rate_run ( void ) {
  int16_t err =  e_roll_dot - setpoint_roll_dot;
  int16_t d_err = err - roll_dot_last_err;
  roll_dot_last_err = err;
  //  roll_dot_sum_err += err;
  //  control_commands[RADIO_ROLL] = TRIM_PPRZ((err + d_err * roll_dot_dgain + roll_dot_sum_err * roll_dot_igain)
  control_commands[RADIO_ROLL] = TRIM_PPRZ((int16_t)((err + d_err * roll_dot_dgain) * roll_dot_pgain));

  err =  e_pitch_dot - setpoint_pitch_dot;
  d_err = err - pitch_dot_last_err;
  pitch_dot_last_err = err;
  //  pitch_dot_sum_err += err;
  //  control_commands[RADIO_PITCH] = TRIM_PPRZ((err + d_err * pitch_dot_dgain + pitch_dot_sum_err * pitch_dot_igain) * pitch_dot_pgain);
  control_commands[RADIO_PITCH] =  TRIM_PPRZ((int16_t)((err + d_err * pitch_dot_dgain) * pitch_dot_pgain));

  err = e_yaw_dot - setpoint_yaw_dot;
  //  d_err = err - yaw_dot_last_err;
  //  yaw_dot_last_err = err;
  //  yaw_dot_sum_err += err;
  //  control_commands[RADIO_YAW] = TRIM_PPRZ((err + d_err * yaw_dot_dgain + yaw_dot_sum_err * yaw_dot_igain) * yaw_dot_pgain);
  control_commands[RADIO_YAW] =  TRIM_PPRZ((int16_t)(err * yaw_dot_pgain));

  control_commands[RADIO_THROTTLE] = TRIM_PPRZ(setpoint_throttle);
}



void control_attitude_run ( void ) {
  int16_t err =  e_roll - setpoint_roll;
  setpoint_roll_dot = TRIM(roll_pgain * err, -MAX_ROLL_DOT, MAX_ROLL_DOT);

  err = e_pitch - setpoint_pitch;
  setpoint_pitch_dot = TRIM(pitch_pgain * err, -MAX_PITCH_DOT, MAX_PITCH_DOT);
}

void control_heading_run ( void ) {
  int16_t err = - (e_yaw/4 - setpoint_yaw/4);
  NORMALIZE(err, -8192, 8192);
  setpoint_yaw_dot = TRIM(yaw_pgain * err, -MAX_YAW_DOT, MAX_YAW_DOT);
}
#endif
