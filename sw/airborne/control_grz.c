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
#include "imu.h"
#include "main_fbw.h"

#define NORMALIZE(n, lower, uper) {		\
    while ( n > uper)  { n -= (uper - lower);}	\
    while ( n < lower) { n += (uper - lower);}	\
}

float fbw_roll_pgain = -0.8;
#define MAX_ROLL_DOT (1. * 32768000. / 8500.)

float fbw_pitch_pgain = -0.5;
#define MAX_PITCH_DOT (1. * 32768000. / 8500.)

float fbw_yaw_pgain = -2.5;
#define MAX_YAW_DOT (1. * 32768000. / 8500.)

float   roll_dot_pgain = -1.9;
float   roll_dot_igain = 0.0;
float   roll_dot_dgain = 0.0;
int16_t roll_dot_sum_err = 0;
int16_t roll_dot_last_err = 0;

float   pitch_dot_pgain = -1.9;
float   pitch_dot_igain = 0.0;
float   pitch_dot_dgain = 0.0;
int16_t pitch_dot_sum_err = 0;
int16_t pitch_dot_last_err = 0;

float   yaw_dot_pgain = -4.8;
float   yaw_dot_igain = 0.0;
float   yaw_dot_dgain = 0.0;
int16_t yaw_dot_sum_err = 0;
int16_t yaw_dot_last_err = 0;


void control_rate_run ( void ) {
  int16_t err =  roll_dot - commands[COMMAND_PHI_DOT];
  int16_t d_err = err - roll_dot_last_err;
  roll_dot_last_err = err;
  //  roll_dot_sum_err += err;
  //  control_commands[RADIO_ROLL] = TRIM_PPRZ((err + d_err * roll_dot_dgain + roll_dot_sum_err * roll_dot_igain)
  commands[COMMAND_ROLL] = TRIM_PPRZ((int16_t)((err + d_err * roll_dot_dgain) * roll_dot_pgain));

  err = pitch_dot - commands[COMMAND_THETA_DOT];
  d_err = err - pitch_dot_last_err;
  pitch_dot_last_err = err;
  //  pitch_dot_sum_err += err;
  //  control_commands[RADIO_PITCH] = TRIM_PPRZ((err + d_err * pitch_dot_dgain + pitch_dot_sum_err * pitch_dot_igain) * pitch_dot_pgain);
  commands[COMMAND_PITCH] =  TRIM_PPRZ((int16_t)((err + d_err * pitch_dot_dgain) * pitch_dot_pgain));

  err = yaw_dot - commands[COMMAND_PSI_DOT];
  //  d_err = err - yaw_dot_last_err;
  //  yaw_dot_last_err = err;
  //  yaw_dot_sum_err += err;
  //  control_commands[RADIO_YAW] = TRIM_PPRZ((err + d_err * yaw_dot_dgain + yaw_dot_sum_err * yaw_dot_igain) * yaw_dot_pgain);
  commands[COMMAND_YAW] =  TRIM_PPRZ((int16_t)(err * yaw_dot_pgain));
}


void control_attitude_run ( void ) {
  int16_t err =  roll - commands[COMMAND_PHI];
  commands[COMMAND_PHI_DOT] = Chop(fbw_roll_pgain * err, -MAX_ROLL_DOT, MAX_ROLL_DOT);

  err = pitch - commands[COMMAND_THETA];
  commands[COMMAND_THETA_DOT] = Chop(fbw_pitch_pgain * err, -MAX_PITCH_DOT, MAX_PITCH_DOT);
}

void control_heading_run ( void ) {
  int16_t err = - (yaw/4 - commands[COMMAND_PSI]/4);
  NORMALIZE(err, -8192, 8192);
  commands[COMMAND_PSI_DOT] = Chop(fbw_yaw_pgain * err, -MAX_YAW_DOT, MAX_YAW_DOT);
}

