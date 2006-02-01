/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2005 Pascal Brisset, Antoine Drouin
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

#ifdef RATE_LOOP
#warning "Compiling control.c"

#include "control.h"
#include "imu.h"


int16_t desired_roll_dot;
float   roll_dot_pgain = ROLL_DOT_PGAIN;
float   roll_dot_igain = ROLL_DOT_IGAIN;
float   roll_dot_dgain = ROLL_DOT_DGAIN;
int16_t roll_dot_sum_err = 0;
int16_t roll_dot_last_err = 0;
pprz_t  command_roll; 

int16_t desired_pitch_dot;
float   pitch_dot_pgain = PITCH_DOT_PGAIN;
float   pitch_dot_igain = PITCH_DOT_IGAIN;
float   pitch_dot_dgain = PITCH_DOT_DGAIN;
int16_t pitch_dot_sum_err = 0;
int16_t pitch_dot_last_err = 0;
pprz_t  command_pitch; 

int16_t desired_yaw_dot;
float   yaw_dot_pgain = YAW_DOT_PGAIN;
float   yaw_dot_igain = YAW_DOT_IGAIN;
float   yaw_dot_dgain = YAW_DOT_DGAIN;
int16_t yaw_dot_sum_err = 0;
int16_t yaw_dot_last_err = 0;
pprz_t  command_yaw; 

int16_t desired_throttle;
pprz_t  command_throttle;

pprz_t control_commands[] = {0, 0, 0, 0};


#define TRIM(a, m)  ((a) > m ? m : ((a) < -m ? -m : (a)))

#define Int16(x) (x > 32768. ? 32768 : x < -32768 ? -32768 : (int16_t)x)


void control_run ( void ) {
  int16_t err =   roll_dot - desired_roll_dot;
  int16_t d_err = err - roll_dot_last_err;
  roll_dot_last_err = err;
  //  roll_dot_sum_err += err;
  //  control_commands[RADIO_ROLL] = TRIM_PPRZ((err + d_err * roll_dot_dgain + roll_dot_sum_err * roll_dot_igain) * roll_dot_pgain);
  control_commands[COMMAND_ROLL] = TRIM_PPRZ(Int16((err +d_err * roll_dot_dgain) * roll_dot_pgain));

  err =  pitch_dot - desired_pitch_dot;
  d_err = err - pitch_dot_last_err;
  pitch_dot_last_err = err;
  //  pitch_dot_sum_err += err;
  //  control_commands[RADIO_PITCH] = TRIM_PPRZ((err + d_err * pitch_dot_dgain + pitch_dot_sum_err * pitch_dot_igain) * pitch_dot_pgain);
  control_commands[COMMAND_PITCH] =  TRIM_PPRZ(Int16((err + d_err * pitch_dot_dgain) * pitch_dot_pgain));

  err = yaw_dot - desired_yaw_dot;
  //  d_err = err - yaw_dot_last_err;
  //  yaw_dot_last_err = err;
  //  yaw_dot_sum_err += err;
  //  control_commands[RADIO_YAW] = TRIM_PPRZ((err + d_err * yaw_dot_dgain + yaw_dot_sum_err * yaw_dot_igain) * yaw_dot_pgain);
  control_commands[COMMAND_YAW] =  TRIM_PPRZ(Int16(err * yaw_dot_pgain));

  control_commands[COMMAND_THROTTLE] = TRIM_PPRZ(desired_throttle);
}

void control_set_desired ( const pprz_t values[] ) {
  desired_roll_dot  =  DESIRED_OF_RADIO_ROLL_DOT*values[COMMAND_ROLL];
  desired_pitch_dot =  DESIRED_OF_RADIO_PITCH_DOT*values[COMMAND_PITCH];
  desired_yaw_dot   =  DESIRED_OF_RADIO_YAW_DOT*values[COMMAND_YAW];
  desired_throttle  =  values[COMMAND_THROTTLE];
}

#endif
