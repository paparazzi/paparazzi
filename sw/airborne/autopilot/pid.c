/*
 * $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

#include <stdlib.h>
#include <math.h>

#include "pid.h"

#include "autopilot.h"
#include "infrared.h"
//#include "gps.h"
#include "estimator.h"
#include "nav.h"

float desired_roll = 0.;
float desired_pitch = 0.;
int16_t desired_gaz, desired_aileron, desired_elevator;
float roll_pgain = ROLL_PGAIN;
float pitch_pgain = PITCH_PGAIN;
float pitch_of_roll = PITCH_OF_ROLL;

void roll_pitch_pid_run( void ) {
  float err =  estimator_phi - desired_roll;
  desired_aileron = TRIM_PPRZ(roll_pgain * err);
  if (pitch_of_roll <0.)
    pitch_of_roll = 0.;
  err = -(estimator_theta - desired_pitch - pitch_of_roll * fabs(estimator_phi));
  desired_elevator = TRIM_PPRZ(pitch_pgain * err);
}

float course_pgain = COURSE_PGAIN;
float desired_course = 0.;
float max_roll = MAX_ROLL;

void course_pid_run( void ) {
  float err = estimator_hspeed_dir - desired_course;
  NORM_RAD_ANGLE(err);
  float roll = course_pgain * err; //  * fspeed / AIR_SPEED;
  if (roll > max_roll)
    desired_roll = max_roll;
  else if (roll < -max_roll)
    desired_roll = -max_roll;
  else desired_roll = roll;
}

const float climb_pgain   = CLIMB_PGAIN;
const float climb_igain   =  CLIMB_IGAIN;
float desired_climb = 0., pre_climb = 0.;
static const float level_gaz = CLIMB_LEVEL_GAZ;
float climb_sum_err  = 0;

void climb_pid_run ( void ) {
  float err  = estimator_z_dot - desired_climb;
  float fgaz = climb_pgain * (err + climb_igain * climb_sum_err) + CLIMB_LEVEL_GAZ + (0.9-CLIMB_LEVEL_GAZ)/CLIMB_GAZ_MAX*desired_climb;
  climb_sum_err += err;
  desired_gaz = TRIM_UPPRZ(fgaz * MAX_PPRZ);
}

float altitude_pgain = ALTITUDE_PGAIN;


void altitude_pid_run(void) {
  float err = estimator_z - desired_altitude;
  desired_climb = pre_climb + altitude_pgain * err;
  if (desired_climb < -CLIMB_MAX) desired_climb = -CLIMB_MAX;
  if (desired_climb > CLIMB_MAX) desired_climb = CLIMB_MAX;
}
