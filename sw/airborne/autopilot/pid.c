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

/** \file pid.c
 *  \brief PID controllers (roll, pitch, climb, altitude, course).
 *
 */

#include <stdlib.h>
#include <math.h>

#include "pid.h"

#include "autopilot.h"
#include "infrared.h"
#include "estimator.h"
#include "nav.h"

float desired_roll = 0.;
float desired_pitch = 0.;
int16_t desired_gaz, desired_aileron, desired_elevator;
float roll_pgain = ROLL_PGAIN;
float pitch_pgain = PITCH_PGAIN;
float pitch_of_roll = PITCH_OF_ROLL;

float pitch_of_vz_pgain = CLIMB_PITCH_OF_VZ_PGAIN;
float pitch_of_vz = 0.;


/** \brief Computes ::desired_aileron and ::desired_elevator from attitude estimation and expected attitude. */
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
  nav_desired_roll = course_pgain * err; //  * fspeed / AIR_SPEED;
  if (nav_desired_roll > max_roll)
    nav_desired_roll = max_roll;
  else if (nav_desired_roll < -max_roll)
    nav_desired_roll = -max_roll;
}

const float climb_pgain   = CLIMB_PGAIN;
const float climb_igain   =  CLIMB_IGAIN;
float desired_climb = 0., pre_climb = 0.;
static const float level_gaz = CLIMB_LEVEL_GAZ;
float climb_sum_err  = 0;

float climb_pitch_pgain = CLIMB_PITCH_PGAIN;
float climb_pitch_igain = CLIMB_PITCH_IGAIN;
float climb_pitch_sum_err = 0.;
float max_pitch = MAX_PITCH;
float min_pitch = MIN_PITCH;


#define MAX_CLIMB_SUM_ERR 100
#define MAX_PITCH_CLIMB_SUM_ERR 100

/** \brief Computes desired_gaz and desired_pitch from desired_climb */
void 
climb_pid_run ( void ) {
  float err  = estimator_z_dot - desired_climb;
  if (auto_pitch) { /* gaz constant */
    desired_gaz = nav_desired_gaz;
    desired_pitch = climb_pitch_pgain * (err + climb_pitch_igain * climb_pitch_sum_err);
    if (desired_pitch > max_pitch)
      desired_pitch = max_pitch;
    if (desired_pitch < min_pitch)
      desired_pitch = min_pitch;
    climb_pitch_sum_err += err;
    if (climb_pitch_sum_err > MAX_PITCH_CLIMB_SUM_ERR)
      climb_pitch_sum_err = MAX_PITCH_CLIMB_SUM_ERR;
    if (climb_pitch_sum_err < - MAX_PITCH_CLIMB_SUM_ERR)
      climb_pitch_sum_err = - MAX_PITCH_CLIMB_SUM_ERR; 
  } else { /* pitch almost constant */
    /* pitch offset for climb */
    pitch_of_vz = (desired_climb > 0) ? desired_climb * pitch_of_vz_pgain : 0.;
    float fgaz = climb_pgain * (err + climb_igain * climb_sum_err) + CLIMB_LEVEL_GAZ + CLIMB_GAZ_OF_CLIMB*desired_climb;
    climb_sum_err += err;
    if (climb_sum_err > MAX_CLIMB_SUM_ERR) climb_sum_err = MAX_CLIMB_SUM_ERR;
    if (climb_sum_err < - MAX_CLIMB_SUM_ERR) climb_sum_err = - MAX_CLIMB_SUM_ERR;
    desired_gaz = TRIM_UPPRZ(fgaz * MAX_PPRZ);
    desired_pitch = nav_pitch + pitch_of_vz;
  }
}

float altitude_pgain = ALTITUDE_PGAIN;


void altitude_pid_run(void) {
  float err = estimator_z - desired_altitude;
  desired_climb = pre_climb + altitude_pgain * err;
  if (desired_climb < -CLIMB_MAX) desired_climb = -CLIMB_MAX;
  if (desired_climb > CLIMB_MAX) desired_climb = CLIMB_MAX;
}
