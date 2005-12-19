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

float aileron_of_gaz = AILERON_OF_GAZ;


/** \brief Computes ::desired_aileron and ::desired_elevator from attitude
 * estimation and expected attitude.
*/
void roll_pitch_pid_run( void ) {
  float err =  estimator_phi - desired_roll;
  desired_aileron = TRIM_PPRZ(roll_pgain * err + desired_gaz * aileron_of_gaz);
  if (pitch_of_roll <0.)
    pitch_of_roll = 0.;
  err = -(estimator_theta - desired_pitch - pitch_of_roll * fabs(estimator_phi));
  desired_elevator = TRIM_PPRZ(pitch_pgain * err);
}
