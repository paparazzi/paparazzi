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
#include "gyro.h"
#include "estimator.h"
#include "nav.h"

/* roll loop parameters */
float desired_roll;
float roll_pgain;
pprz_t desired_aileron;

/* pitch loop parameters */
float desired_pitch;
float pitch_pgain;
pprz_t desired_elevator;

/* pre-command */
float pitch_of_roll;
float pitch_of_vz_pgain;
float pitch_of_vz;
float aileron_of_gaz;

pprz_t desired_gaz;

#ifdef PID_RATE_LOOP
float alt_roll_pgain;  // alternate roll pgain
float roll_rate_pgain;
float roll_rate_dgain;
float roll_rate_igain;
float rate_mode;
#ifndef RATE_MODE_DEFAULT
#define RATE_MODE_DEFAULT 1
#endif
#endif

inline static void pid_roll_loop_run(void);
inline static void pid_pitch_loop_run(void);

void pid_init( void ) {
  desired_roll = 0.;
  roll_pgain = ROLL_PGAIN;

  desired_pitch = 0.;
  pitch_pgain = PITCH_PGAIN;

  pitch_of_roll = PITCH_OF_ROLL;
  pitch_of_vz = 0.;
  pitch_of_vz_pgain = CLIMB_PITCH_OF_VZ_PGAIN;
  aileron_of_gaz = AILERON_OF_GAZ;

  desired_gaz = 0;
  desired_aileron = 0;
  desired_elevator = 0;

#ifdef PID_RATE_LOOP
  alt_roll_pgain = ALT_ROLL_PGAIN;
  roll_rate_pgain = ROLL_RATE_PGAIN;
  roll_rate_igain = ROLL_RATE_IGAIN;
  roll_rate_dgain = ROLL_RATE_DGAIN;
  rate_mode = RATE_MODE_DEFAULT;
#endif

}

/** \brief Computes ::desired_aileron and ::desired_elevator from attitude
 * estimation and expected attitude.
*/
void pid_attitude_loop_run( void ) {
  pid_roll_loop_run();
  pid_pitch_loop_run();
}


inline static void pid_roll_loop_run(void) {
  /** Attitude PID */
  float err =  estimator_phi - desired_roll;
  Bound(err, -M_PI/2., M_PI/2);

  float std_desired_aileron = TRIM_PPRZ(roll_pgain * err + desired_gaz * aileron_of_gaz);
#ifndef PID_RATE_LOOP
  desired_aileron = std_desired_aileron;
#else
  float desired_roll_rate = -alt_roll_pgain*err;
  Bound(desired_roll_rate, -GYRO_MAX_RATE, GYRO_MAX_RATE);

  /** Roll rate pid */
  err = roll_rate - desired_roll_rate;
  Bound(err, -GYRO_MAX_RATE, GYRO_MAX_RATE);
  
  static float rollrate_sum_err = 0.;
  static uint8_t rollratesum_idx = 0;
  static float rollratesum_values[ROLLRATESUM_NB_SAMPLES];

  rollrate_sum_err -= rollratesum_values[rollratesum_idx];
  rollratesum_values[rollratesum_idx] = err;
  rollrate_sum_err += err;
  rollratesum_idx++;
  if (rollratesum_idx >= ROLLRATESUM_NB_SAMPLES) rollratesum_idx = 0;
  
  /* D term calculations */
  static float last_roll_rate;
  float d_err = roll_rate - last_roll_rate;
  last_roll_rate = roll_rate;

  float rate_desired_aileron = TRIM_PPRZ(roll_rate_pgain*(err + roll_rate_igain*(rollrate_sum_err/ROLLRATESUM_NB_SAMPLES) + roll_rate_dgain*d_err));

  desired_aileron = rate_mode*rate_desired_aileron+(1-rate_mode)*std_desired_aileron;
#endif

}

/** Pitch loop */
inline static void pid_pitch_loop_run(void) {
  /* sanity check */
  if (pitch_of_roll <0.)
    pitch_of_roll = 0.;

  float err = -(estimator_theta - desired_pitch - pitch_of_roll*fabs(estimator_phi));
  Bound(err, -M_PI/2, M_PI/2);
  desired_elevator = TRIM_PPRZ(pitch_pgain * err);
}


#ifndef CLIMB_MAX_DIFF_GAZ
#define CLIMB_MAX_DIFF_GAZ 1.
#endif

void pid_slew_gaz( void ) {
  static pprz_t last_gaz;
  pprz_t diff_gaz = desired_gaz - last_gaz;
  Bound(diff_gaz, -TRIM_PPRZ(CLIMB_MAX_DIFF_GAZ*MAX_PPRZ), TRIM_PPRZ(CLIMB_MAX_DIFF_GAZ*MAX_PPRZ));
  desired_gaz = last_gaz + diff_gaz;
  last_gaz = desired_gaz;
}
