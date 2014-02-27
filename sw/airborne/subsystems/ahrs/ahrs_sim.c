/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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

/**
 * @file subsystems/ahrs/ahrs_sim.c
 *
 * Dummy plug to set the AHRS from the simple OCaml simulator.
 *
 */

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_sim.h"
#include "math/pprz_algebra_float.h"
#include "generated/airframe.h"

extern float sim_phi;
extern float sim_theta;
extern float sim_psi;
extern float sim_p;
extern float sim_q;
extern float sim_r;
extern bool_t ahrs_sim_available;

#ifdef AHRS_UPDATE_FW_ESTIMATOR
// remotely settable
#ifndef INS_ROLL_NEUTRAL_DEFAULT
#define INS_ROLL_NEUTRAL_DEFAULT 0
#endif
#ifndef INS_PITCH_NEUTRAL_DEFAULT
#define INS_PITCH_NEUTRAL_DEFAULT 0
#endif
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
#endif //AHRS_UPDATE_FW_ESTIMATOR

void update_ahrs_from_sim(void) {

  struct FloatEulers ltp_to_imu_euler = { sim_phi, sim_theta, sim_psi };
#ifdef AHRS_UPDATE_FW_ESTIMATOR
  ltp_to_imu_euler.phi -= ins_roll_neutral;
  ltp_to_imu_euler.theta -= ins_pitch_neutral;
#endif
  struct FloatRates imu_rate = { sim_p, sim_q, sim_r };
  /* set ltp_to_body to same as ltp_to_imu, currently no difference simulated */
  stateSetNedToBodyEulers_f(&ltp_to_imu_euler);
  stateSetBodyRates_f(&imu_rate);

}


void ahrs_init(void) {
  //ahrs_float.status = AHRS_UNINIT;
  // set to running for now
  ahrs.status = AHRS_RUNNING;

  ahrs_sim_available = FALSE;

}

void ahrs_align(void)
{
  /* Currently not really simulated
   * body and imu have the same frame and always set to true value from sim
   */

  update_ahrs_from_sim();

  ahrs.status = AHRS_RUNNING;
}


void ahrs_propagate(void) {
  if (ahrs_sim_available) {
    update_ahrs_from_sim();
    ahrs_sim_available = FALSE;
  }
}

