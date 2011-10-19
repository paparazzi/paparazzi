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


#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_sim.h"
#include "math/pprz_algebra_float.h"
#include "generated/airframe.h"

#include <inttypes.h>
#include <caml/mlvalues.h>

extern float sim_phi;
extern float sim_theta;
extern float sim_psi;
extern float sim_p;
extern float sim_q;
extern bool_t ahrs_sim_available;


void compute_body_orientation_and_rates(void);

void update_ahrs_from_sim(void) {
  ahrs_float.ltp_to_imu_euler.phi = sim_phi;
  ahrs_float.ltp_to_imu_euler.theta = sim_theta;
  ahrs_float.ltp_to_imu_euler.psi = sim_psi;

  ahrs_float.imu_rate.p = sim_p;
  ahrs_float.imu_rate.q = sim_q;

  /* set quaternion and rotation matrix representations as well */
  FLOAT_QUAT_OF_EULERS(ahrs_float.ltp_to_imu_quat, ahrs_float.ltp_to_imu_euler);
  FLOAT_RMAT_OF_EULERS(ahrs_float.ltp_to_imu_rmat, ahrs_float.ltp_to_imu_euler);

  compute_body_orientation_and_rates();
}


void ahrs_init(void) {
  //ahrs_float.status = AHRS_UNINIT;
  // set to running for now and only use ahrs.status, not ahrs_float.status
  ahrs.status = AHRS_RUNNING;

  ahrs_sim_available = FALSE;

  /* set ltp_to_body to zero */
  FLOAT_QUAT_ZERO(ahrs_float.ltp_to_body_quat);
  FLOAT_EULERS_ZERO(ahrs_float.ltp_to_body_euler);
  FLOAT_RMAT_ZERO(ahrs_float.ltp_to_body_rmat);
  FLOAT_RATES_ZERO(ahrs_float.body_rate);

  /* set ltp_to_imu to same as ltp_to_body, currently no difference simulated */
  QUAT_COPY(ahrs_float.ltp_to_imu_quat, ahrs_float.ltp_to_body_quat);
  EULERS_COPY(ahrs_float.ltp_to_imu_euler, ahrs_float.ltp_to_body_euler);
  RMAT_COPY(ahrs_float.ltp_to_imu_rmat, ahrs_float.ltp_to_body_rmat);
  RATES_COPY(ahrs_float.imu_rate, ahrs_float.body_rate);
}

void ahrs_align(void)
{
  /* Currently not really simulated
   * body and imu have the same frame and always set to true value from sim
   */

  update_ahrs_from_sim();

  /* Compute initial body orientation */
  compute_body_orientation_and_rates();

  ahrs.status = AHRS_RUNNING;
}


void ahrs_propagate(void) {
}

void ahrs_update_accel(void) {
}

void ahrs_update_mag(void) {
}


/*
 * Compute body orientation and rates from imu orientation and rates
 */
void compute_body_orientation_and_rates(void) {

  /* set ltp_to_body to same as ltp_to_imu, currently no difference simulated */

  QUAT_COPY(ahrs_float.ltp_to_body_quat, ahrs_float.ltp_to_imu_quat);
  EULERS_COPY(ahrs_float.ltp_to_body_euler, ahrs_float.ltp_to_imu_euler);
  RMAT_COPY(ahrs_float.ltp_to_body_rmat, ahrs_float.ltp_to_imu_rmat);
  RATES_COPY(ahrs_float.body_rate, ahrs_float.imu_rate);
}


#ifdef AHRS_UPDATE_FW_ESTIMATOR
// TODO use ahrs result directly
#include "estimator.h"
// remotely settable
#ifndef INS_ROLL_NEUTRAL_DEFAULT
#define INS_ROLL_NEUTRAL_DEFAULT 0
#endif
#ifndef INS_PITCH_NEUTRAL_DEFAULT
#define INS_PITCH_NEUTRAL_DEFAULT 0
#endif
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
void ahrs_update_fw_estimator(void)
{
  // really subtract ins neutrals here?
  estimator_phi   = ahrs_float.ltp_to_body_euler.phi - ins_roll_neutral;
  estimator_theta = ahrs_float.ltp_to_body_euler.theta - ins_pitch_neutral;
  estimator_psi   = ahrs_float.ltp_to_body_euler.psi;

  estimator_p = ahrs_float.body_rate.p;
  estimator_q = ahrs_float.body_rate.q;
}
#endif //AHRS_UPDATE_FW_ESTIMATOR
