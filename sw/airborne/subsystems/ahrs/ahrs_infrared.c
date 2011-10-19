/*
 * Copyright (C) 2011 The Paparazzi Team
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

#include "subsystems/ahrs/ahrs_infrared.h"

#include "subsystems/sensors/infrared.h"
#include "subsystems/gps.h"

#include "estimator.h"



void ahrs_init(void) {
  ahrs.status = AHRS_UNINIT;

  /* set ltp_to_body to zero */
  FLOAT_QUAT_ZERO(ahrs_float.ltp_to_body_quat);
  FLOAT_EULERS_ZERO(ahrs_float.ltp_to_body_euler);
  FLOAT_RMAT_ZERO(ahrs_float.ltp_to_body_rmat);
  FLOAT_RATES_ZERO(ahrs_float.body_rate);

  /* set ltp_to_body to zero */
  FLOAT_QUAT_ZERO(ahrs_float.ltp_to_imu_quat);
  FLOAT_EULERS_ZERO(ahrs_float.ltp_to_imu_euler);
  FLOAT_RMAT_ZERO(ahrs_float.ltp_to_imu_rmat);
  FLOAT_RATES_ZERO(ahrs_float.imu_rate);
}

void ahrs_align(void) {

  //TODO set gyro bias if used

  ahrs.status = AHRS_RUNNING;
}

void ahrs_propagate(void) {
}

void ahrs_update_accel(void) {
}

void ahrs_update_mag(void) {
}

void ahrs_update_gps(void) {

  float hspeed_mod_f = gps.gspeed / 100.;
  float course_f = gps.course / 1e7;

  // Heading estimator from wind-information, usually computed with -DWIND_INFO
  // wind_north and wind_east initialized to 0, so still correct if not updated
  float w_vn = cosf(course_f) * hspeed_mod_f - wind_north;
  float w_ve = sinf(course_f) * hspeed_mod_f - wind_east;
  ahrs_float.ltp_to_body_euler.psi = atan2f(w_ve, w_vn);
  if (ahrs_float.ltp_to_body_euler.psi < 0.)
    ahrs_float.ltp_to_body_euler.psi += 2 * M_PI;

  ahrs_update_fw_estimator();
}

void ahrs_update_infrared(void) {
  ahrs_float.ltp_to_body_euler.phi  = atan2(infrared.roll, infrared.top) - infrared.roll_neutral;

  ahrs_float.ltp_to_body_euler.theta  = atan2(infrared.pitch, infrared.top) - infrared.pitch_neutral;

  if (ahrs_float.ltp_to_body_euler.theta < -M_PI_2)
    ahrs_float.ltp_to_body_euler.theta += M_PI;
  else if (ahrs_float.ltp_to_body_euler.theta > M_PI_2)
    ahrs_float.ltp_to_body_euler.theta -= M_PI;

  if (ahrs_float.ltp_to_body_euler.phi >= 0)
    ahrs_float.ltp_to_body_euler.phi *= infrared.correction_right;
  else
    ahrs_float.ltp_to_body_euler.phi *= infrared.correction_left;

  if (ahrs_float.ltp_to_body_euler.theta >= 0)
    ahrs_float.ltp_to_body_euler.theta *= infrared.correction_up;
  else
    ahrs_float.ltp_to_body_euler.theta *= infrared.correction_down;

  ahrs_update_fw_estimator();
}


// TODO use ahrs result directly
void ahrs_update_fw_estimator(void)
{
  // export results to estimator

  estimator_phi   = ahrs_float.ltp_to_body_euler.phi;
  estimator_theta = ahrs_float.ltp_to_body_euler.theta;
  estimator_psi   = ahrs_float.ltp_to_body_euler.psi;

  estimator_p = ahrs_float.body_rate.p;
  estimator_q = ahrs_float.body_rate.q;

}
