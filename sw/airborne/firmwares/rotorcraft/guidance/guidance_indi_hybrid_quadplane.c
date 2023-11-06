/*
 * Copyright (C) 2015 Ewoud Smeur <ewoud.smeur@gmail.com>
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
 * @file firmwares/rotorcraft/guidance/guidance_indi_hybrid_quadplane.c
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "filters/low_pass_filter.h"
#include "state.h"
#include "autopilot.h"
#include "generated/modules.h"


#ifndef COMMAND_THRUST_X
#error "Quadplanes require a forward thrust actuator"
#endif

#ifndef GUIDANCE_INDI_THRUST_Z_EFF
#error "You need to define GUIDANCE_INDI_THRUST_Z_EFF"
#else
float guidance_indi_thrust_z_eff = GUIDANCE_INDI_THRUST_Z_EFF;
#endif

#ifndef GUIDANCE_INDI_THRUST_X_EFF
#error "You need to define GUIDANCE_INDI_THRUST_X_EFF"
#else
float guidance_indi_thrust_x_eff = GUIDANCE_INDI_THRUST_X_EFF;
#endif


float bodyz_filter_cutoff = 0.2;

Butterworth2LowPass accel_bodyz_filt;



/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_quadplane_init(void) {
  float tau_bodyz = 1.0/(2.0*M_PI*bodyz_filter_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&accel_bodyz_filt, tau_bodyz, sample_time, -9.81);
}


/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 * The roll and pitch also need to be filtered to synchronize them with the
 * acceleration
 * Called as a periodic function with PERIODIC_FREQ
 */
void guidance_indi_quadplane_propagate_filters(void) {
   // Propagate filter for thrust/lift estimate
  float accelz = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->z);
  update_butterworth_2_low_pass(&accel_bodyz_filt, accelz);
}




/**
 * Perform WLS
 *
 * @param Gmat Dynamics matrix
 * @param a_diff acceleration errors in earth frame
 * @param body_v 3D vector to write the control objective v
 */
void guidance_indi_calcg_wing(float Gmat[GUIDANCE_INDI_HYBRID_V][GUIDANCE_INDI_HYBRID_U], struct FloatVect3 a_diff, float body_v[GUIDANCE_INDI_HYBRID_V]) {
  // Get attitude
  struct FloatEulers eulers_zxy;
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  /*Pre-calculate sines and cosines*/
  float sphi = sinf(eulers_zxy.phi);
  float cphi = cosf(eulers_zxy.phi);
  float stheta = sinf(eulers_zxy.theta);
  float ctheta = cosf(eulers_zxy.theta);
  float spsi = sinf(eulers_zxy.psi);
  float cpsi = cosf(eulers_zxy.psi);

#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
#endif

  /*Amount of lift produced by the wing*/
  float lift_thrust_bz = accel_bodyz_filt.o[0]; // Sum of lift and thrust in boxy z axis (level flight)

  // get the derivative of the lift wrt to theta
  float liftd = guidance_indi_get_liftd(0.0f, 0.0f);

  Gmat[0][0] = -sphi*stheta*lift_thrust_bz;
  Gmat[1][0] = -cphi*lift_thrust_bz;
  Gmat[2][0] = -sphi*ctheta*lift_thrust_bz;

  Gmat[0][1] =  cphi*ctheta*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING;
  Gmat[1][1] =  sphi*stheta*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING - sphi*liftd;
  Gmat[2][1] = -cphi*stheta*lift_thrust_bz*GUIDANCE_INDI_PITCH_EFF_SCALING + cphi*liftd;

  Gmat[0][2] =  cphi*stheta;
  Gmat[1][2] = -sphi;
  Gmat[2][2] =  cphi*ctheta;

  Gmat[0][3] =  ctheta;
  Gmat[1][3] =  0;
  Gmat[2][3] = -stheta;

  // Convert acceleration error to body axis system
  body_v[0] =  cpsi * a_diff.x + spsi * a_diff.y;
  body_v[1] = -spsi * a_diff.x + cpsi * a_diff.y;
  body_v[2] =  a_diff.z;
}


void WEAK guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle)
{
  float roll_limit_rad = GUIDANCE_H_MAX_BANK;
  float max_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MAX_PITCH);
  float min_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MIN_PITCH);

  float pitch_pref_rad = RadOfDeg(guidance_indi_pitch_pref_deg);

  // Set lower limits
  du_min_gih[0] = -roll_limit_rad - roll_angle; //roll
  du_min_gih[1] = min_pitch_limit_rad - pitch_angle; // pitch
  du_min_gih[2] = (MAX_PPRZ - stabilization_cmd[COMMAND_THRUST]) * guidance_indi_thrust_z_eff;
  du_min_gih[3] = -stabilization_cmd[COMMAND_THRUST_X]*guidance_indi_thrust_x_eff;

  // Set upper limits limits
  du_max_gih[0] = roll_limit_rad - roll_angle; //roll
  du_max_gih[1] = max_pitch_limit_rad - pitch_angle; // pitch
  du_max_gih[2] = -stabilization_cmd[COMMAND_THRUST] * guidance_indi_thrust_z_eff;
  du_max_gih[3] = (MAX_PPRZ - stabilization_cmd[COMMAND_THRUST_X])*guidance_indi_thrust_x_eff;

  // Set prefered states
  du_pref_gih[0] = -roll_angle; // prefered delta roll angle
  du_pref_gih[1] = -pitch_angle + pitch_pref_rad;// prefered delta pitch angle
  du_pref_gih[2] = du_max_gih[2]; // Low thrust better for efficiency
  du_pref_gih[3] = body_v[0]; // solve the body acceleration
}


/** Quadplanes can still be in-flight with COMMAND_THRUST==0 and can even soar not descending in updrafts with all thrust off */
bool autopilot_in_flight_end_detection(bool motors_on UNUSED) {
  return ! motors_on;
}