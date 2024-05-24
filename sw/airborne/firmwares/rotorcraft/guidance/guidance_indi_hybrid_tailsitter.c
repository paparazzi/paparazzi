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
 * @file firmwares/rotorcraft/guidance/guidance_indi_hybrid_tailsitter
 */

#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid_tailsitter.h"
#include "state.h"
#include "generated/modules.h"

/**
 * Calculate the matrix of partial derivatives of the roll, pitch and thrust
 * w.r.t. the NED accelerations, taking into account the lift of a wing that is
 * horizontal at -90 degrees pitch
 *
 * @param Gmat Dynamics matrix
 * @param a_diff acceleration errors in earth frame
 * @param body_v 3D vector to write the control objective v
 */

void guidance_indi_calcg_wing(float Gmat[GUIDANCE_INDI_HYBRID_V][GUIDANCE_INDI_HYBRID_U], struct FloatVect3 a_diff, float v_gih[GUIDANCE_INDI_HYBRID_V]) {
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
  //minus gravity is a guesstimate of the thrust force, thrust measurement would be better

#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
#endif

  /*Amount of lift produced by the wing*/
  float pitch_lift = eulers_zxy.theta;
  Bound(pitch_lift,-M_PI_2,0);
  float lift = sinf(pitch_lift)*9.81;
  float T = cosf(pitch_lift)*-9.81;

  // get the derivative of the lift wrt to theta
  float liftd = guidance_indi_get_liftd(stateGetAirspeed_f(), eulers_zxy.theta);

  Gmat[0][0] =  cphi*ctheta*spsi*T + cphi*spsi*lift;
  Gmat[1][0] = -cphi*ctheta*cpsi*T - cphi*cpsi*lift;
  Gmat[2][0] = -sphi*ctheta*T -sphi*lift;
  Gmat[0][1] = (ctheta*cpsi - sphi*stheta*spsi)*T*GUIDANCE_INDI_PITCH_EFF_SCALING + sphi*spsi*liftd;
  Gmat[1][1] = (ctheta*spsi + sphi*stheta*cpsi)*T*GUIDANCE_INDI_PITCH_EFF_SCALING - sphi*cpsi*liftd;
  Gmat[2][1] = -cphi*stheta*T*GUIDANCE_INDI_PITCH_EFF_SCALING + cphi*liftd;
  Gmat[0][2] = stheta*cpsi + sphi*ctheta*spsi;
  Gmat[1][2] = stheta*spsi - sphi*ctheta*cpsi;
  Gmat[2][2] = cphi*ctheta;

  v_gih[0] = a_diff.x;
  v_gih[1] = a_diff.y;
  v_gih[2] = a_diff.z;
}


#if GUIDANCE_INDI_HYBRID_USE_WLS



void guidance_indi_hybrid_set_wls_settings(float body_v[3] UNUSED, float roll_angle, float pitch_angle)
{
  // Set lower limits
  du_min_gih[0] = -guidance_indi_max_bank - roll_angle; // roll
  du_min_gih[1] = RadOfDeg(guidance_indi_min_pitch) - pitch_angle; // pitch
  du_min_gih[2] = (MAX_PPRZ - actuator_state_filt_vect[0]) * g1g2[3][0] + (MAX_PPRZ - actuator_state_filt_vect[1]) * g1g2[3][1] + (MAX_PPRZ - actuator_state_filt_vect[2]) * g1g2[3][2] + (MAX_PPRZ - actuator_state_filt_vect[3]) * g1g2[3][3];

  // Set upper limits limits
  du_max_gih[0] = guidance_indi_max_bank - roll_angle; //roll
  du_max_gih[1] = RadOfDeg(GUIDANCE_INDI_MAX_PITCH) - pitch_angle; // pitch
  du_max_gih[2] = -(actuator_state_filt_vect[0]*g1g2[3][0] + actuator_state_filt_vect[1]*g1g2[3][1] + actuator_state_filt_vect[2]*g1g2[3][2] + actuator_state_filt_vect[3]*g1g2[3][3]);

  // Set prefered states
  du_pref_gih[0] = -roll_angle; // prefered delta roll angle
  du_pref_gih[1] = -pitch_angle; // prefered delta pitch angle
  du_pref_gih[2] = du_max_gih[2];
}


#endif


