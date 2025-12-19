/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               20025 Mohamad Hachem
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi_fully_actuated.c
 *
 * Fully actuated plateform can be achieve with hexa-copter with
 * tilted propellers for example
 * TODO cite Mohamad Hachem
 */

#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "state.h"
#include "generated/modules.h"

// FIXME should be a function of the mass

#ifndef GUIDANCE_INDI_MAX_H_THRUST
#define GUIDANCE_INDI_MAX_H_THRUST 1.f
#endif

#ifndef GUIDANCE_INDI_MAX_V_THRUST
#define GUIDANCE_INDI_MAX_V_THRUST 15.f
#endif

float guidance_indi_max_h_thrust = GUIDANCE_INDI_MAX_H_THRUST;
float guidance_indi_max_v_thrust = GUIDANCE_INDI_MAX_V_THRUST;

/**
 * @param Gmat array to write the matrix to [3x6]
 *
 * Calculate the matrix of partial derivatives of the pitch, roll and thrust.
 * w.r.t. the NED accelerations for YXZ eulers
 * ddx = G*[dphi,dtheta,dpsi,dTx,dTy,dTz]
 */
static void guidance_indi_calcG_yxz(float Gmat[3][6], struct FloatEulers euler_yxz)
{
  // Euler Angles
  float sphi = sinf(euler_yxz.phi);
  float cphi = cosf(euler_yxz.phi);
  float stheta = sinf(euler_yxz.theta);
  float ctheta = cosf(euler_yxz.theta);
  float cpsi  = cosf(euler_yxz.psi);
  float spsi  = sinf(euler_yxz.psi);

  // Estimated Thrust
  float Tx = stab_thrust_filt.x;
  float Ty = stab_thrust_filt.y;
  float Tz = stab_thrust_filt.z;

  // dPhi (Roll)
  Gmat[0][0] = Tx * (-spsi * cphi * stheta) + Ty * (spsi * sphi) + Tz * (spsi * cphi * ctheta);
  Gmat[1][0] = Tx * (cpsi * cphi * stheta) + Ty * (-cpsi * sphi) + Tz * (-cpsi * cphi * ctheta);
  Gmat[2][0] = Tx * (sphi * stheta) + Ty * (cphi) + Tz * (-sphi * ctheta);

  // dTheta (Pitch)
  Gmat[0][1] = Tx * (-cpsi * stheta - spsi * sphi * ctheta) + Tz * (cpsi * ctheta - spsi * sphi * stheta);
  Gmat[1][1] = Tx * (-spsi * stheta + cpsi * sphi * ctheta) + Tz * (spsi * ctheta + cpsi * sphi * stheta);
  Gmat[2][1] = Tx * (-cphi * ctheta)                        + Tz * (-cphi * stheta);

  // dPsi added to add constraints on psi
  Gmat[0][2] = 0.f;
  Gmat[1][2] = 0.f;
  Gmat[2][2] = 0.f;

  // dTx
  Gmat[0][3] =  cpsi * ctheta - spsi * sphi * stheta;
  Gmat[1][3] =  spsi * ctheta + cpsi * sphi * stheta;
  Gmat[2][3] = -stheta * cphi;

  // dTy
  Gmat[0][4] = -spsi * cphi;
  Gmat[1][4] =  cpsi * cphi;
  Gmat[2][4] =  sphi;

  // dTz
  Gmat[0][5] = cpsi * stheta + spsi * sphi * ctheta;
  Gmat[1][5] = spsi * stheta - cpsi * sphi * ctheta;
  Gmat[2][5] = cphi * ctheta;
}

/** Compute effectiveness matrix for guidance
 *
 * @param Gmat Dynamics matrix
 */
void guidance_indi_calcG(float Gmat[3][6], struct FloatEulers euler) {
  guidance_indi_calcG_yxz(Gmat, euler);
}

void guidance_indi_set_wls_settings(struct WLS_t *wls, struct FloatEulers *euler_yxz, float heading_sp UNUSED)
{
  struct FloatEulers euler_yxz_ref = { 0 };
#if GUIDANCE_INDI_RC_SWITCH_EULER
  euler_yxz_ref.phi = (radio_control_get(RADIO_PITCH) / MAX_PPRZ) * guidance_indi_max_bank;
  euler_yxz_ref.theta = (radio_control_get(RADIO_ROLL) / MAX_PPRZ) * guidance_indi_max_bank;
  euler_yxz_ref.psi = heading_sp // TODO check this one
#endif

  // Set lower limits
  wls->u_min[0] = -guidance_indi_max_bank - euler_yxz->phi;          //phi
  wls->u_min[1] = -guidance_indi_max_bank - euler_yxz->theta;        //theta
  wls->u_min[2] = -M_PI - euler_yxz->psi;                            //psi FIXME M_PI or a lower bound ? (was 1 in initial code)
  wls->u_min[3] = -guidance_indi_max_h_thrust - stab_thrust_filt.x;  //Tx
  wls->u_min[4] = -guidance_indi_max_h_thrust - stab_thrust_filt.y;  //Ty
  wls->u_min[5] = -guidance_indi_max_v_thrust - stab_thrust_filt.z;  //Tz (MAX_PPRZ  - stabilization.cmd[COMMAND_THRUST])

  // ->r limits limits
  wls->u_max[0] = guidance_indi_max_bank - euler_yxz->phi;          //phi
  wls->u_max[1] = guidance_indi_max_bank - euler_yxz->theta;        //theta
  wls->u_max[2] = M_PI - euler_yxz->psi;                            //psi
  wls->u_max[3] = guidance_indi_max_h_thrust - stab_thrust_filt.x;  //Tx
  wls->u_max[4] = guidance_indi_max_h_thrust - stab_thrust_filt.y;  //Ty
  wls->u_max[5] = 9.81f * GUIDANCE_INDI_MASS - stab_thrust_filt.z;  //Tz

  // ->ered states
  wls->u_pref[0] = euler_yxz_ref.phi - euler_yxz->phi;     // prefered delta phi
  wls->u_pref[1] = euler_yxz_ref.theta - euler_yxz->theta; // prefered delta theta
  wls->u_pref[2] = euler_yxz_ref.psi - euler_yxz->psi;     // prefered Ty
  wls->u_pref[3] = stab_thrust_filt.x;                      // prefred Tx
  wls->u_pref[4] = stab_thrust_filt.y;                      // prefered Ty
  wls->u_pref[5] = stab_thrust_filt.z;                      // prefered Tz
}

