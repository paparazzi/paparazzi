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
#define GUIDANCE_INDI_MAX_H_THRUST 2.f
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
void guidance_indi_calcG(float Gmat[3][6], const struct FloatQuat *att)
{
  struct FloatEulers euler_yxz;
  float_eulers_of_quat_yxz(&euler_yxz, att);
  // Euler Angles
  const float sphi = sinf(euler_yxz.phi);
  const float cphi = cosf(euler_yxz.phi);
  const float stheta = sinf(euler_yxz.theta);
  const float ctheta = cosf(euler_yxz.theta);
  const float cpsi  = cosf(euler_yxz.psi);
  const float spsi  = sinf(euler_yxz.psi);

  // Estimated Thrust
  const float Tx = stab_thrust_filt.x;
  const float Ty = stab_thrust_filt.y;
  const float Tz = stab_thrust_filt.z;

  // dPhi (Roll)
  Gmat[0][0] = Tx * (stheta * cphi * spsi) + Ty * (stheta * cphi * cpsi) + Tz * (-stheta * sphi);
  Gmat[1][0] = Tx * (-sphi * spsi) + Ty * (-sphi * cpsi) + Tz * (-cphi);
  Gmat[2][0] = Tx * (ctheta * cphi * spsi) + Ty * (ctheta * cphi * cpsi) + Tz * (-ctheta * sphi);

  // dTheta (Pitch)
  Gmat[0][1] = Tx * (-stheta * cpsi + ctheta * sphi * spsi) + Ty * (stheta * spsi + ctheta * sphi * cpsi) + Tz * (ctheta * cphi);
  Gmat[1][1] = 0.f;
  Gmat[2][1] = Tx * (-ctheta * cpsi - stheta * sphi * spsi) +Ty * (ctheta * spsi - stheta * sphi * cpsi) + Tz * (-stheta * cphi);

  // dPsi added to add constraints on psi
  Gmat[0][2] = 0.f;
  Gmat[1][2] = 0.f;
  Gmat[2][2] = 0.f;

  // dTx
  Gmat[0][3] =  ctheta * cpsi - stheta * sphi * spsi;
  Gmat[1][3] =  ctheta * spsi;
  Gmat[2][3] = -stheta * cpsi + ctheta * sphi * spsi;

  // dTy
  Gmat[0][4] = -ctheta * spsi + stheta * sphi * cpsi;
  Gmat[1][4] =  cphi * cpsi;
  Gmat[2][4] =  stheta * spsi + ctheta * sphi * cpsi;

  // dTz
  Gmat[0][5] =  stheta * cphi;
  Gmat[1][5] = -sphi;
  Gmat[2][5] =  ctheta * cphi;
}

void guidance_indi_set_wls_settings(struct WLS_t *wls, const struct FloatQuat *att, const float heading_sp)
{
  struct FloatEulers euler_yxz_ref = { 0 };
#if GUIDANCE_INDI_RC_SWITCH_EULER
  euler_yxz_ref.phi = (radio_control_get(RADIO_PITCH) / MAX_PPRZ) * guidance_indi_max_bank;
  euler_yxz_ref.theta = (radio_control_get(RADIO_ROLL) / MAX_PPRZ) * guidance_indi_max_bank;
#endif
  euler_yxz_ref.psi = heading_sp;

  struct FloatEulers euler_yxz;
  float_eulers_of_quat_yxz(&euler_yxz, att);

  // Set lower limits
  wls->u_min[0] = -guidance_indi_max_bank - euler_yxz.phi;          // phi
  wls->u_min[1] = -guidance_indi_max_bank - euler_yxz.theta;        // theta
  wls->u_min[2] = -M_PI - euler_yxz.psi;                            // psi
  wls->u_min[3] = -guidance_indi_max_h_thrust - stab_thrust_filt.x; // Tx
  wls->u_min[4] = -guidance_indi_max_h_thrust - stab_thrust_filt.y; // Ty
  wls->u_min[5] = -guidance_indi_max_v_thrust - stab_thrust_filt.z; // Tz

  // Set lower limits limits
  wls->u_max[0] = guidance_indi_max_bank - euler_yxz.phi;          // phi
  wls->u_max[1] = guidance_indi_max_bank - euler_yxz.theta;        // theta
  wls->u_max[2] = M_PI - euler_yxz.psi;                            // psi
  wls->u_max[3] = guidance_indi_max_h_thrust - stab_thrust_filt.x; // Tx
  wls->u_max[4] = guidance_indi_max_h_thrust - stab_thrust_filt.y; // Ty
  wls->u_max[5] = 9.81f * GUIDANCE_INDI_MASS - stab_thrust_filt.z; // Tz

  // Set prefered states
  wls->u_pref[0] = euler_yxz_ref.phi - euler_yxz.phi;     // prefered delta phi
  wls->u_pref[1] = euler_yxz_ref.theta - euler_yxz.theta; // prefered delta theta
  wls->u_pref[2] = euler_yxz_ref.psi - euler_yxz.psi;     // prefered delta psi
  wls->u_pref[3] = stab_thrust_filt.x;                    // prefered delta Tx
  wls->u_pref[4] = stab_thrust_filt.y;                    // prefered delta Ty
  wls->u_pref[5] = stab_thrust_filt.z;                    // prefered delta Tz
}

