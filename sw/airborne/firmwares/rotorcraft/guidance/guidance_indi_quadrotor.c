/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/rotorcraft/guidance/guidance_indi_quadrotor.c
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#include "filters/low_pass_filter.h"
#include "state.h"
#include "autopilot.h"
#include "generated/modules.h"

/**
 * @param Gmat array to write the matrix to [3x3]
 *
 * Calculate the matrix of partial derivatives of the roll, pitch and thrust.
 * w.r.t. the NED accelerations for ZYX eulers
 * ddx = G*[dtheta,dphi,dT]
 */
UNUSED static void guidance_indi_calcG_zyx(float Gmat[3][3], struct FloatEulers euler_zyx)
{
  float sphi = sinf(euler_zyx.phi);
  float cphi = cosf(euler_zyx.phi);
  float stheta = sinf(euler_zyx.theta);
  float ctheta = cosf(euler_zyx.theta);
  float spsi = sinf(euler_zyx.psi);
  float cpsi = cosf(euler_zyx.psi);
  // minus gravity is a guesstimate of the thrust force, thrust measurement would be better
  float T = -9.81f;

  Gmat[0][0] = (cphi * spsi - sphi * cpsi * stheta) * T;
  Gmat[1][0] = (-sphi * spsi * stheta - cpsi * cphi) * T;
  Gmat[2][0] = -ctheta * sphi * T;
  Gmat[0][1] = (cphi * cpsi * ctheta) * T;
  Gmat[1][1] = (cphi * spsi * ctheta) * T;
  Gmat[2][1] = -stheta * cphi * T;
  Gmat[0][2] = sphi * spsi + cphi * cpsi * stheta;
  Gmat[1][2] = cphi * spsi * stheta - cpsi * sphi;
  Gmat[2][2] = cphi * ctheta;
}

/**
 * @param Gmat array to write the matrix to [3x3]
 *
 * Calculate the matrix of partial derivatives of the pitch, roll and thrust.
 * w.r.t. the NED accelerations for YXZ eulers
 * ddx = G*[dtheta,dphi,dT]
 */
static void guidance_indi_calcG_yxz(float Gmat[3][3], struct FloatEulers euler_yxz)
{
  float sphi = sinf(euler_yxz.phi);
  float cphi = cosf(euler_yxz.phi);
  float stheta = sinf(euler_yxz.theta);
  float ctheta = cosf(euler_yxz.theta);
  // minus gravity is a guesstimate of the thrust force, thrust measurement would be better
  float T = -9.81f;

  Gmat[0][0] = ctheta * cphi * T;
  Gmat[1][0] = 0;
  Gmat[2][0] = -stheta * cphi * T;
  Gmat[0][1] = -stheta * sphi * T;
  Gmat[1][1] = -cphi * T;
  Gmat[2][1] = -ctheta * sphi * T;
  Gmat[0][2] = stheta * cphi;
  Gmat[1][2] = -sphi;
  Gmat[2][2] = ctheta * cphi;
}

/** Compute effectiveness matrix for guidance
 *
 * @param Gmat Dynamics matrix
 */
void guidance_indi_calcG(float Gmat[3][3], struct FloatEulers att) {
#ifdef GUIDANCE_INDI_CALC_G_ZYX
  guidance_indi_calcG_zyx(Gmat, att);
#else
  guidance_indi_calcG_yxz(Gmat, att); // default case
#endif
}

