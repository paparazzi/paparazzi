/*
 * Copyright (C) Ewoud Smeur <ewoud_smeur@msn.com>
 * MAVLab Delft University of Technology
 *
 * This control algorithm is Incremental Nonlinear Dynamic Inversion (INDI)
 *
 * This is a simplified implementation of the (soon to be) publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
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

/** @file stabilization_attitude_quat_indi.h
 * This is the header file of the corresponding c file
 */

#ifndef STABILIZATION_ATTITUDE_QUAT_INDI_H
#define STABILIZATION_ATTITUDE_QUAT_INDI_H

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"

struct ReferenceSystem {
  float err_p;
  float err_q;
  float err_r;
  float rate_p;
  float rate_q;
  float rate_r;
};

extern struct FloatRates g1;
extern float g2;
extern struct ReferenceSystem reference_acceleration;

extern struct FloatRates filtered_rate;
extern struct FloatRates filtered_rate_deriv;
extern struct FloatRates filtered_rate_2deriv;
extern struct FloatRates angular_accel_ref;
extern struct FloatRates indi_u;
extern struct FloatRates indi_du;
extern struct FloatRates u_act_dyn;
extern struct FloatRates u_in;
extern struct FloatRates udot;
extern struct FloatRates udotdot;

extern struct FloatRates g_est;

void stabilization_indi_second_order_filter(struct FloatRates *input, struct FloatRates *filter_ddx, struct FloatRates *filter_dx, struct FloatRates *filter_x, float omega, float zeta, float omega_r);
void lms_estimation(void);

#endif /* STABILIZATION_ATTITUDE_QUAT_INT_H */

