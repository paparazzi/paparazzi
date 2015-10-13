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

struct IndiVariables {
  struct FloatRates filtered_rate;
  struct FloatRates filtered_rate_deriv;
  struct FloatRates filtered_rate_2deriv;
  struct FloatRates angular_accel_ref;
  struct FloatRates du;
  struct FloatRates u_act_dyn;
  struct FloatRates u_in;
  struct FloatRates u;
  struct FloatRates udot;
  struct FloatRates udotdot;
};

extern struct FloatRates g1;
extern float g2;
extern struct ReferenceSystem reference_acceleration;

extern struct FloatRates g_est;
extern bool_t use_adaptive_indi;

extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC
extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC

void stabilization_indi_second_order_filter(struct FloatRates *input, struct FloatRates *filter_ddx,
    struct FloatRates *filter_dx, struct FloatRates *filter_x, float omega, float zeta, float omega_r);
void lms_estimation(void);

#endif /* STABILIZATION_ATTITUDE_QUAT_INT_H */

