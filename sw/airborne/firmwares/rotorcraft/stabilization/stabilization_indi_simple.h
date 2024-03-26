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

/** @file stabilization_attitude_quat_indi_simple.h
 * Stabilization based on INDI for multicopters.
 * It supports both rate and attidue control.
 */

#ifndef STABILIZATION_INDI_SIMPLE_H
#define STABILIZATION_INDI_SIMPLE_H

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"
#include "filters/low_pass_filter.h"

struct Indi_gains {
  struct FloatRates att;
  struct FloatRates rate;
};

struct IndiEstimation {
  Butterworth2LowPass u[3];
  Butterworth2LowPass rate[3];
  float rate_d[3];
  float rate_dd[3];
  float u_d[3];
  float u_dd[3];
  struct FloatRates g1;
  float g2;
  float mu;
};

struct IndiVariables {
  float cutoff_r;
  struct FloatRates angular_accel_ref;
  struct FloatRates du;
  struct FloatRates u_in;
  struct FloatRates u_act_dyn;
  float rate_d[3];

  Butterworth2LowPass u[3];
  Butterworth2LowPass rate[3];
  struct FloatRates g1;
  float g2;
  struct FloatRates act_dyn;

  struct Indi_gains gains;

  bool adaptive;             ///< Enable adataptive estimation
  float max_rate;            ///< Maximum rate in rate control in rad/s
  float attitude_max_yaw_rate; ///< Maximum yaw rate in atttiude control in rad/s
  struct IndiEstimation est; ///< Estimation parameters for adaptive INDI
};


extern struct IndiVariables indi;
extern void stabilization_indi_init(void);
extern void stabilization_indi_enter(void);
extern void stabilization_indi_rate_run(bool in_flight, struct StabilizationSetpoint *rate_sp, struct ThrustSetpoint *thrust, int32_t *cmd);
extern void stabilization_indi_attitude_run(bool in_flight, struct StabilizationSetpoint *att_sp, struct ThrustSetpoint *thrust, int32_t *cmd);
extern void stabilization_indi_simple_reset_r_filter_cutoff(float new_cutoff);

#endif /* STABILIZATION_INDI_SIMPLE_H */

