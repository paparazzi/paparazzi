/*
 * Copyright (C) Ewoud Smeur <ewoud_smeur@msn.com>
 * MAVLab Delft University of Technology
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

#ifndef STABILIZATION_INDI
#define STABILIZATION_INDI

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"

// Scaling for the control effectiveness to make it readible
#define INDI_G_SCALING 1000.0

extern float g1g2[INDI_OUTPUTS][INDI_NUM_ACT];
extern float actuator_state_filt_vect[INDI_NUM_ACT];
extern bool act_is_servo[INDI_NUM_ACT];

extern bool indi_use_adaptive;

extern float *Bwls[INDI_OUTPUTS];

extern float thrust_bx_eff;
extern float thrust_bx_act_dyn;
extern float actuator_thrust_bx_pprz;
extern float thrust_bx_state_filt;

extern float act_pref[INDI_NUM_ACT];

extern float indi_Wu[INDI_NUM_ACT];
struct Indi_gains {
  struct FloatRates att;
  struct FloatRates rate;
};

extern float stablization_indi_yaw_dist_limit;

extern struct Indi_gains indi_gains;
extern float stabilization_indi_filter_freq; //for setting handler

extern void stabilization_indi_init(void);
extern void stabilization_indi_enter(void);
extern void stabilization_indi_rate_run(bool in_flight, struct StabilizationSetpoint *rate_sp, struct ThrustSetpoint *thrust, int32_t *cmd);
extern void stabilization_indi_attitude_run(bool in_flight, struct StabilizationSetpoint *att_sp, struct ThrustSetpoint *thrust, int32_t *cmd);
extern void stabilization_indi_set_wls_settings(void);
extern void stabilization_indi_update_filt_freq(float freq); // setting handler

#if !STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
#include "math/wls/wls_alloc.h"
extern struct WLS_t wls_stab_p;
#endif
#endif /* STABILIZATION_INDI */

