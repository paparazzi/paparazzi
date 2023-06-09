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

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"
#include "math/lsq_package/common/solveActiveSet.h"
#include <stdint.h>

#ifdef USE_CHIBIOS_RTOS
#include <ch.h>
#endif

// Scaling for the control effectiveness to make it readible
//#define INDI_G_SCALING (1000.0/9600.0)
#define INDI_G_SCALING 1000.0

extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC
extern float g1g2[INDI_OUTPUTS][INDI_NUM_ACT];
extern float actuator_state_filt_vect[INDI_NUM_ACT];
extern float indi_v[INDI_OUTPUTS];
extern float indi_u[INDI_NUM_ACT];
extern float indi_du[INDI_NUM_ACT];

extern bool indi_use_adaptive;
extern activeSetAlgoChoice indi_ctl_alloc_algo;
extern float indi_ctl_alloc_cond_bound;
extern float indi_ctl_alloc_theta;
extern float indi_max_cmd_scaler;
extern bool indi_ctl_alloc_warmstart;
extern uint16_t indi_ctl_alloc_imax;

extern float *Bwls[INDI_OUTPUTS];

struct Indi_gains {
  struct FloatRates att;
  struct FloatRates rate;
};
extern struct FloatRates rate_sp;
extern struct FloatRates rates_current;

extern struct Indi_gains indi_gains;

float inverse_thrust_transform(float T);
extern void stabilization_indi_init(void);
extern void stabilization_indi_enter(void);
extern void stabilization_indi_set_failsafe_setpoint(void);
extern void stabilization_indi_set_rpy_setpoint_i(struct Int32Eulers *rpy);
extern void stabilization_indi_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading);
extern void stabilization_indi_rate_run(struct FloatRates rate_ref, bool in_flight);
extern void stabilization_indi_attitude_run(struct Int32Quat quat_sp, bool in_flight);
extern void stabilization_indi_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn);

#endif /* STABILIZATION_INDI */

