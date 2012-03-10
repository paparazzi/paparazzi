/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
#ifndef STABILIZATION_ATTITUDE_INT_REF_QUAT_INT_H
#define STABILIZATION_ATTITUDE_INT_REF_QUAT_INT_H

#include "firmwares/rotorcraft/stabilization.h"

#include "subsystems/radio_control.h"
#include "math/pprz_algebra_float.h"

#include "stabilization_attitude_ref_int.h"

#include "subsystems/ahrs.h"

#define ROLL_COEF   (STABILIZATION_ATTITUDE_SP_MAX_PHI   / MAX_PPRZ)
// FIXME: unused, what was it supposed to be?
//#define ROLL_COEF_H (STABILIZATION_ATTITUDE_SP_MAX_P_H   / MAX_PPRZ)
#define PITCH_COEF  (STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ)
// FIXME: what is this supposed to be??
#define YAW_COEF    (STABILIZATION_ATTITUDE_SP_MAX_PSI   / MAX_PPRZ)

#define ROLL_COEF_RATE  (-STABILIZATION_ATTITUDE_SP_MAX_P / MAX_PPRZ)
#define PITCH_COEF_RATE ( STABILIZATION_ATTITUDE_SP_MAX_Q / MAX_PPRZ)
#define YAW_COEF_RATE   ( STABILIZATION_ATTITUDE_SP_MAX_R / MAX_PPRZ)

#define DEADBAND_EXCEEDED(VARIABLE, VALUE) ((VARIABLE > VALUE) || (VARIABLE < -VALUE))
#define APPLY_DEADBAND(VARIABLE, VALUE) (DEADBAND_EXCEEDED(VARIABLE, VALUE) ? VARIABLE : 0.0)

#define ROLL_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_ROLL] >  STABILIZATION_ATTITUDE_DEADBAND_A || \
   radio_control.values[RADIO_ROLL] < -STABILIZATION_ATTITUDE_DEADBAND_A)
#define PITCH_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_PITCH] >  STABILIZATION_ATTITUDE_DEADBAND_E || \
   radio_control.values[RADIO_PITCH] < -STABILIZATION_ATTITUDE_DEADBAND_E)

#define STABILIZATION_ATTITUDE_RESET_PSI_REF(_sp) {}

static inline void update_quat_from_eulers(struct Int32Quat *quat, struct Int32Eulers *eulers) {
  struct Int32RMat rmat;

#ifdef STICKS_RMAT312
  INT32_RMAT_OF_EULERS_312(rmat, *eulers);
#else
  INT32_RMAT_OF_EULERS_321(rmat, *eulers);
#endif
  INT32_QUAT_OF_RMAT(*quat, rmat);
  INT32_QUAT_WRAP_SHORTEST(*quat);
}


static inline void stabilization_attitude_read_rc_setpoint(bool_t in_flight) {

  stab_att_sp_euler.phi = ((int32_t)-radio_control.values[RADIO_ROLL]  * SP_MAX_PHI / MAX_PPRZ);
  stab_att_sp_euler.theta = ((int32_t) radio_control.values[RADIO_PITCH] * SP_MAX_THETA / MAX_PPRZ);

  if (in_flight) {
    if (YAW_DEADBAND_EXCEEDED()) {
      stab_att_sp_euler.psi += ((int32_t)-radio_control.values[RADIO_YAW] * SP_MAX_R / MAX_PPRZ / RC_UPDATE_FREQ);
      INT32_ANGLE_NORMALIZE(stab_att_sp_euler.psi);
    }
  }
  else { /* if not flying, use current yaw as setpoint */
    stab_att_sp_euler.psi = ahrs.ltp_to_body_euler.psi;
  }

  /* update quaternion setpoint from euler setpoint */
  update_quat_from_eulers(&stab_att_sp_quat, &stab_att_sp_euler);

}

void stabilization_attitude_ref_enter(void);

#endif /* STABILIZATION_ATTITUDE_INT_REF_QUAT_INT_H */
