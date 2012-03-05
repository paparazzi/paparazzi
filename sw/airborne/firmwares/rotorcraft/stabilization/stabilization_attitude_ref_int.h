/*
 * $Id$
 *
 * Copyright (C) 2008-2010 The Paparazzi Team
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
#ifndef STABILISATION_ATTITUDE_REF_INT_H
#define STABILISATION_ATTITUDE_REF_INT_H

#include "generated/airframe.h"
#include "math/pprz_algebra_int.h"

// FIXME: move stabilization_attitude_read_rc_ref somewere else, so we don't need these includes here???
#include "subsystems/radio_control.h"
#include "subsystems/ahrs.h"

/*
 * CAUTION! stabilization euler has the euler angles with REF_ANGLE_FRAC, but stab quat with INT32_ANGLE_FRAC
 */
extern struct Int32Eulers stab_att_sp_euler;
extern struct Int32Quat   stab_att_sp_quat;   ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_ref_euler;
extern struct Int32Quat   stab_att_ref_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Rates  stab_att_ref_rate;  ///< with #REF_RATE_FRAC
extern struct Int32Rates  stab_att_ref_accel; ///< with #REF_ACCEL_FRAC

struct Int32RefModel {
  struct Int32Rates omega;
  struct Int32Rates zeta;
};

extern struct Int32RefModel stab_att_ref_model;

#define REF_ACCEL_FRAC 12
#define REF_RATE_FRAC  16
#define REF_ANGLE_FRAC 20

#define REF_ANGLE_PI      BFP_OF_REAL(3.1415926535897932384626433832795029, REF_ANGLE_FRAC)
#define REF_ANGLE_TWO_PI  BFP_OF_REAL(2.*3.1415926535897932384626433832795029, REF_ANGLE_FRAC)
#define ANGLE_REF_NORMALIZE(_a) {                       \
    while (_a >  REF_ANGLE_PI)  _a -= REF_ANGLE_TWO_PI; \
    while (_a < -REF_ANGLE_PI)  _a += REF_ANGLE_TWO_PI; \
  }

#define SP_MAX_PHI   (int32_t)ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_PHI)
#define SP_MAX_THETA (int32_t)ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_THETA)
#define SP_MAX_R     (int32_t)ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_R)


#define RC_UPDATE_FREQ 40

#define YAW_DEADBAND_EXCEEDED()                                         \
  (radio_control.values[RADIO_YAW] >  STABILIZATION_ATTITUDE_DEADBAND_R || \
   radio_control.values[RADIO_YAW] < -STABILIZATION_ATTITUDE_DEADBAND_R)

static inline void stabilization_attitude_read_rc_ref(struct Int32Eulers *sp, bool_t in_flight) {

  sp->phi = ((int32_t)-radio_control.values[RADIO_ROLL]  * SP_MAX_PHI / MAX_PPRZ)
    << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);

  sp->theta = ((int32_t) radio_control.values[RADIO_PITCH] * SP_MAX_THETA / MAX_PPRZ)
    << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);

  if (in_flight) {
    if (YAW_DEADBAND_EXCEEDED()) {
      sp->psi += ((int32_t)-radio_control.values[RADIO_YAW] * SP_MAX_R / MAX_PPRZ / RC_UPDATE_FREQ)
        << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
      ANGLE_REF_NORMALIZE(sp->psi);
    }
  }
  else { /* if not flying, use current yaw as setpoint */
    sp->psi = (ahrs.ltp_to_body_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
  }

}

#endif /* STABILISATION_ATTITUDE_REF_INT_H */
