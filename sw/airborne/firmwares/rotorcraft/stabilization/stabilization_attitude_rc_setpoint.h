/*
 * Copyright (C) 2012 Felix Ruess <felix.ruess@gmail.com>
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

/** @file stabilization_attitude_rc_setpoint.h
 *  Read an attitude setpoint from the RC.
 */

#ifndef STABILISATION_ATTITUDE_RC_SETPOINT_H
#define STABILISATION_ATTITUDE_RC_SETPOINT_H

#include "std.h"
#include "generated/airframe.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

#include "subsystems/radio_control.h"
#include "subsystems/ahrs.h"

#ifdef STABILISATION_ATTITUDE_TYPE_INT
#define SP_MAX_PHI     (int32_t)ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_PHI)
#define SP_MAX_THETA   (int32_t)ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_THETA)
#define SP_MAX_R       (int32_t)ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_R)
#endif // STABILISATION_ATTITUDE_TYPE_INT

#ifdef STABILISATION_ATTITUDE_TYPE_FLOAT
#define SP_MAX_PHI   STABILIZATION_ATTITUDE_SP_MAX_PHI
#define SP_MAX_THETA STABILIZATION_ATTITUDE_SP_MAX_THETA
#define SP_MAX_R     STABILIZATION_ATTITUDE_SP_MAX_R
#endif // STABILISATION_ATTITUDE_TYPE_FLOAT

#define RC_UPDATE_FREQ 40

#ifdef STABILIZATION_ATTITUDE_DEADBAND_A
#define ROLL_DEADBAND_EXCEEDED()                                        \
  (radio_control.values[RADIO_ROLL] >  STABILIZATION_ATTITUDE_DEADBAND_A || \
   radio_control.values[RADIO_ROLL] < -STABILIZATION_ATTITUDE_DEADBAND_A)
#else
#define ROLL_DEADBAND_EXCEEDED() (TRUE)
#endif /* STABILIZATION_ATTITUDE_DEADBAND_A */

#ifdef STABILIZATION_ATTITUDE_DEADBAND_E
#define PITCH_DEADBAND_EXCEEDED()                                       \
  (radio_control.values[RADIO_PITCH] >  STABILIZATION_ATTITUDE_DEADBAND_E || \
   radio_control.values[RADIO_PITCH] < -STABILIZATION_ATTITUDE_DEADBAND_E)
#else
#define PITCH_DEADBAND_EXCEEDED() (TRUE)
#endif /* STABILIZATION_ATTITUDE_DEADBAND_E */

#define YAW_DEADBAND_EXCEEDED()                                         \
  (radio_control.values[RADIO_YAW] >  STABILIZATION_ATTITUDE_DEADBAND_R || \
   radio_control.values[RADIO_YAW] < -STABILIZATION_ATTITUDE_DEADBAND_R)

static inline void stabilization_attitude_read_rc_setpoint_eulers(struct Int32Eulers *sp, bool_t in_flight) {

  sp->phi = ((int32_t) radio_control.values[RADIO_ROLL]  * SP_MAX_PHI / MAX_PPRZ);
  sp->theta = ((int32_t) radio_control.values[RADIO_PITCH] * SP_MAX_THETA / MAX_PPRZ);

  if (in_flight) {
    if (YAW_DEADBAND_EXCEEDED()) {
      sp->psi += ((int32_t) radio_control.values[RADIO_YAW] * SP_MAX_R / MAX_PPRZ / RC_UPDATE_FREQ);
      INT32_ANGLE_NORMALIZE(sp->psi);
    }
  }
  else { /* if not flying, use current yaw as setpoint */
    sp->psi = ahrs.ltp_to_body_euler.psi;
  }

}

static inline void stabilization_attitude_read_rc_roll_pitch_quat(struct FloatQuat* q) {
  q->qx = radio_control.values[RADIO_ROLL] * STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ / 2;
  q->qy = radio_control.values[RADIO_PITCH] * STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ / 2;
  q->qz = 0.0;

  /* normalize */
  float norm = sqrtf(1.0 + SQUARE(q->qx)+ SQUARE(q->qy));
  q->qi = 1.0 / norm;
  q->qx /= norm;
  q->qy /= norm;
}

#endif /* STABILISATION_ATTITUDE_RC_SETPOINT_H */
