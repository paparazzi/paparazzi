/*
 * $Id: stabilization_attitude_ref_traj_euler.h 3796 2009-07-25 00:01:02Z poine $
 *
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

#ifndef STABILIZATION_ATTITUDE_FLOAT_REF_EULER_FLOAT_H
#define STABILIZATION_ATTITUDE_FLOAT_REF_EULER_FLOAT_H

#include "subsystems/radio_control.h"
#include "math/pprz_algebra_float.h"


#include "stabilization_attitude_ref_float.h"

/*
 * Radio Control
 */
#define SP_MAX_PHI   STABILIZATION_ATTITUDE_FLOAT_SP_MAX_PHI
#define SP_MAX_THETA STABILIZATION_ATTITUDE_FLOAT_SP_MAX_THETA
#define SP_MAX_R     STABILIZATION_ATTITUDE_FLOAT_SP_MAX_R





#define RC_UPDATE_FREQ 40.

#define YAW_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_YAW] >  STABILIZATION_ATTITUDE_FLOAT_DEADBAND_R || \
   radio_control.values[RADIO_YAW] < -STABILIZATION_ATTITUDE_FLOAT_DEADBAND_R)

#define STABILIZATION_ATTITUDE_FLOAT_READ_RC(_sp, _inflight) {          \
                                                                        \
    _sp.phi = (radio_control.values[RADIO_ROLL]  * SP_MAX_PHI / MAX_PPRZ); \
    _sp.theta = (radio_control.values[RADIO_PITCH] * SP_MAX_THETA / MAX_PPRZ); \
    if (_inflight) {                                                    \
      if (YAW_DEADBAND_EXCEEDED()) {                                    \
        _sp.psi +=                                                      \
          (radio_control.values[RADIO_YAW] * SP_MAX_R / MAX_PPRZ / RC_UPDATE_FREQ); \
        FLOAT_ANGLE_NORMALIZE(_sp.psi);                                 \
      }                                                                 \
    }                                                                   \
    else { /* if not flying, use current yaw as setpoint */             \
      _sp.psi = ANGLE_FLOAT_OF_BFP(ahrs.ltp_to_body_euler.psi);         \
    }                                                                   \
  }

#define STABILIZATION_ATTITUDE_FLOAT_ADD_SP(_add_sp) {  \
    struct FloatEulers add_sp_float;                    \
    EULERS_FLOAT_OF_BFP(add_sp_float, (_add_sp));		\
    EULERS_ADD(stabilization_att_sp,add_sp_float);		\
    FLOAT_ANGLE_NORMALIZE(stabilization_att_sp.psi);	\
  }



#endif /* STABILIZATION_ATTITUDE_FLOAT_REF_EULER_FLOAT_H */
