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

/**
 * @file stabilization_attitude_quat_float.h
 *
 * Rotorcraft attitude stabilization in quaternion float version.
 */

#ifndef STABILIZATION_ATTITUDE_QUAT_FLOAT_H
#define STABILIZATION_ATTITUDE_QUAT_FLOAT_H

#include "generated/airframe.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_float.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_float.h"


#ifndef STABILIZATION_ATTITUDE_GAIN_NB
#define STABILIZATION_ATTITUDE_GAIN_NB 1
#endif

#ifndef STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT
#define STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT 0
#endif

extern struct FloatAttitudeGains stabilization_gains[];

extern struct FloatEulers stab_att_sp_euler;
extern struct FloatQuat   stab_att_sp_quat;

extern struct AttRefQuatFloat att_ref_quat_f;


void stabilization_attitude_gain_schedule(uint8_t idx);

/* settings handlers */
#define stabilization_attitude_quat_float_SetOmegaP(_val) {      \
    attitude_ref_quat_float_set_omega_p(&att_ref_quat_f, _val);  \
  }
#define stabilization_attitude_quat_float_SetOmegaQ(_val) {      \
    attitude_ref_quat_float_set_omega_q(&att_ref_quat_f, _val);  \
  }
#define stabilization_attitude_quat_float_SetOmegaR(_val) {      \
    attitude_ref_quat_float_set_omega_r(&att_ref_quat_f, _val);  \
  }

#endif /* STABILIZATION_ATTITUDE_QUAT_FLOAT_H */
