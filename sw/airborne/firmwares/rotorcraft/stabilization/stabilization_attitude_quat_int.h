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

#ifndef STABILIZATION_ATTITUDE_QUAT_INT_H
#define STABILIZATION_ATTITUDE_QUAT_INT_H

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"

#include "math/pprz_algebra_int.h"

extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC

extern struct AttRefQuatInt att_ref_quat_i;

/* settings handlers for ref model params */
#define stabilization_attitude_quat_int_SetOmegaP(_val) {   \
    attitude_ref_quat_int_set_omega_p(&att_ref_quat_i, _val);   \
  }
#define stabilization_attitude_quat_int_SetOmegaQ(_val) {   \
    attitude_ref_quat_int_set_omega_q(&att_ref_quat_i, _val);   \
  }
#define stabilization_attitude_quat_int_SetOmegaR(_val) {   \
    attitude_ref_quat_int_set_omega_r(&att_ref_quat_i, _val);   \
  }

#define stabilization_attitude_quat_int_SetZetaP(_val) {    \
    attitude_ref_quat_int_set_zeta_p(&att_ref_quat_i, _val);    \
  }
#define stabilization_attitude_quat_int_SetZetaQ(_val) {    \
    attitude_ref_quat_int_set_zeta_q(&att_ref_quat_i, _val);    \
  }
#define stabilization_attitude_quat_int_SetZetaR(_val) {    \
    attitude_ref_quat_int_set_zeta_r(&att_ref_quat_i, _val);    \
  }

#endif /* STABILIZATION_ATTITUDE_QUAT_INT_H */
