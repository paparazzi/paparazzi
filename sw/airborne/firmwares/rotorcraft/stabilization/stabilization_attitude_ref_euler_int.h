/*
 * $Id$
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

#ifndef STABILIZATION_ATTITUDE_REF_EULER_INT_H
#define STABILIZATION_ATTITUDE_REF_EULER_INT_H

#include "stabilization_attitude_ref_int.h"

#define STABILIZATION_ATTITUDE_ADD_SP(_add_sp) {	\
    EULERS_ADD(stab_att_sp_euler,_add_sp);          \
    ANGLE_REF_NORMALIZE(stab_att_sp_euler.psi);     \
  }

#define STABILIZATION_ATTITUDE_RESET_PSI_REF(_sp) {                     \
    _sp.psi = ahrs.ltp_to_body_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC); \
    stab_att_ref_euler.psi = _sp.psi;                                   \
    stab_att_ref_rate.r = 0;                                            \
  }


#endif /* STABILIZATION_ATTITUDE_REF_EULER_INT_H */
