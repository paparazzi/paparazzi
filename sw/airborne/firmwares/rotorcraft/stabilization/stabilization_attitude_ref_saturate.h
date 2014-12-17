/*
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

/** @file firmwares/rotorcraft/stabilization/stabilization_attitude_ref_saturate.h
 *  Common rotorcraft attitude reference saturation include.
 */

#ifndef STABILIZATION_ATTITUDE_REF_SATURATE_H
#define STABILIZATION_ATTITUDE_REF_SATURATE_H

#define SATURATE_SPEED_TRIM_ACCEL() {                   \
    if (stab_att_ref_rate.p >= REF_RATE_MAX_P) {    \
      stab_att_ref_rate.p = REF_RATE_MAX_P;             \
      if (stab_att_ref_accel.p > 0)                     \
        stab_att_ref_accel.p = 0;                       \
    }                                                   \
    else if (stab_att_ref_rate.p <= -REF_RATE_MAX_P) {  \
      stab_att_ref_rate.p = -REF_RATE_MAX_P;            \
      if (stab_att_ref_accel.p < 0)                     \
        stab_att_ref_accel.p = 0;                       \
    }                                                   \
    if (stab_att_ref_rate.q >= REF_RATE_MAX_Q) {    \
      stab_att_ref_rate.q = REF_RATE_MAX_Q;             \
      if (stab_att_ref_accel.q > 0)                     \
        stab_att_ref_accel.q = 0;                       \
    }                                                   \
    else if (stab_att_ref_rate.q <= -REF_RATE_MAX_Q) {  \
      stab_att_ref_rate.q = -REF_RATE_MAX_Q;            \
      if (stab_att_ref_accel.q < 0)                     \
        stab_att_ref_accel.q = 0;                       \
    }                                                   \
    if (stab_att_ref_rate.r >= REF_RATE_MAX_R) {    \
      stab_att_ref_rate.r = REF_RATE_MAX_R;             \
      if (stab_att_ref_accel.r > 0)                     \
        stab_att_ref_accel.r = 0;                       \
    }                                                   \
    else if (stab_att_ref_rate.r <= -REF_RATE_MAX_R) {  \
      stab_att_ref_rate.r = -REF_RATE_MAX_R;            \
      if (stab_att_ref_accel.r < 0)                     \
        stab_att_ref_accel.r = 0;                       \
    }                                                   \
  }


#endif /* STABILIZATION_ATTITUDE_REF_SATURATE_H */
