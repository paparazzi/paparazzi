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

#define SATURATE_SPEED_TRIM_ACCEL(_ref) {         \
    if ((_ref).rate.p >= REF_RATE_MAX_P) {        \
      (_ref).rate.p = REF_RATE_MAX_P;             \
      if ((_ref).accel.p > 0)                     \
        (_ref).accel.p = 0;                       \
    }                                             \
    else if ((_ref).rate.p <= -REF_RATE_MAX_P) {  \
      (_ref).rate.p = -REF_RATE_MAX_P;            \
      if ((_ref).accel.p < 0)                     \
        (_ref).accel.p = 0;                       \
    }                                             \
    if ((_ref).rate.q >= REF_RATE_MAX_Q) {        \
      (_ref).rate.q = REF_RATE_MAX_Q;             \
      if ((_ref).accel.q > 0)                     \
        (_ref).accel.q = 0;                       \
    }                                             \
    else if ((_ref).rate.q <= -REF_RATE_MAX_Q) {  \
      (_ref).rate.q = -REF_RATE_MAX_Q;            \
      if ((_ref).accel.q < 0)                     \
        (_ref).accel.q = 0;                       \
    }                                             \
    if ((_ref).rate.r >= REF_RATE_MAX_R) {        \
      (_ref).rate.r = REF_RATE_MAX_R;             \
      if ((_ref).accel.r > 0)                     \
        (_ref).accel.r = 0;                       \
    }                                             \
    else if ((_ref).rate.r <= -REF_RATE_MAX_R) {  \
      (_ref).rate.r = -REF_RATE_MAX_R;            \
      if ((_ref).accel.r < 0)                     \
        (_ref).accel.r = 0;                       \
    }                                             \
  }


#endif /* STABILIZATION_ATTITUDE_REF_SATURATE_H */
