/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 * @file subsystems/ins/ins_int.h
 *
 * INS for rotorcrafts combining vertical and horizontal filters.
 *
 */

#ifndef INS_INT_H
#define INS_INT_H

#include "subsystems/ins.h"
#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"

#if USE_SONAR
#include "filters/median_filter.h"
#endif

/** Ins implementation state (fixed point) */
struct InsInt {
  struct LtpDef_i  ltp_def;
  bool_t           ltp_initialized;

  /* output LTP NED */
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;

  /* baro */
  float baro_z;  ///< z-position calculated from baro in meters (z-down)
  float qfe;
  bool_t baro_initialized;

#if USE_SONAR
  bool_t  update_on_agl; /* use sonar to update agl if available */
  int32_t sonar_alt;
  int32_t sonar_offset;
  struct MedianFilterInt sonar_median;
#endif
};

/** global INS state */
extern struct InsInt ins_impl;

#endif /* INS_INT_H */
