/*
 * Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file "modules/nav/nav_base.h"
 * @author 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Basic navigation structure and definition
 */

#ifndef NAV_BASE_H
#define NAV_BASE_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

/** Waypoint and route pattern
 */
struct NavGoto_t {
  struct EnuCoor_f from;    ///< start WP position
  struct EnuCoor_f to;      ///< end WP position
  float dist2_to_wp;        ///< squared distance to next waypoint
  float leg_progress;       ///< progress over leg
  float leg_length;         ///< leg length
};

/** Circle pattern
 */
struct NavCircle_t {
  struct EnuCoor_f center;  ///< center WP position
  float radius;             ///< radius in meters
  float qdr;                ///< qdr in radians
  float radians;            ///< incremental angular distance
};

/** Oval pattern
 */
enum oval_status { OR12, OC2, OR21, OC1 };
struct NavOval_t {
  enum oval_status status;  ///< oval status
  uint8_t count;            ///< number of laps
};

/** Basic Nav struct
 */
struct NavBase_t {
  struct NavGoto_t goto_wp;
  struct NavCircle_t circle;
  struct NavOval_t oval;
};

/** helper functions
 */
static inline float nav_circle_get_count(struct NavCircle_t *circle)
{
  return fabsf(circle->radians) / (2.f*M_PI);
}

static inline float nav_circle_qdr(struct NavCircle_t *circle)
{
  float qdr = DegOfRad(M_PI_2 - circle->qdr);
  NormCourse(qdr);
  return qdr;
}


#endif

