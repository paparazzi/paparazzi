/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/decawave/trilateration.h"
 * @author Gautier Hattenberger
 * Trilateration algorithm
 * https://en.wikipedia.org/wiki/Trilateration
 */

#ifndef TRILATERATION_H
#define TRILATERATION_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

/** Anchor structure */
struct Anchor {
  float distance;       ///< last measured distance
  float time;           ///< time of the last received data
  struct EnuCoor_f pos; ///< position of the anchor
  uint16_t id;          ///< anchor ID
  bool updated;         ///< new data available
};

/** Init internal trilateration structures
 *
 * @param[in] anchors array of anchors with their location
 */
extern int trilateration_init(struct Anchor *anchors);

/** Compute trilateration based on the latest measurments
 *
 * @param[in] anchors array of anchors with updated distance measurements
 * @param[out] pos computed position
 * @return error status (0 for valid position)
 */
extern int trilateration_compute(struct Anchor *anchors, struct EnuCoor_f *pos);

#endif


