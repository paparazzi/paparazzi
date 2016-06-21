/*
 * Copyright (C) Roland Meertens
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
 * @file "modules/pose_history/pose_history.h"
 * @author Roland Meertens
 * Ask this module for the pose the drone had closest to a given timestamp
 */

#ifndef POSE_HISTORY_H
#define POSE_HISTORY_H

#include "math/pprz_algebra_float.h"

struct pose_t {
  uint32_t timestamp;
  struct FloatEulers eulers;
  struct FloatRates rates;
};

extern void pose_init(void);
extern void pose_periodic(void);
extern struct pose_t get_rotation_at_timestamp(uint32_t timestamp);
#endif

