/*
 * Copyright (C) 2026 Fabien-B <fabien-B@github.com>
 *                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/search_and_rescue/scout_detector.h"
 * Scoutector driver
 * Initially made for IMAV 2026 !
 */

#ifndef SCOUT_DETECTOR_H
#define SCOUT_DETECTOR_H

#include "std.h"
#include "modules/search_and_rescue/search_map.h"

typedef struct {
  float snr;
  float lit;
} scoutector_t;

extern void scout_detector_init(void);
extern void scout_detector_report(void);
extern void scout_detector_sim(void);

extern scoutector_t scout_data;

extern struct search_map_t scout_map;

#endif  // SCOUT_DETECTOR_H

