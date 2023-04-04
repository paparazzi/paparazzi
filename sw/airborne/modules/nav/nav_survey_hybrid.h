/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Based on OSAM poly survey
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/nav/nav_survey_hybrid.h
 *
 */

#ifndef NAV_SURVEY_HYBRID_H
#define NAV_SURVEY_HYBRID_H

#include "std.h"

struct SurveyHybrid {
  uint16_t sweep_nb_max;
  uint16_t sweep_back_nb_max;
  uint16_t sweep_nb;
  uint16_t sweep_back_nb;
  bool half_sweep_enabled;
};

extern struct SurveyHybrid survey_hybrid;

/** Init function
 */
extern void nav_survey_hybrid_init(void);

/**
 * Setup polygon survey.
 * @param start_wp     first waypoint/corner of the polygon
 * @param orientation  angle of scan lines in degrees (CCW, east)
 * @param size         number of waypoints/corners used to define the polygon
 * @param sweep        distance between scan lines
 * @param radius       turn radius (<0: automatic, radius = sweep/2; 0: no turns, use straight lines only; >0: fixed radius)
 */
extern void nav_survey_hybrid_setup_orientation(uint8_t start_wp, float orientation, uint8_t size, float sweep, float radius);

/**
 * Setup "dynamic" polygon survey with sweep orientation towards a waypoint.
 * Computes the sweep orientation angle from the line first-second WP.
 * @param start_wp  first waypoint/corner of the polygon
 * @param second_wp second waypoint towards which the sweep orientation is computed
 * @param size      number of waypoints/corners used to define the polygon
 * @param sweep     distance between scan lines, if zero uses Poly_Distance
 * @param radius       turn radius (<0: automatic, radius = sweep/2; 0: no turns, use straight lines only; >0: fixed radius)
 */
extern void nav_survey_hybrid_setup_towards(uint8_t start_wp, uint8_t second_wp, uint8_t size, float sweep, float radius);

/** Run polygon hybrid survey */
extern bool nav_survey_hybrid_run(void);

#endif
