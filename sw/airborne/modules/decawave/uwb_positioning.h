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
 * @file "modules/decawave/uwb_positioning.h"
 * @author Gautier Hattenberger
 * UWB positioning from anchor measurements.
 */

#pragma once

#include "std.h"

/** enable EKF filtering */
extern bool uwb_positioning_use_ekf;

/** process and measurements noise */
extern float uwb_positioning_ekf_q;
extern float uwb_positioning_ekf_r_dist;
extern float uwb_positioning_ekf_r_speed;
/**  settings handler */
extern void uwb_positioning_update_ekf_q(float v);
extern void uwb_positioning_update_ekf_r_dist(float v);
extern void uwb_positioning_update_ekf_r_speed(float v);

extern void uwb_positioning_init(void);
extern void uwb_positioning_periodic(void);
extern void uwb_positioning_range_periodic(void);
extern void uwb_positioning_report(void);

/** Reset reference heading to current heading
 * AHRS/INS should be aligned before calling this function
 */
extern void uwb_positioning_reset_heading_ref(void);

// when used as a GPS
#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_UWB
#endif

void uwb_range(uint16_t id);
