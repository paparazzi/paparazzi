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
 * @file "modules/decawave/dw1000_arduino.h"
 * @author Gautier Hattenberger
 * Driver to get ranging data from Decawave DW1000 modules connected to Arduino
 */

#ifndef DW1000_ARDUINO_H
#define DW1000_ARDUINO_H

#include "std.h"

/** enable EKF filtering */
extern bool dw1000_use_ekf;

/** process and measurements noise */
extern float dw1000_ekf_q;
extern float dw1000_ekf_r_dist;
extern float dw1000_ekf_r_speed;
/**  settings handler */
extern void dw1000_arduino_update_ekf_q(float v);
extern void dw1000_arduino_update_ekf_r_dist(float v);
extern void dw1000_arduino_update_ekf_r_speed(float v);

extern void dw1000_arduino_init(void);
extern void dw1000_arduino_periodic(void);
extern void dw1000_arduino_report(void);
extern void dw1000_arduino_event(void);

/** Reset reference heading to current heading
 * AHRS/INS should be aligned before calling this function
 */
extern void dw1000_reset_heading_ref(void);

// when used as a GPS
#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_DW1000
#endif

#endif

