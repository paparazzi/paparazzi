/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 *
 */

/** @file modules/optical_flow/px4flow.h
 *  @brief driver for the optical flow sensor PX4FLOW
 *
 *  Sensor from the PIXHAWK project
 */

#ifndef PX4FLOW_H
#define PX4FLOW_H

#include "std.h"

/** Mavlink optical flow structure.
 *  Using MAVLINK v1.0 generated code:
 *   Message ID 100
 *   Fields are ordered to guarentee alignment
 */
struct mavlink_optical_flow {
  uint64_t time_usec;     ///< Timestamp (UNIX)
  float flow_comp_m_x;    ///< Flow in meters in x-sensor direction, angular-speed compensated
  float flow_comp_m_y;    ///< Flow in meters in y-sensor direction, angular-speed compensated
  float ground_distance;  ///< Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
  int16_t flow_x;         ///< Flow in pixels in x-sensor direction
  int16_t flow_y;         ///< Flow in pixels in y-sensor direction
  uint8_t sensor_id;      ///< Sensor ID
  uint8_t quality;        ///< Optical flow quality / confidence. 0: bad, 255: maximum quality
};

extern struct mavlink_optical_flow optical_flow;
extern bool optical_flow_available;

extern void px4flow_init(void);
extern void px4flow_downlink(void);

#endif /* PX4FLOW_H */

