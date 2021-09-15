/*
 * Copyright (C) 2013 Gautier Hattenberger
 * 2016 Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
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

// control variables
extern bool px4flow_update_agl;
extern bool px4flow_compensate_rotation;
extern float px4flow_stddev;

/** Mavlink optical flow structure.
 *  Using MAVLINK v1.0 generated code:
 *  Message ID 100
 *  Fields are ordered to guarantee alignment
 */
struct mavlink_optical_flow {
  uint64_t time_usec;     ///< Timestamp (UNIX)
  float flow_comp_m_x;    ///< Flow in meters in x-sensor direction, angular-speed compensated [meters/sec]
  float flow_comp_m_y;    ///< Flow in meters in y-sensor direction, angular-speed compensated [meters/sec]
  float ground_distance;  ///< Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
  float distance;					///< Distance measured without compensation in meters
  int32_t flow_x;         ///< Flow in pixels in x-sensor direction
  int32_t flow_y;         ///< Flow in pixels in y-sensor direction
  uint8_t sensor_id;      ///< Sensor ID
  uint8_t quality;        ///< Optical flow quality / confidence. 0: bad, 255: maximum quality
};

/** Mavlink optical flow rad structure.
 *  Using MAVLINK v1.0 generated code:
 *  Message ID 106
 *  Fields are ordered to guarantee alignment
 */
struct mavlink_optical_flow_rad {
  // Timestamp (microseconds, synced to UNIX time or since system boot)
  uint64_t time_usec;
  // Sensor ID
	uint8_t sensor_id;
	// Integration time in microseconds.
	// Divide integrated_x and integrated_y by the integration time to obtain average flow.
	uint32_t integration_time_us;
	// Flow in radians around X axis
	// Sensor RH rotation about the X axis induces a positive flow.
	// Sensor linear motion along the positive Y axis induces a negative flow.
	float integrated_x;
	// Flow in radians around Y axis
	// Sensor RH rotation about the Y axis induces a positive flow.
	// Sensor linear motion along the positive X axis induces a positive flow.
	float integrated_y;
	// RH rotation around X axis (rad)
	float integrated_xgyro;
	// RH rotation around Y axis (rad)
	float integrated_ygyro;
	// RH rotation around Z axis (rad)
	float integrated_zgyro;
	// Temperature * 100 in centi-degrees Celsius
	int16_t temperature;
	// Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
	uint8_t quality;
	// Time in microseconds since the distance was sampled.
	uint32_t time_delta_distance_us;
	// Distance to the center of the flow field in meters.
	// Positive value (including zero): distance known.
	// Negative value: Unknown distance.
	float distance;
};

struct mavlink_heartbeat {
  uint8_t type; // Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
  uint8_t autopilot; // Autopilot type / class. defined in MAV_AUTOPILOT ENUM
  uint8_t base_mode; // System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
  uint32_t custom_mode; //  A bitfield for use for autopilot-specific flags.
  uint8_t system_status; // System status flag, see MAV_STATE ENUM
  uint8_t mavlink_version; //_mavlink_version MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
};


extern struct mavlink_heartbeat heartbeat;
extern struct mavlink_optical_flow optical_flow;
extern struct mavlink_optical_flow_rad optical_flow_rad;

extern void px4flow_init(void);
extern void px4flow_downlink(void);

#endif /* PX4FLOW_H */

