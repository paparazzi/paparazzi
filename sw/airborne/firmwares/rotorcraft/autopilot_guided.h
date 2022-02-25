/*
 * Copyright (C) 2016 The Paparazzi Team
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
 * @file firmwares/rotorcraft/autopilot_guided.h
 *
 * Autopilot guided mode interface.
 *
 */

#ifndef AUTOPILOT_GUIDED_H
#define AUTOPILOT_GUIDED_H

#include "std.h"

/** Set position and heading setpoints in GUIDED mode.
 * @param x North position (local NED frame) in meters.
 * @param y East position (local NED frame) in meters.
 * @param z Down position (local NED frame) in meters.
 * @param heading Setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern bool autopilot_guided_goto_ned(float x, float y, float z, float heading);

/** Set position and heading setpoints wrt. current position in GUIDED mode.
 * @param dx Offset relative to current north position (local NED frame) in meters.
 * @param dy Offset relative to current east position (local NED frame) in meters.
 * @param dz Offset relative to current down position (local NED frame) in meters.
 * @param dyaw Offset relative to current heading setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern bool autopilot_guided_goto_ned_relative(float dx, float dy, float dz, float dyaw);

/** Set position and heading setpoints wrt. current position AND heading in GUIDED mode.
 * @param dx relative position (body frame, forward) in meters.
 * @param dy relative position (body frame, right) in meters.
 * @param dz relative position (body frame, down) in meters.
 * @param dyaw Offset relative to current heading setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern bool autopilot_guided_goto_body_relative(float dx, float dy, float dz, float dyaw);

/** Set velocity and heading setpoints in GUIDED mode.
 * @param vx North velocity (local NED frame) in meters/sec.
 * @param vy East velocity (local NED frame) in meters/sec.
 * @param vz Down velocity (local NED frame) in meters/sec.
 * @param heading Setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern bool autopilot_guided_move_ned(float vx, float vy, float vz, float heading);

/** Set guided setpoints using flag mask in GUIDED mode.
 * @param flags Bits 0-3 are used to determine the axis system to be used.
 * If bits 0 and 1 are clear then the coordinates are set in absolute NE coordinates.
 * If bit 1 is set bit 0 is ignored.
 * Bits 5-7 define whether the setpoints should be used as position or velocity.
 * Bit flags are defined as follows:
      bit 0: x,y as offset coordinates
      bit 1: x,y in body coordinates
      bit 2: z as offset coordinates
      bit 3: yaw as offset coordinates
      bit 4: free
      bit 5: x,y as vel
      bit 6: z as vel
      bit 7: yaw as rate
 * @param x North position/velocity in meters or meters/sec.
 * @param y East position/velocity in meters or meters/sec.
 * @param z Down position/velocity in meters or meters/sec.
 * @param yaw Heading or heading rate setpoint in radians or radians/sec.
 */
extern void autopilot_guided_update(uint8_t flags, float x, float y, float z, float yaw);

/** Parse GUIDED_SETPOINT_NED messages from datalink
 */
extern void autopilot_guided_parse_GUIDED(uint8_t *buf);

/** Bitmask for setting the flags attribute in autopilot_guided_update function
 *  See function description for more details
 */
#define GUIDED_FLAG_XY_OFFSET   (1<<0)
#define GUIDED_FLAG_XY_BODY     (1<<1)
#define GUIDED_FLAG_Z_OFFSET    (1<<2)
#define GUIDED_FLAG_YAW_OFFSET  (1<<3)
#define GUIDED_FLAG_XY_VEL      (1<<5)
#define GUIDED_FLAG_Z_VEL       (1<<6)
#define GUIDED_FLAG_YAW_RATE    (1<<7)

#endif /* AUTOPILOT_GUIDED_H */

