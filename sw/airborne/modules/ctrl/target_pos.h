/*
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/ctrl/target_pos.h"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Create a target position derived from and RTK gps or TARGET_POS message
 */

#ifndef TARGET_POS_H
#define TARGET_POS_H

#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"

struct target_pos_t {
  bool valid;               ///< If the data of the target position is valid
  uint32_t recv_time;       ///< Time of when the target position message was received [msec]
  uint32_t tow;             ///< Time of week of the target position measurement
  struct LlaCoor_i lla;     ///< Lat, lon and altitude position of the target
  float ground_speed;       ///< Ground speed of the target [m/s]
  float course;             ///< Ground course of the target [deg]
  float heading;            ///< Heading of the target [deg]
  float climb;              ///< Climb speed, z-up [m/s]
};

struct target_offset_t {
  float heading;            ///< Target offset heading
  float distance;           ///< Target offset distance
  float height;             ///< Target offset height
};

struct target_t {
  struct target_pos_t pos;                  ///< The target position message
  struct target_offset_t offset;            ///< The target offset relative to ground heading
  uint32_t target_pos_timeout;              ///< Ground target position message timeout [msec]
  uint32_t rtk_timeout;                     ///< RTK message timeout [msec]
  bool integrate_xy;                        ///< Enable integration of the position in X-Y (North/East) frame
  bool integrate_z;                         ///< Enable integration of the position in Z (Up) frame
  struct LlaCoor_i gps_lla;                 ///< GPS LLA position
};

extern struct target_t target;
extern void target_pos_init(void);
extern void target_parse_target_pos(uint8_t *buf);
extern bool target_get_pos(struct NedCoor_f *pos, float *heading);
extern bool target_get_vel(struct NedCoor_f *vel);
extern bool target_pos_set_current_offset(float unk);

#endif
