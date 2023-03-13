/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/nav/nav_takeoff_and_landing.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Navigation routines for takeoff and landing
 * Basic procedures for rotorcraft and fixed-wing
 */

#ifndef NAV_TAKEOFF_AND_LANDING_H
#define NAV_TAKEOFF_AND_LANDING_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

enum nav_takeoff_status {
  NAV_TAKEOFF_INIT,
  NAV_TAKEOFF_START_MOTOR,
  NAV_TAKEOFF_CLIMB,
  NAV_TAKEOFF_DONE
};

enum nav_landing_status {
  NAV_LANDING_INIT,
  NAV_LANDING_REACH_AF,
  NAV_LANDING_DESCENT,
  NAV_LANDING_FLARE,
  NAV_LANDING_DONE
};

/** Structure for takeoff
 */
struct nav_takeoff {
  enum nav_takeoff_status status; ///< current step
  struct EnuCoor_f climb_pos;     ///< climb point
  struct EnuCoor_f start_pos;     ///< start position
  uint8_t climb_id;               ///< climb waypoint id
  bool timeout;                   ///< true if status should be set to init
};

/** Structure for landing
 */
struct nav_landing {
  enum nav_landing_status status; ///< current step
  struct EnuCoor_f td_pos;        ///< touch down point
  struct EnuCoor_f af_pos;        ///< start of descent point
  float radius;                   ///< circle radius for fixedwing landing
  uint8_t td_id;                  ///< touch down wp id
  uint8_t af_id;                  ///< start of descent wp id
  bool timeout;                   ///< true if status should be set to init
};

/** Takeoff direction in range [0-360] (deg)
 *  set to flight plan QFU by default
 */
extern float nav_takeoff_direction;

/** Init function
 */
extern void nav_takeoff_and_landing_init(void);

/** Periodic timeout check function
 */
extern void nav_takeoff_and_landing_periodic(void);

/** Takeoff from a waypoint
 *
 * - fixedwing: set the climb direction
 * - rotorcraft: set the wp at current location and climb verticaly
 *
 * @param[in] wp_id waypoint ID
 * @return true until procedure is completed
 */
extern bool nav_takeoff_from_wp(uint8_t wp_id);

/** Takeoff from lat long location
 *
 * - fixedwing: set the climb direction
 * - rotorcraft: start from current location and climb verticaly (lat, lon have no effect)
 *
 * @param[in] lat takeoff latitude (deg)
 * @param[in] lon takeoff longitude (deg)
 * @return true until procedure is completed
 */
extern bool nav_takeoff_from_loc(float lat, float lon);

/** Takeoff from current location
 *
 * - fixedwing: climb direction is specified with setting
 * - rotorcraft: start from current location and climb verticaly
 *
 * @return true until procedure is completed
 */
extern bool nav_takeoff_from_here(void);

/** Land at waypoint location
 *
 * set touch down (TD) and start of descent (AF)
 * - rotorcraft: if TD and AF id are the same, vertical landing
 *
 * @param[in] td_id waypoint ID for touch down point
 * @param[in] af_if waypoint ID for start of descent
 * @param[in] radius circle radius for final turn (if positive: turn right, if negative: turn left) (m)
 * @return true until procedure is completed
 */
extern bool nav_land_at_wp(uint8_t td_id, uint8_t af_id, float radius);

/** Land at lat long location
 *
 * @param[in] td_alt touch down altitude above ground ref (m)
 * @param[in] lat landing latitude (deg)
 * @param[in] lon landing longitude (deg)
 * @param[in] dir direction approach direction ([0-360], deg)
 * @param[in] dist distance to start descent (m)
 * @param[in] radius circle radius for final turn (if positive: turn right, if negative: turn left) (m)
 * @return true until procedure is completed
 */
extern bool nav_land_at_loc(float td_alt, float lat, float lon, float dir, float dist, float radius);

/** Land at current location
 *
 * emergency landing
 * - fixedwing: circle down around current location
 * - rotorcraft: vertical landing at current location
 *
 * @param[in] td_alt touch down altitude above ground ref (m)
 * @param[in] radius circle radius for final turn (if positive: turn right, if negative: turn left) (m)
 * @return true until procedure is completed
 */
extern bool nav_land_here(float td_alt, float radius);

#endif  // NAV_TAKEOFF_AND_LANDING_H

