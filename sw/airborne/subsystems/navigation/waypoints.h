/*
 * Copyright (C) 2015 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/navigation/waypoints.h
 *
 */

#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"

#define WP_FLAG_GLOBAL 0
#define WP_FLAG_ENU_I 1
#define WP_FLAG_ENU_F 2
#define WP_FLAG_LLA_I 3

struct Waypoint {
  uint8_t flags; ///< bitmask encoding valid representations and if local or global
  struct EnuCoor_i enu_i;  ///< with #INT32_POS_FRAC
  struct EnuCoor_f enu_f;
  struct LlaCoor_i lla;
};

extern const uint8_t nb_waypoint;
/** size == nb_waypoint, waypoint 0 is a dummy waypoint */
extern struct Waypoint waypoints[];

#define WaypointX(_wp)    waypoints[_wp].enu_f.x
#define WaypointY(_wp)    waypoints[_wp].enu_f.y
#define WaypointAlt(_wp)  waypoints[_wp].enu_f.z
#define Height(_h) (_h)

static inline bool_t nav_wp_is_global(uint8_t wp_id)
{
  if (wp_id < nb_waypoint) {
    return bit_is_set(waypoints[wp_id].flags, WP_FLAG_GLOBAL);
  }
  return FALSE;
}

static inline void waypoint_set_global_flag(uint8_t wp_id)
{
  if (wp_id < nb_waypoint) {
    SetBit(waypoints[wp_id].flags, WP_FLAG_GLOBAL);
  }
}

static inline void waypoint_clear_global_flag(uint8_t wp_id)
{
  if (wp_id < nb_waypoint) {
    ClearBit(waypoints[wp_id].flags, WP_FLAG_GLOBAL);
  }
}

extern void waypoints_init(void);

extern void nav_set_waypoint_enu_f(uint8_t wp_id, struct EnuCoor_f *enu);
extern void nav_set_waypoint_enu_i(uint8_t wp_id, struct EnuCoor_i *enu);
extern void nav_set_waypoint_xy_i(uint8_t wp_id, int32_t x, int32_t y);
extern void nav_move_waypoint_enu_i(uint8_t wp_id, struct EnuCoor_i *new_pos);
extern void nav_set_waypoint_lla(uint8_t wp_id, struct LlaCoor_i *lla);
extern void nav_move_waypoint_lla(uint8_t wp_id, struct LlaCoor_i *lla);
/** set waypoint latitude/longitude without updating altitude */
void nav_set_waypoint_latlon(uint8_t wp_id, struct LlaCoor_i *lla);

/** set waypoint to current location and altitude */
extern void nav_set_waypoint_here(uint8_t wp_id);

/** set waypoint to current horizontal location without modifying altitude */
extern void nav_set_waypoint_here_2d(uint8_t wp_id);

/** update global LLA coordinates from its ENU coordinates */
extern void nav_globalize_local_wp(uint8_t wp_id);

/** update local ENU coordinates from its LLA coordinates */
extern void nav_localize_global_wp(uint8_t wp_id);
/** update local ENU coordinates of global waypoints */
extern void nav_localize_global_waypoints(void);

/** Get LLA coordinates of waypoint.
 * If the waypoint does not have its global coordinates set,
 * the LLA representation is computed if the local origin is set.
 *
 * @param  wp_id waypoint id
 * @return pointer to waypoint LLA coordinates, NULL if invalid
 */
extern struct LlaCoor_i *nav_get_waypoint_lla(uint8_t wp_id);

#endif /* WAYPOINTS_H */
