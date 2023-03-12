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
 * @file modules/nav/waypoints.h
 *
 */

#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#ifdef __cplusplus
extern "C" {
#endif

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

/* aliases for backwards compatibilty */
#define WaypointX(_wp)    waypoint_get_x(_wp)
#define WaypointY(_wp)    waypoint_get_y(_wp)
#define WaypointAlt(_wp)  waypoint_get_alt(_wp)
#define Height(_h) (_h)

extern void waypoints_init(void);

extern bool waypoint_is_global(uint8_t wp_id);
extern void waypoint_set_global_flag(uint8_t wp_id);
extern void waypoint_clear_global_flag(uint8_t wp_id);


/*
 * Get waypoint coordinates.
 */
/** Get X/East coordinate of waypoint in meters */
extern float waypoint_get_x(uint8_t wp_id);
/** Get Y/North coordinate of waypoint in meters */
extern float waypoint_get_y(uint8_t wp_id);
/** Get altitude of waypoint in meters (above reference) */
extern float waypoint_get_alt(uint8_t wp_id);
/** Get latitude of waypoint in deg */
extern float waypoint_get_lat_deg(uint8_t wp_id);
/** Get latitude of waypoint in rad */
extern float waypoint_get_lat_rad(uint8_t wp_id);
/** Get longitude of waypoint in deg */
extern float waypoint_get_lon_deg(uint8_t wp_id);
/** Get longitude of waypoint in rad */
extern float waypoint_get_lon_rad(uint8_t wp_id);

/** Get LLA coordinates of waypoint.
 * If the waypoint does not have its global coordinates set,
 * the LLA representation is computed if the local origin is set.
 *
 * @param  wp_id waypoint id
 * @return pointer to waypoint LLA coordinates, NULL if invalid
 */
extern struct LlaCoor_i *waypoint_get_lla(uint8_t wp_id);

/** Get ENU coordinates (float)
 * @param wp_id waypoint id
 * @return pointer to waypoint ENU (float) coordinates, NULL if invalid
 */
extern struct EnuCoor_f *waypoint_get_enu_f(uint8_t wp_id);

/** Get ENU coordinates (integer)
 * @param wp_id waypoint id
 * @return pointer to waypoint ENU (integer) coordinates, NULL if invalid
 */
extern struct EnuCoor_i *waypoint_get_enu_i(uint8_t wp_id);

/*
 * Set waypoint coordinates.
 */
/** Set local ENU waypoint coordinates */
extern void waypoint_set_enu(uint8_t wp_id, struct EnuCoor_f *enu);
/** Set altitude of waypoint in meters (above reference) */
extern void waypoint_set_alt(uint8_t wp_id, float alt);

/** set waypoint to current location and altitude */
extern void waypoint_set_here(uint8_t wp_id);
/** set waypoint to current horizontal location without modifying altitude */
extern void waypoint_set_here_2d(uint8_t wp_id);

/* functions to set fixedpoint representation directly */
extern void waypoint_set_enu_i(uint8_t wp_id, struct EnuCoor_i *enu);
extern void waypoint_set_xy_i(uint8_t wp_id, int32_t x, int32_t y);
extern void waypoint_set_alt_i(uint8_t wp_id, int32_t alt);
extern void waypoint_set_lla(uint8_t wp_id, struct LlaCoor_i *lla);

/** set waypoint latitude/longitude without updating altitude */
extern void waypoint_set_latlon(uint8_t wp_id, struct LlaCoor_i *lla);

/** copy one waypoint to another, this includes all flags from the source waypoint */
extern void waypoint_copy(uint8_t wp_dest, uint8_t wp_src);
extern void waypoint_position_copy(uint8_t wp_dest, uint8_t wp_src);


/*
 * Move waypoints.
 * Basically sets the coordinates and sends the WP_MOVED telemetry message as ack.
 */
extern void waypoint_move_here_2d(uint8_t wp_id);
extern void waypoint_move_enu_i(uint8_t wp_id, struct EnuCoor_i *new_pos);
extern void waypoint_move_xy_i(uint8_t wp_id, int32_t x, int32_t y);
extern void waypoint_move_lla(uint8_t wp_id, struct LlaCoor_i *lla);


/*
 * Global(LLA) / Local(ENU) conversions.
 */

/** update global LLA coordinates from its ENU coordinates */
extern void waypoint_globalize(uint8_t wp_id);

/** update local ENU coordinates from its LLA coordinates */
extern void waypoint_localize(uint8_t wp_id);
/** update local ENU coordinates of all global waypoints */
extern void waypoints_localize_all(void);

#ifdef __cplusplus
}
#endif

#endif /* WAYPOINTS_H */
