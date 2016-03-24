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
 * @file subsystems/navigation/waypoints.c
 *
 */

#include "subsystems/navigation/waypoints.h"
#include "state.h"
#include "subsystems/datalink/downlink.h"
#include "generated/flight_plan.h"

const uint8_t nb_waypoint = NB_WAYPOINT;
struct Waypoint waypoints[NB_WAYPOINT];

/** initialize global and local waypoints */
void waypoints_init(void)
{
  struct EnuCoor_f wp_tmp_float[NB_WAYPOINT] = WAYPOINTS_ENU;
  struct LlaCoor_i wp_tmp_lla_i[NB_WAYPOINT] = WAYPOINTS_LLA_WGS84;
  /* element in array is TRUE if absolute/global waypoint */
  bool is_global[NB_WAYPOINT] = WAYPOINTS_GLOBAL;
  uint8_t i = 0;
  for (i = 0; i < nb_waypoint; i++) {
    /* clear all flags */
    waypoints[i].flags = 0;
    /* init waypoint as global LLA or local ENU */
    if (is_global[i]) {
      waypoint_set_global_flag(i);
      waypoint_set_lla(i, &wp_tmp_lla_i[i]);
    } else {
      waypoint_set_enu(i, &wp_tmp_float[i]);
    }
  }
}

bool waypoint_is_global(uint8_t wp_id)
{
  if (wp_id < nb_waypoint) {
    return bit_is_set(waypoints[wp_id].flags, WP_FLAG_GLOBAL);
  }
  return false;
}

void waypoint_set_global_flag(uint8_t wp_id)
{
  if (wp_id < nb_waypoint) {
    SetBit(waypoints[wp_id].flags, WP_FLAG_GLOBAL);
  }
}

void waypoint_clear_global_flag(uint8_t wp_id)
{
  if (wp_id < nb_waypoint) {
    ClearBit(waypoints[wp_id].flags, WP_FLAG_GLOBAL);
  }
}

float waypoint_get_x(uint8_t wp_id)
{
  if (wp_id < nb_waypoint) {
    return waypoints[wp_id].enu_f.x;
  }
  return 0.f;
}

float waypoint_get_y(uint8_t wp_id)
{
  if (wp_id < nb_waypoint) {
    return waypoints[wp_id].enu_f.y;
  }
  return 0.f;
}

float waypoint_get_alt(uint8_t wp_id)
{
  if (wp_id < nb_waypoint) {
    return waypoints[wp_id].enu_f.z;
  }
  return 0.f;
}

void waypoint_set_enu_i(uint8_t wp_id, struct EnuCoor_i *enu)
{
  if (wp_id < nb_waypoint) {
    waypoints[wp_id].enu_i = *enu;
    SetBit(waypoints[wp_id].flags, WP_FLAG_ENU_I);
    ENU_FLOAT_OF_BFP(waypoints[wp_id].enu_f, waypoints[wp_id].enu_i);
    SetBit(waypoints[wp_id].flags, WP_FLAG_ENU_F);
    ClearBit(waypoints[wp_id].flags, WP_FLAG_LLA_I);
    waypoint_globalize(wp_id);
  }
}

void waypoint_set_enu(uint8_t wp_id, struct EnuCoor_f *enu)
{
  if (wp_id < nb_waypoint) {
    waypoints[wp_id].enu_f = *enu;
    SetBit(waypoints[wp_id].flags, WP_FLAG_ENU_I);
    ENU_BFP_OF_REAL(waypoints[wp_id].enu_i, waypoints[wp_id].enu_f);
    SetBit(waypoints[wp_id].flags, WP_FLAG_ENU_F);
    ClearBit(waypoints[wp_id].flags, WP_FLAG_LLA_I);
    waypoint_globalize(wp_id);
  }
}

void waypoint_move_enu_i(uint8_t wp_id, struct EnuCoor_i *new_pos)
{
  if (wp_id < nb_waypoint) {
    waypoint_set_enu_i(wp_id, new_pos);
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &(new_pos->x),
                               &(new_pos->y), &(new_pos->z));
  }
}

/**
 * Set only local XY coordinates of waypoint without update altitude.
 * @todo: how to handle global waypoints?
 */
void waypoint_set_xy_i(uint8_t wp_id, int32_t x, int32_t y)
{
  if (wp_id < nb_waypoint) {
    waypoints[wp_id].enu_i.x = x;
    waypoints[wp_id].enu_i.y = y;
    /* also update ENU float representation */
    waypoints[wp_id].enu_f.x = POS_FLOAT_OF_BFP(waypoints[wp_id].enu_i.x);
    waypoints[wp_id].enu_f.y = POS_FLOAT_OF_BFP(waypoints[wp_id].enu_i.y);
    waypoint_globalize(wp_id);
  }
}

void waypoint_set_alt_i(uint8_t wp_id, int32_t alt)
{
  if (wp_id < nb_waypoint) {
    waypoints[wp_id].enu_i.z = alt;
    /* also update ENU float representation */
    waypoints[wp_id].enu_f.z = POS_FLOAT_OF_BFP(waypoints[wp_id].enu_i.z);
    waypoint_globalize(wp_id);
  }
}

void waypoint_set_alt(uint8_t wp_id, float alt)
{
  if (wp_id < nb_waypoint) {
    waypoints[wp_id].enu_f.z = alt;
    /* also update ENU fixed point representation */
    waypoints[wp_id].enu_i.z = POS_BFP_OF_REAL(waypoints[wp_id].enu_f.z);
    waypoint_globalize(wp_id);
  }
}

void waypoint_set_lla(uint8_t wp_id, struct LlaCoor_i *lla)
{
  if (wp_id >= nb_waypoint) {
    return;
  }
  waypoints[wp_id].lla = *lla;
  SetBit(waypoints[wp_id].flags, WP_FLAG_LLA_I);
  waypoint_localize(wp_id);
}

void waypoint_move_lla(uint8_t wp_id, struct LlaCoor_i *lla)
{
  if (wp_id >= nb_waypoint) {
    return;
  }
  waypoint_set_lla(wp_id, lla);
  if (waypoint_is_global(wp_id)) {
    /* lla->alt is above ellipsoid, WP_MOVED_LLA has hmsl alt */
    int32_t hmsl = lla->alt - state.ned_origin_i.lla.alt + state.ned_origin_i.hmsl;
    DOWNLINK_SEND_WP_MOVED_LLA(DefaultChannel, DefaultDevice, &wp_id,
                               &lla->lat, &lla->lon, &hmsl);
  } else {
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                               &waypoints[wp_id].enu_i.x,
                               &waypoints[wp_id].enu_i.y,
                               &waypoints[wp_id].enu_i.z);
  }
}

/** set waypoint latitude/longitude without updating altitude */
void waypoint_set_latlon(uint8_t wp_id, struct LlaCoor_i *lla)
{
  if (wp_id >= nb_waypoint) {
    return;
  }
  waypoints[wp_id].lla.lat = lla->lat;
  waypoints[wp_id].lla.lon = lla->lon;
  SetBit(waypoints[wp_id].flags, WP_FLAG_LLA_I);
  waypoint_localize(wp_id);
}

/** set waypoint to current location and altitude */
void waypoint_set_here(uint8_t wp_id)
{
  if (wp_id >= nb_waypoint) {
    return;
  }
  if (waypoint_is_global(wp_id)) {
    waypoint_set_lla(wp_id, stateGetPositionLla_i());
  } else {
    waypoint_set_enu_i(wp_id, stateGetPositionEnu_i());
  }
}

/** set waypoint to current horizontal location without modifying altitude */
void waypoint_set_here_2d(uint8_t wp_id)
{
  if (wp_id >= nb_waypoint) {
    return;
  }
  if (waypoint_is_global(wp_id)) {
    waypoint_set_latlon(wp_id, stateGetPositionLla_i());
  } else {
    waypoint_set_xy_i(wp_id, stateGetPositionEnu_i()->x, stateGetPositionEnu_i()->y);
  }
}

void waypoint_globalize(uint8_t wp_id)
{
  if (state.ned_initialized_i) {
    struct EcefCoor_i ecef;
    ecef_of_enu_pos_i(&ecef, &state.ned_origin_i, &waypoints[wp_id].enu_i);
    lla_of_ecef_i(&waypoints[wp_id].lla, &ecef);
    SetBit(waypoints[wp_id].flags, WP_FLAG_LLA_I);
  }
}

/** update local ENU coordinates from its LLA coordinates */
void waypoint_localize(uint8_t wp_id)
{
  if (state.ned_initialized_i) {
    struct EnuCoor_i enu;
    enu_of_lla_point_i(&enu, &state.ned_origin_i, &waypoints[wp_id].lla);
    // convert ENU pos from cm to BFP with INT32_POS_FRAC
    enu.x = POS_BFP_OF_REAL(enu.x) / 100;
    enu.y = POS_BFP_OF_REAL(enu.y) / 100;
    enu.z = POS_BFP_OF_REAL(enu.z) / 100;
    waypoints[wp_id].enu_i = enu;
    SetBit(waypoints[wp_id].flags, WP_FLAG_ENU_I);
    ENU_FLOAT_OF_BFP(waypoints[wp_id].enu_f, waypoints[wp_id].enu_i);
    SetBit(waypoints[wp_id].flags, WP_FLAG_ENU_F);
  }
}

/** update local ENU coordinates of global waypoints */
void waypoints_localize_all(void)
{
  uint8_t i = 0;
  for (i = 0; i < nb_waypoint; i++) {
    if (waypoint_is_global(i)) {
      waypoint_localize(i);
    }
  }
}

/** Get LLA coordinates of waypoint.
 * If the waypoint does not have its global coordinates set,
 * the LLA representation is computed if the local origin is set.
 *
 * @param  wp_id waypoint id
 * @return pointer to waypoint LLA coordinates, NULL if invalid
 */
struct LlaCoor_i *waypoint_get_lla(uint8_t wp_id)
{
  if (wp_id < nb_waypoint) {
    if (!bit_is_set(waypoints[wp_id].flags, WP_FLAG_LLA_I)) {
      waypoint_globalize(wp_id);
    }
    return &waypoints[wp_id].lla;
  }
  else {
    return NULL;
  }
}

void waypoint_copy(uint8_t wp_dest, uint8_t wp_src)
{
  if (wp_dest < nb_waypoint && wp_src < nb_waypoint) {
    waypoints[wp_dest] = waypoints[wp_src];
  }
}

void waypoint_position_copy(uint8_t wp_dest, uint8_t wp_src)
{
  if (wp_dest < nb_waypoint && wp_src < nb_waypoint) {
    waypoints[wp_dest].enu_f.x = waypoints[wp_src].enu_f.x;
    waypoints[wp_dest].enu_f.y = waypoints[wp_src].enu_f.y;
    waypoints[wp_dest].enu_i.x = waypoints[wp_src].enu_i.x;
    waypoints[wp_dest].enu_i.y = waypoints[wp_src].enu_i.y;
    waypoints[wp_dest].lla.lat = waypoints[wp_src].lla.lat;
    waypoints[wp_dest].lla.lon = waypoints[wp_src].lla.lon;
  }
}
