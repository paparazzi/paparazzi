/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/meteo/cloud_sim.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Basic cloud simulation for testing adaptive navigation patterns
 */

#include "modules/meteo/cloud_sim.h"
#include "modules/nav/common_nav.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"
#include "state.h"

// default radius in WP mode
#ifndef CLOUD_SIM_RADIUS
#define CLOUD_SIM_RADIUS 150.f
#endif

// default mode
#ifndef CLOUD_SIM_MODE
#define CLOUD_SIM_MODE CLOUD_SIM_WP
#endif

// use CLOUD waypoint if any by default, 0 (none) otherwise
#if (defined WP_CLOUD) && !(defined CLOUD_SIM_WP_ID)
#define CLOUD_SIM_WP_ID WP_CLOUD
#elif !(defined CLOUD_SIM_WP_ID)
#define CLOUD_SIM_WP_ID 0
#endif
static uint8_t cloud_sim_circle_id = CLOUD_SIM_WP_ID;

#if (defined CLOUD_SIM_WP_POLYGON)
static uint8_t cloud_sim_polygon[] = CLOUD_SIM_WP_POLYGON;
#ifndef CLOUD_SIM_WPS_NB
#pragma error "CLOUD_SIM: please define CLOUD_SIM_WPS_NB for custom polygon"
#endif
#elif (defined SECTOR_CLOUD)
static uint8_t cloud_sim_polygon[] = SECTOR_CLOUD;
#define CLOUD_SIM_WPS_NB SECTOR_CLOUD_NB
#else
#define CLOUD_SIM_WPS_NB 1
static uint8_t cloud_sim_polygon[CLOUD_SIM_WPS_NB] = {0};
#endif

#ifndef CLOUD_SIM_SPEED_X
#define CLOUD_SIM_SPEED_X 0.f
#endif

#ifndef CLOUD_SIM_SPEED_Y
#define CLOUD_SIM_SPEED_Y 0.f
#endif

struct CloudSim cloud_sim;

/*********************
 * Utility functions *
 *********************/

static float distance_to_wp(struct EnuCoor_f * pos, uint8_t id)
{
  if (id > 0 && id < nb_waypoint) {
    struct FloatVect2 diff = { pos->x - waypoints[id].x, pos->y - waypoints[id].y };
    return float_vect2_norm(&diff);
  } else {
    return -1.f; // invalid WP id
  }
}

/**********************
 * External functions *
 **********************/

void cloud_sim_init(void)
{
  cloud_sim.reset = false;
  cloud_sim.mode = CLOUD_SIM_MODE;
  cloud_sim.speed.x = CLOUD_SIM_SPEED_X;
  cloud_sim.speed.y = CLOUD_SIM_SPEED_Y;
  cloud_sim.radius = CLOUD_SIM_RADIUS;
}

/** periodic call for border detection */
void cloud_sim_detect(void)
{
  uint32_t stamp = get_sys_time_usec();
  struct EnuCoor_f * pos = stateGetPositionEnu_f();
  uint8_t inside = 0; // 1: inside, 0: outside

  switch (cloud_sim.mode) {
    case CLOUD_SIM_WP:
      // Test the distance to the reference waypoint
      if (cloud_sim_circle_id > 0 && distance_to_wp(pos, cloud_sim_circle_id) > cloud_sim.radius) {
        inside = 0;
      } else {
        inside = 1;
      }
      AbiSendMsgPAYLOAD_DATA(CLOUD_SENSOR_ID, stamp, 1 /* CLOUD_BORDER */, 1, &inside);
      break;
#ifdef SECTOR_CLOUD
    case CLOUD_SIM_POLYGON:
      inside = (uint8_t) InsideCloud(pos->x, pos->y);
      AbiSendMsgPAYLOAD_DATA(CLOUD_SENSOR_ID, stamp, 1 /* CLOUD_BORDER */, 1, &inside);
      break;
#endif
    default:
      break;
  }
}

/** periodic call for moving waypoints */
void cloud_sim_move(void)
{
  if (cloud_sim.mode == CLOUD_SIM_WP) {
    if (cloud_sim_circle_id > 0 && cloud_sim_circle_id < NB_WAYPOINT) {
      struct point wp = waypoints[cloud_sim_circle_id];
      wp.x += cloud_sim.speed.x; // assuming dt = 1.
      wp.y += cloud_sim.speed.y; // assuming dt = 1.
      nav_move_waypoint_point(cloud_sim_circle_id, &wp);
      nav_send_waypoint(cloud_sim_circle_id);
    }
  } else if (cloud_sim.mode == CLOUD_SIM_POLYGON) {
    for (int i = 0; i < CLOUD_SIM_WPS_NB; i++) {
      if (cloud_sim_polygon[i] > 0 && cloud_sim_polygon[i] < NB_WAYPOINT) {
        struct point wp = waypoints[cloud_sim_polygon[i]];
        wp.x += cloud_sim.speed.x; // assuming dt = 1.
        wp.y += cloud_sim.speed.y; // assuming dt = 1.
        nav_move_waypoint_point(cloud_sim_polygon[i], &wp);
        nav_send_waypoint(cloud_sim_polygon[i]);
      }
    }
  }
}

/** reset handler */
void cloud_sim_reset(bool reset)
{
  if (reset) {
    struct point wps[NB_WAYPOINT] = WAYPOINTS_UTM;
    // reset WP
    if (cloud_sim_circle_id > 0 && cloud_sim_circle_id < NB_WAYPOINT) {
      nav_move_waypoint_point(cloud_sim_circle_id, &wps[cloud_sim_circle_id]);
      nav_send_waypoint(cloud_sim_circle_id);
    }
    for (int i = 0; i < CLOUD_SIM_WPS_NB; i++) {
      if (cloud_sim_polygon[i] > 0 && cloud_sim_polygon[i] < NB_WAYPOINT) {
        nav_move_waypoint_point(cloud_sim_polygon[i], &wps[cloud_sim_polygon[i]]);
        nav_send_waypoint(cloud_sim_polygon[i]);
      }
    }
  }
  cloud_sim.reset = false;
}

