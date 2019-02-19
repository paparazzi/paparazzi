/*
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/ctrl/follow_me.c"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Control a rotorcraft to follow at a defined distance from the target
 */

#include "follow_me.h"

#include "subsystems/datalink/telemetry.h"
#include "generated/modules.h"

// Distance to the target to hover from is by default 5 meters
#ifndef FOLLOW_ME_DISTANCE
#define FOLLOW_ME_DISTANCE 5
#endif

// Minimum speed in m/s which the ground needs to have in order to update the heading
#ifndef FOLLOW_ME_MIN_SPEED
#define FOLLOW_ME_MIN_SPEED 1.0f
#endif

float follow_me_distance = FOLLOW_ME_DISTANCE;
float follow_me_heading = 0.;
float follow_me_min_speed = FOLLOW_ME_MIN_SPEED;

static bool ground_set = false;
static struct LlaCoor_i ground_lla;
static float ground_speed;
static float ground_climb;
static float ground_course;

void follow_me_init(void)
{
  ground_set = false;
}

void follow_me_parse_ground_gps(uint8_t *buf)
{
  if(DL_GROUND_GPS_ac_id(buf) != AC_ID)
    return;
  
  // Save the received values
  ground_lla.lat = DL_GROUND_GPS_lat(buf);
  ground_lla.lon = DL_GROUND_GPS_lon(buf);
  ground_lla.alt = DL_GROUND_GPS_alt(buf);
  ground_speed = DL_GROUND_GPS_speed(buf);
  ground_climb = DL_GROUND_GPS_climb(buf);
  ground_course = DL_GROUND_GPS_course(buf);

  // Update the heading based on the course
  if(ground_speed > follow_me_min_speed) {
    follow_me_heading = ground_course + 180.f;
    if(follow_me_heading > 360.f) follow_me_heading -= 360.f;
  }
  ground_set = true;
}

void follow_me_set_wp(uint8_t wp_id)
{
  // Only if we have a valid ground position
  if(ground_set) {
    // Claculate x and y offset
    int32_t x = POS_BFP_OF_REAL(follow_me_distance*sinf(follow_me_heading/180.*M_PI));
    int32_t y = POS_BFP_OF_REAL(follow_me_distance*cosf(follow_me_heading/180.*M_PI));

    // Set the waypoint
    waypoint_set_latlon(wp_id, &ground_lla);
    waypoint_set_xy_i(wp_id, waypoints[wp_id].enu_i.x+x, waypoints[wp_id].enu_i.y+y);

    // Send to the GCS that the waypoint has been moved
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                               &waypoints[wp_id].enu_i.x,
                               &waypoints[wp_id].enu_i.y,
                               &waypoints[wp_id].enu_i.z);
    ground_set = false;
  }
}

