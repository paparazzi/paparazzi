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

#include "modules/datalink/telemetry.h"
#include "generated/modules.h"
#include "generated/flight_plan.h"

// Distance to the target to hover from is by default 45 meters
#ifndef FOLLOW_ME_DISTANCE
#define FOLLOW_ME_DISTANCE 45
#endif

// Height difference between te target be default 60 meters
#ifndef FOLLOW_ME_HEIGHT
#define FOLLOW_ME_HEIGHT 60
#endif

// The relative position GPS timeout in ms
#ifndef FOLLOW_ME_GPS_TIMEOUT
#define FOLLOW_ME_GPS_TIMEOUT 5000
#endif

// The timeout when receiving GPS messages from the ground in ms
#ifndef FOLLOW_ME_GROUND_TIMEOUT
#define FOLLOW_ME_GROUND_TIMEOUT 5000
#endif

// The default heading sin/cos filter value (higher is harder filtering)
#ifndef FOLLOW_ME_FILT
#define FOLLOW_ME_FILT 0.9
#endif

// By default no moving waypoints
#ifndef FOLLOW_ME_MOVING_WPS
#define FOLLOW_ME_MOVING_WPS
#endif

float follow_me_distance = FOLLOW_ME_DISTANCE;
float follow_me_height = FOLLOW_ME_HEIGHT;
float follow_me_heading = 180.;
float follow_me_filt = FOLLOW_ME_FILT;
float follow_me_diag_speed = 1.0;
float follow_me_gps_delay = 200;
float follow_me_datalink_delay = 600;
float follow_me_advance_ms = 800;
float follow_me_min_dist = 1;
float follow_me_min_height = 30;

static uint32_t ground_time_msec = 0;
static bool ground_set = false;
static struct LlaCoor_i ground_lla;
static float ground_speed;
static float ground_climb;
static float ground_course;
static float ground_heading;

static uint8_t moving_wps[] = {FOLLOW_ME_MOVING_WPS};
static uint8_t moving_wps_cnt = 0;
static struct EnuCoor_f last_targetpos;
static float last_targetpos_heading;
static bool last_targetpos_valid = false;

void follow_me_init(void)
{
  ground_set = false;
  ground_time_msec = 0;
  last_targetpos_valid = false;
  moving_wps_cnt = sizeof(moving_wps) / sizeof(uint8_t);
}

void follow_me_periodic(void)
{
  if(!ground_set) {
    return;
  }

  // Calculate the difference to move the waypoints
  struct EnuCoor_i target_pos_cm;
  struct EnuCoor_f cur_targetpos, diff_targetpos;
  float cur_targetpos_heading, diff_targetpos_heading;

  enu_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &ground_lla);
  VECT3_FLOAT_OF_CM(cur_targetpos, target_pos_cm);
  VECT3_DIFF(diff_targetpos, cur_targetpos, last_targetpos);

  cur_targetpos_heading = ground_heading;
  diff_targetpos_heading = cur_targetpos_heading - last_targetpos_heading;

  // Only move if we had a previous location
  VECT3_COPY(last_targetpos, cur_targetpos);
  last_targetpos_heading = cur_targetpos_heading;
  if(!last_targetpos_valid) {
    last_targetpos_valid = true;
    return;
  }

  // Go through all waypoints
  for(uint8_t i = 0; i < moving_wps_cnt; i++) {
    uint8_t wp_id = moving_wps[i];
    struct EnuCoor_f *wp_enu = waypoint_get_enu_f(wp_id);
    struct EnuCoor_f wp_new_enu;
    wp_new_enu.x = wp_enu->x + diff_targetpos.x;
    wp_new_enu.y = wp_enu->y + diff_targetpos.y;
    wp_new_enu.z = wp_enu->z;

    // Rotate the waypoint
    float cos_heading = cosf(diff_targetpos_heading/180.*M_PI);
    float sin_heading = sinf(diff_targetpos_heading/180.*M_PI);
    wp_new_enu.x = ((wp_new_enu.x - cur_targetpos.x) * cos_heading) + ((wp_new_enu.y - cur_targetpos.y) * sin_heading) + cur_targetpos.x;
    wp_new_enu.y = (-(wp_new_enu.x - cur_targetpos.x) * sin_heading) + ((wp_new_enu.y - cur_targetpos.y) * cos_heading) + cur_targetpos.y;

    // Update the waypoint
    waypoint_set_enu(wp_id, &wp_new_enu);

    // Send to the GCS that the waypoint has been moved
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                               &waypoints[wp_id].enu_i.x,
                               &waypoints[wp_id].enu_i.y,
                               &waypoints[wp_id].enu_i.z);
  }
}

void follow_me_parse_target_pos(uint8_t *buf)
{
  if(DL_TARGET_POS_ac_id(buf) != AC_ID)
    return;
  
  // Save the received values
  ground_time_msec = get_sys_time_msec();
  ground_lla.lat = DL_TARGET_POS_lat(buf);
  ground_lla.lon = DL_TARGET_POS_lon(buf);
  ground_lla.alt = DL_TARGET_POS_alt(buf);
  ground_speed = DL_TARGET_POS_speed(buf);
  ground_climb = DL_TARGET_POS_climb(buf);
  ground_course = DL_TARGET_POS_course(buf);
  ground_heading = DL_TARGET_POS_heading(buf);
  if(ground_heading > 360.f) {
    // Ground heading is invalid
    ground_set = false;
    return;
  }

  ground_set = true;
}

void follow_me_set_wp(uint8_t wp_id, float speed)
{
  bool target_valid = true;
  struct NedCoor_f target_pos;
  float diff_time_ms = 0;

  // Check if we got a valid relative position which didn't timeout (FIXME)
  /*if(bit_is_set(gps.valid_fields, GPS_VALID_RELPOS_BIT) && gps.relpos_tow+FOLLOW_ME_GPS_TIMEOUT > gps_tow_from_sys_ticks(sys_time.nb_tick)) {
    static struct NedCoor_f cur_pos;
    static uint32_t last_relpos_tow = 0;

    // Make sure to only use the current state from the receive of the GPS message (FIXME overflow at sunday)
    if(last_relpos_tow < gps.relpos_tow) {
      cur_pos = *stateGetPositionNed_f();
      last_relpos_tow = gps.relpos_tow;
    }

    // Set the target position
    target_pos.x = cur_pos.x - gps.relpos_ned.x / 100.0f;
    target_pos.y = cur_pos.y - gps.relpos_ned.y / 100.0f;
    target_pos.z = cur_pos.z - gps.relpos_ned.z / 100.0f;

    // Calculate the difference in time from measurement
    diff_time_ms = gps_tow_from_sys_ticks(sys_time.nb_tick) - gps.relpos_tow + follow_me_gps_delay;
    if(diff_time_ms < 0) diff_time_ms += (1000*60*60*24*7); //msec of a week
  }
  // Check if we got a position from the ground which didn't timeout and local NED is initialized
  else*/ if(ground_set && state.ned_initialized_i && ground_time_msec+FOLLOW_ME_GROUND_TIMEOUT > get_sys_time_msec()) {
    struct NedCoor_i target_pos_cm;
    ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &ground_lla);
    target_pos.x = target_pos_cm.x / 100.;
    target_pos.y = target_pos_cm.y / 100.;
    target_pos.z = target_pos_cm.z / 100.;

    // Calculate the difference in time from the measurement
    diff_time_ms = get_sys_time_msec() - ground_time_msec + follow_me_datalink_delay;
  }
  // No target found
  else {
    target_valid = false;
  }

  static float gc_cos_filt = 0, gc_sin_filt = 0;

  // Integrate NE over the time (only if information from the ground is valid)
  if(target_valid && ground_set && ground_time_msec+FOLLOW_ME_GROUND_TIMEOUT > get_sys_time_msec() && (diff_time_ms > 0 || follow_me_advance_ms > 0)) {
    // Filter the cosine and sine of the ground course to avoid wrapping
    gc_cos_filt = gc_cos_filt * follow_me_filt + cosf(ground_course/180.*M_PI) * (1 - follow_me_filt);
    gc_sin_filt = gc_sin_filt * follow_me_filt + sinf(ground_course/180.*M_PI) * (1 - follow_me_filt);

    // Add an advance and the difference in measured time multiplied by the speed
    float int_dist_m = (follow_me_advance_ms + diff_time_ms) / 1000.f * ground_speed;
    target_pos.x += int_dist_m * gc_cos_filt;
    target_pos.y += int_dist_m * gc_sin_filt;
  }

  static uint32_t last_time_ms = 0;
  static float dist = FOLLOW_ME_DISTANCE;
  static float height = FOLLOW_ME_HEIGHT;
  static float fmh_cos_filt = 0, fmh_sin_filt = 0;

  // Update the waypoint only when target is valid
  if(target_valid) {
    // Move the distance and height according to the given speed
    if(last_time_ms != 0 && speed != 0) {
      float time_diff = (get_sys_time_msec() - last_time_ms) / 1000.f;
      dist   -= speed * time_diff;
      height -= speed * time_diff;

      if(dist <= follow_me_min_dist) dist = follow_me_min_dist;
      if(height <= follow_me_min_height) height = follow_me_min_height;
    }
    // Reset distance and height if speed is 0
    else if(speed == 0) {
      dist   = follow_me_distance;
      height = follow_me_height;
    }

    // Filter the cosine and sine of the follow me heading to avoid wrapping
    fmh_cos_filt = fmh_cos_filt * follow_me_filt + cosf((ground_heading+follow_me_heading)/180.*M_PI) * (1 - follow_me_filt);
    fmh_sin_filt = fmh_sin_filt * follow_me_filt + sinf((ground_heading+follow_me_heading)/180.*M_PI) * (1 - follow_me_filt);

    // Add the target distance in the direction of the follow me heading
    target_pos.x += dist * fmh_cos_filt;
    target_pos.y += dist * fmh_sin_filt;
    target_pos.z -= height; // Target is in NED

    // Update the waypoint
    struct EnuCoor_f target_enu;
    ENU_OF_TO_NED(target_enu, target_pos);
    waypoint_set_enu(wp_id, &target_enu);

    // Send to the GCS that the waypoint has been moved
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                               &waypoints[wp_id].enu_i.x,
                               &waypoints[wp_id].enu_i.y,
                               &waypoints[wp_id].enu_i.z);
  }

  // Allways update the time to avoid big jumps in distance and height
  last_time_ms = get_sys_time_msec();
}
