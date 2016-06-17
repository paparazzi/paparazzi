/*
 * Copyright (C) 2010-2014 The Paparazzi Team
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

/** @file modules/digital_cam/dc.c
 * Standard Digital Camera Control Interface.
 *
 * -Standard IO
 * -I2C Control
 *
 * Usage: (from the flight plan, the settings or any airborne code):
 * - dc_send_command(  )
 * - set the appropriate autoshoot mode (off/time/distance/trigger)
 * - use the module periodic function to set the autorepeat interval
 * - define SENSOR_SYNC_SEND to get the DC_SHOT_MESSAGE on every SHOOT command
 */

#include "dc.h"

// for waypoints, include correct header until we have unified API
#ifdef AP
#include "subsystems/navigation/common_nav.h"
#else
#include "firmwares/rotorcraft/navigation.h"
#endif

#include "mcu_periph/sys_time.h"

/** default time interval for periodic mode: 1sec */
#ifndef DC_AUTOSHOOT_PERIOD
#define DC_AUTOSHOOT_PERIOD 1.0
#endif

/** default distance interval for distance mode: 50m */
#ifndef DC_AUTOSHOOT_DISTANCE_INTERVAL
#define DC_AUTOSHOOT_DISTANCE_INTERVAL 50
#endif

/** default distance interval for survey mode: 50m */
#ifndef DC_AUTOSHOOT_SURVEY_INTERVAL
#define DC_AUTOSHOOT_SURVEY_INTERVAL 50
#endif

// Variables with boot defaults
dc_autoshoot_type dc_autoshoot = DC_AUTOSHOOT_STOP;
uint16_t dc_gps_count = 0;
uint8_t dc_cam_tracing = 1;
float dc_cam_angle = 0;

float dc_circle_interval = 0;
float dc_circle_start_angle = 0;
float dc_circle_last_block = 0;
float dc_circle_max_blocks = 0;

float dc_survey_interval = DC_AUTOSHOOT_SURVEY_INTERVAL;
float dc_gps_next_dist = 0;
float dc_gps_x = 0;
float dc_gps_y = 0;

static struct FloatVect2 last_shot_pos = {0.0, 0.0};
float dc_distance_interval;
float dc_autoshoot_period;

/** by default send DC_SHOT message when photo was taken */
#ifndef DC_SHOT_SYNC_SEND
#define DC_SHOT_SYNC_SEND 1
#endif

#if DC_SHOT_SYNC_SEND

uint16_t dc_photo_nr = 0;

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "state.h"
#include "subsystems/gps.h"

void dc_send_shot_position(void)
{
  // angles in decideg
  int16_t phi = DegOfRad(stateGetNedToBodyEulers_f()->phi * 10.0f);
  int16_t theta = DegOfRad(stateGetNedToBodyEulers_f()->theta * 10.0f);
  int16_t psi = DegOfRad(stateGetNedToBodyEulers_f()->psi * 10.0f);
  // course in decideg
  int16_t course = DegOfRad(stateGetHorizontalSpeedDir_f()) * 10;
  // ground speed in cm/s
  uint16_t speed = stateGetHorizontalSpeedNorm_f() * 10;
  int16_t photo_nr = -1;

  if (dc_photo_nr < DC_IMAGE_BUFFER) {
    dc_photo_nr++;
    photo_nr = dc_photo_nr;
  }

  DOWNLINK_SEND_DC_SHOT(DefaultChannel, DefaultDevice,
                        &photo_nr,
                        &stateGetPositionLla_i()->lat,
                        &stateGetPositionLla_i()->lon,
                        &stateGetPositionLla_i()->alt,
                        &gps.hmsl,
                        &phi,
                        &theta,
                        &psi,
                        &course,
                        &speed,
                        &gps.tow);
}
#else
void dc_send_shot_position(void)
{
}
#endif /* DC_SHOT_SYNC_SEND */

void dc_init(void)
{
  dc_autoshoot = DC_AUTOSHOOT_STOP;
  dc_autoshoot_period = DC_AUTOSHOOT_PERIOD;
  dc_distance_interval = DC_AUTOSHOOT_DISTANCE_INTERVAL;
}

uint8_t dc_info(void)
{
#ifdef DOWNLINK_SEND_DC_INFO
  float course = DegOfRad(stateGetNedToBodyEulers_f()->psi);
  int16_t mode = dc_autoshoot;
  uint8_t shutter = dc_autoshoot_period * 10;
  DOWNLINK_SEND_DC_INFO(DefaultChannel, DefaultDevice,
                        &mode,
                        &stateGetPositionLla_i()->lat,
                        &stateGetPositionLla_i()->lon,
                        &stateGetPositionLla_i()->alt,
                        &course,
                        &dc_photo_nr,
                        &dc_survey_interval,
                        &dc_gps_next_dist,
                        &dc_gps_x,
                        &dc_gps_y,
                        &dc_circle_start_angle,
                        &dc_circle_interval,
                        &dc_circle_last_block,
                        &dc_gps_count,
                        &shutter);
#endif
  return 0;
}

/* shoot on distance */
uint8_t dc_distance(float interval)
{
  dc_autoshoot = DC_AUTOSHOOT_DISTANCE;
  dc_gps_count = 0;
  dc_distance_interval = interval;
  last_shot_pos.x = 0;
  last_shot_pos.y = 0;

  dc_info();
  return 0;
}

/* shoot on circle */
uint8_t dc_circle(float interval, float start)
{
  dc_autoshoot = DC_AUTOSHOOT_CIRCLE;
  dc_gps_count = 0;
  dc_circle_interval = fmodf(fmaxf(interval, 1.), 360.);

  if (start == DC_IGNORE) {
    start = DegOfRad(stateGetNedToBodyEulers_f()->psi);
  }

  dc_circle_start_angle = fmodf(start, 360.);
  if (start < 0.) {
    start += 360.;
  }
  //dc_circle_last_block = floorf(dc_circle_start_angle/dc_circle_interval);
  dc_circle_last_block = 0;
  dc_circle_max_blocks = floorf(360. / dc_circle_interval);
  dc_info();
  return 0;
}

/* shoot on survey */
uint8_t dc_survey(float interval, float x, float y)
{
  dc_autoshoot = DC_AUTOSHOOT_SURVEY;
  dc_gps_count = 0;
  dc_survey_interval = interval;

  if (x == DC_IGNORE && y == DC_IGNORE) {
    dc_gps_x = stateGetPositionEnu_f()->x;
    dc_gps_y = stateGetPositionEnu_f()->y;
  } else if (y == DC_IGNORE) {
    uint8_t wp = (uint8_t)x;
    dc_gps_x = WaypointX(wp);
    dc_gps_y = WaypointY(wp);
  } else {
    dc_gps_x = x;
    dc_gps_y = y;
  }
  dc_gps_next_dist = 0;
  dc_info();
  return 0;
}

uint8_t dc_stop(void)
{
  dc_autoshoot = DC_AUTOSHOOT_STOP;
  dc_info();
  return 0;
}

static float dim_mod(float a, float b, float m)
{
  if (a < b) {
    float tmp = a;
    a = b;
    b = tmp;
  }
  return fminf(a - b, b + m - a);
}

void dc_periodic(void)
{
  static float last_shot_time = 0.;

  switch (dc_autoshoot) {

    case DC_AUTOSHOOT_PERIODIC: {
      float now = get_sys_time_float();
      if (now - last_shot_time > dc_autoshoot_period) {
        last_shot_time = now;
        dc_send_command(DC_SHOOT);
      }
    }
    break;

    case DC_AUTOSHOOT_DISTANCE: {
      struct FloatVect2 cur_pos;
      cur_pos.x = stateGetPositionEnu_f()->x;
      cur_pos.y = stateGetPositionEnu_f()->y;
      struct FloatVect2 delta_pos;
      VECT2_DIFF(delta_pos, cur_pos, last_shot_pos);
      float dist_to_last_shot = float_vect2_norm(&delta_pos);
      if (dist_to_last_shot > dc_distance_interval) {
        dc_gps_count++;
        dc_send_command(DC_SHOOT);
        VECT2_COPY(last_shot_pos, cur_pos);
      }
    }
    break;

    case DC_AUTOSHOOT_CIRCLE: {
      float course = DegOfRad(stateGetNedToBodyEulers_f()->psi) - dc_circle_start_angle;
      if (course < 0.) {
        course += 360.;
      }
      float current_block = floorf(course / dc_circle_interval);

      if (dim_mod(current_block, dc_circle_last_block, dc_circle_max_blocks) == 1) {
        dc_gps_count++;
        dc_circle_last_block = current_block;
        dc_send_command(DC_SHOOT);
      }
    }
    break;

    case DC_AUTOSHOOT_SURVEY: {
      float dist_x = dc_gps_x - stateGetPositionEnu_f()->x;
      float dist_y = dc_gps_y - stateGetPositionEnu_f()->y;

      if (dist_x * dist_x + dist_y * dist_y >= dc_gps_next_dist * dc_gps_next_dist) {
        dc_gps_next_dist += dc_survey_interval;
        dc_gps_count++;
        dc_send_command(DC_SHOOT);
      }
    }
    break;

    default:
      dc_autoshoot = DC_AUTOSHOOT_STOP;
  }
}
