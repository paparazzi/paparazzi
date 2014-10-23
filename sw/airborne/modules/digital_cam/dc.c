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

// for waypoints, but still only fixedwing
#include "subsystems/navigation/common_nav.h"

// Variables with boot defaults
uint8_t dc_autoshoot_meter_grid = 100;
uint8_t dc_autoshoot_quartersec_period = 2;
dc_autoshoot_type dc_autoshoot = DC_AUTOSHOOT_STOP;
uint16_t dc_gps_count = 0;
uint8_t dc_cam_tracing = 1;
float dc_cam_angle = 0;

float dc_circle_interval = 0;
float dc_circle_start_angle = 0;
float dc_circle_last_block = 0;
float dc_circle_max_blocks = 0;

float dc_gps_dist = 50;
float dc_gps_next_dist = 0;
float dc_gps_x = 0;
float dc_gps_y = 0;

bool_t dc_probing = FALSE;


#ifdef SENSOR_SYNC_SEND

uint16_t dc_photo_nr = 0;
uint16_t dc_buffer = 0;

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "state.h"
#include "subsystems/gps.h"


void dc_init(void)
{
#ifdef DC_AUTOSHOOT_QUARTERSEC_PERIOD
  dc_autoshoot_quartersec_period = DC_AUTOSHOOT_QUARTERSEC_PERIOD;
#endif
#ifdef DC_AUTOSHOOT_METER_GRID
  dc_autoshoot_meter_grid = DC_AUTOSHOOT_METER_GRID;
#endif
}

void dc_send_shot_position(void)
{
  int16_t phi = DegOfRad(stateGetNedToBodyEulers_f()->phi*10.0f);
  int16_t theta = DegOfRad(stateGetNedToBodyEulers_f()->theta*10.0f);
  float gps_z = ((float)gps.hmsl) / 1000.0f;
  int16_t course = (DegOfRad(gps.course)/((int32_t)1e6));
  int16_t photo_nr = -1;

  if (dc_buffer < DC_IMAGE_BUFFER) {
    dc_buffer++;
    dc_photo_nr++;
    photo_nr = dc_photo_nr;
  }

  DOWNLINK_SEND_DC_SHOT(DefaultChannel, DefaultDevice,
                        &photo_nr,
                        &gps.utm_pos.east,
                        &gps.utm_pos.north,
                        &gps_z,
                        &gps.utm_pos.zone,
                        &phi,
                        &theta,
                        &course,
                        &gps.gspeed,
                        &gps.tow);
}
#endif /* SENSOR_SYNC_SEND */

uint8_t dc_info(void) {
#ifdef DOWNLINK_SEND_DC_INFO
  float course = DegOfRad(stateGetNedToBodyEulers_f()->psi);
  DOWNLINK_SEND_DC_INFO(DefaultChannel, DefaultDevice,
                        &dc_autoshoot,
                        &stateGetPositionEnu_f()->x,
                        &stateGetPositionEnu_f()->y,
                        &course,
                        &dc_buffer,
                        &dc_gps_dist,
                        &dc_gps_next_dist,
                        &dc_gps_x,
                        &dc_gps_y,
                        &dc_circle_start_angle,
                        &dc_circle_interval,
                        &dc_circle_last_block,
                        &dc_gps_count,
                        &dc_autoshoot_quartersec_period);
#endif
  return 0;
}

/* shoot on circle */
uint8_t dc_circle(float interval, float start) {
  dc_autoshoot = DC_AUTOSHOOT_CIRCLE;
  dc_gps_count = 0;
  dc_circle_interval = fmodf(fmaxf(interval, 1.), 360.);

  if(start == DC_IGNORE) {
    start = DegOfRad(stateGetNedToBodyEulers_f()->psi);
  }

  dc_circle_start_angle = fmodf(start, 360.);
  if (start < 0.)
    start += 360.;
  //dc_circle_last_block = floorf(dc_circle_start_angle/dc_circle_interval);
  dc_circle_last_block = 0;
  dc_circle_max_blocks = floorf(360./dc_circle_interval);
  dc_probing = TRUE;
  dc_info();
  return 0;
}

/* shoot on survey */
uint8_t dc_survey(float interval, float x, float y) {
  dc_autoshoot = DC_AUTOSHOOT_SURVEY;
  dc_gps_count = 0;
  dc_gps_dist = interval;

  if (x == DC_IGNORE && y == DC_IGNORE) {
    dc_gps_x = stateGetPositionEnu_f()->x;
    dc_gps_y = stateGetPositionEnu_f()->y;
  } else if (y == DC_IGNORE) {
    dc_gps_x = waypoints[(uint8_t)x].x;
    dc_gps_y = waypoints[(uint8_t)x].y;
  } else {
    dc_gps_x = x;
    dc_gps_y = y;
  }
  dc_gps_next_dist = 0;
  dc_info();
  return 0;
}

uint8_t dc_stop(void) {
  dc_autoshoot = DC_AUTOSHOOT_STOP;
  dc_info();
  return 0;
}

static float dim_mod(float a, float b, float m) {
  if (a < b) {
    float tmp = a;
    a = b;
    b = tmp;
  }
  return fminf(a-b, b+m-a);
}

void dc_periodic_4Hz(void)
{
  static uint8_t dc_shutter_timer = 0;

  switch (dc_autoshoot) {

    case DC_AUTOSHOOT_PERIODIC:
      if (dc_shutter_timer) {
        dc_shutter_timer--;
      } else {
        dc_shutter_timer = dc_autoshoot_quartersec_period;
        dc_send_command(DC_SHOOT);
      }
      break;

    case DC_AUTOSHOOT_DISTANCE:
      {
        uint32_t dist_to_100m_grid = (gps.utm_pos.north / 100) % 100;
        if (dist_to_100m_grid < dc_autoshoot_meter_grid || 100 - dist_to_100m_grid < dc_autoshoot_meter_grid)
        {
          dc_send_command(DC_SHOOT);
        }
      }
      break;

    case DC_AUTOSHOOT_CIRCLE: {
      float course = DegOfRad(stateGetNedToBodyEulers_f()->psi) - dc_circle_start_angle;
      if (course < 0.)
        course += 360.;
      float current_block = floorf(course/dc_circle_interval);

      if (dc_probing) {
        if (current_block == dc_circle_last_block) {
          dc_probing = FALSE;
        }
      }

      if (dim_mod(current_block, dc_circle_last_block, dc_circle_max_blocks) == 1) {
        dc_gps_count++;
        dc_circle_last_block = current_block;
        dc_send_command(DC_SHOOT);
      }
    }
      break;

    case DC_AUTOSHOOT_SURVEY : {
      float dist_x = dc_gps_x - stateGetPositionEnu_f()->x;
      float dist_y = dc_gps_y - stateGetPositionEnu_f()->y;

      if (dc_probing) {
        if (dist_x*dist_x + dist_y*dist_y < dc_gps_dist*dc_gps_dist) {
          dc_probing = FALSE;
        }
      }

      if (dist_x*dist_x + dist_y*dist_y >= dc_gps_next_dist*dc_gps_next_dist) {
        dc_gps_next_dist += dc_gps_dist;
        dc_gps_count++;
        dc_send_command(DC_SHOOT);
      }
    }
      break;

    default :
      dc_autoshoot = DC_AUTOSHOOT_STOP;
  }
}
