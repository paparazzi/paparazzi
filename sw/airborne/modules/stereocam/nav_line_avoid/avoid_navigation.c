/*
 * Copyright (C) 2012-2013
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/stereocam/nav_line_avoid/avoid_navigation.c
 *
 */


// Own Header
#include "avoid_navigation.h"

// Paparazzi Data
#include "state.h"

// Interact with navigation
#include "navigation.h"

// Know waypoint numbers and blocks
#include "generated/flight_plan.h"

// To get data from the stereo cam
#include "modules/stereocam/stereocam.h"

// Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

#include "led.h"


struct AvoidNavigationStruct avoid_navigation_data;
bool obstacle_detected = false;
int32_t counter = 0;

// Called once on paparazzi autopilot start
void init_avoid_navigation()
{
  // Do nothing
  avoid_navigation_data.mode = 0;
}

// Called on each vision analysis result after receiving the struct
void run_avoid_navigation_onvision(void)
{
  // Send ALL vision data to the ground
  DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, 5, avoid_navigation_data.stereo_bin);

  switch (avoid_navigation_data.mode) {
    case 0:     // Go to Goal and stop at obstacles
      //count 4 subsequent obstacles
      if (stereocam_data.data[0] > 60) {
        counter = counter + 1;
        if (counter > 1) {
          counter = 0;
          //Obstacle detected, go to turn until clear mode
          obstacle_detected = true;
          avoid_navigation_data.mode = 1;
        }
      } else {
        counter = 0;
      }
      break;
    case 1:     // Turn until clear
      //count 20 subsequent free frames
      if (stereocam_data.data[0] < 60) {
        counter = counter + 1;
        if (counter > 6) {
          counter = 0;
          //Stop and put waypoint 2.5 m ahead
          struct EnuCoor_i new_coor;
          struct EnuCoor_i *pos = stateGetPositionEnu_i();
          float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
          float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));
          new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (NAV_LINE_AVOID_SEGMENT_LENGTH));
          new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (NAV_LINE_AVOID_SEGMENT_LENGTH));
          new_coor.z = pos->z;
          waypoint_set_xy_i(WP_W1, new_coor.x, new_coor.y);
          obstacle_detected = false;
          avoid_navigation_data.mode = 0;
        }
      } else {
        counter = 0;
      }
      break;
    case 2:
      break;
    default:    // do nothing
      break;
  }
  avoid_navigation_data.stereo_bin[2] = avoid_navigation_data.stereo_bin[0] > 20;
  avoid_navigation_data.stereo_bin[3] = avoid_navigation_data.mode;
  avoid_navigation_data.stereo_bin[4] = counter;
}

void increase_nav_heading(int32_t *heading, int32_t increment)
{
  *heading = *heading + increment;
}

