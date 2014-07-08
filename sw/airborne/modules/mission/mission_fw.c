/*
 * Copyright (C) 2013 ENAC
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
 *
 */

/** @file modules/mission/mission_fw.c
 *  @brief messages parser for mission interface
 */

#include "modules/mission/mission_common.h"

#include <string.h>
#include "subsystems/navigation/common_nav.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"

struct _mission mission;


bool_t mission_insert(enum MissionInsertMode insert, struct _mission_element * element) {
  uint8_t tmp;

  switch (insert) {
    case Append:
      tmp = (mission.insert_idx + 1) % MISSION_ELEMENT_NB;
      if (tmp == mission.current_idx) return FALSE; // no room to insert element
      memcpy(&mission.elements[mission.insert_idx], element, sizeof(struct _mission_element)); // add element
      mission.insert_idx = tmp; // move insert index
      break;
    case Prepend:
      if (mission.current_idx == 0) tmp = MISSION_ELEMENT_NB-1;
      else tmp = mission.current_idx - 1;
      if (tmp == mission.insert_idx) return FALSE; // no room to inser element
      memcpy(&mission.elements[tmp], element, sizeof(struct _mission_element)); // add element
      mission.current_idx = tmp; // move current index
      break;
    case ReplaceCurrent:
      // current element can always be modified, index are not changed
      memcpy(&mission.elements[mission.current_idx], element, sizeof(struct _mission_element));
      break;
    case ReplaceAll:
      // reset queue and index
      memcpy(&mission.elements[0], element, sizeof(struct _mission_element));
      mission.insert_idx = 0;
      mission.current_idx = 0;
      break;
    default:
      // unknown insertion mode
      return FALSE;
  }
  return TRUE;

}

// Utility function: converts lla to local point
bool_t mission_point_of_lla(struct EnuCoor_f *point, struct LlaCoor_f *lla) {
  /* Computes from (lat, long) in the referenced UTM zone */
  struct UtmCoor_f utm;
  utm.zone = nav_utm_zone0;
  utm_of_lla_f(&utm, lla);

  /* Computes relative position to HOME waypoint
   * and bound the distance to max_dist_from_home
   */
  float dx, dy;
  dx = utm.east - nav_utm_east0 - waypoints[WP_HOME].x;
  dy = utm.north - nav_utm_north0 - waypoints[WP_HOME].y;
  BoundAbs(dx, max_dist_from_home);
  BoundAbs(dy, max_dist_from_home);

  /* Update point */
  point->x = waypoints[WP_HOME].x + dx;
  point->y = waypoints[WP_HOME].y + dy;
  point->z = lla->alt;

 return FALSE;
}
