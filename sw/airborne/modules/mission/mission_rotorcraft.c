/*
 * Copyright (C) 2014 Kadir Cimenci
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

/** @file modules/mission/mission_rotorcraft.c
 *  @brief messages parser for mission interface
 */

#include "modules/mission/mission_common.h"
#include <string.h>
#include "subsystems/navigation/common_nav.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"

//Buffer zone in [m] before MAX_DIST_FROM_HOME
#define BUFFER_ZONE_DIST 10
bool_t mission_element_convert(struct _mission_element *);

bool_t mission_insert(enum MissionInsertMode insert, struct _mission_element * element) {
  uint8_t tmp;
  mission_element_convert(element);

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
  if (stateIsLocalCoordinateValid()) {
    struct EnuCoor_i tmp_enu_point;
    struct LlaCoor_i tmp_lla_point;

    struct Int32Vect2 home_distance;
    struct Int32Vect2 home_distance_tmp;
    int32_t dist_to_home;

    LLA_BFP_OF_REAL(tmp_lla_point, *lla);

    //Compute ENU components from LLA with respect to ltp origin
    enu_of_lla_point_i(&tmp_enu_point, &state.ned_origin_i, &tmp_lla_point);

    tmp_enu_point.x = POS_BFP_OF_REAL(tmp_enu_point.x)/100;
    tmp_enu_point.y = POS_BFP_OF_REAL(tmp_enu_point.y)/100;
    tmp_enu_point.z = POS_BFP_OF_REAL(tmp_enu_point.z)/100;

    //Bound the new waypoint with max distance from home
    VECT2_DIFF(home_distance, tmp_enu_point, waypoints[WP_HOME]);
  
    //Compute the home_distance of the target waypoint and
    //normalize the target vector with MAX_DIST_FROM_HOME
    INT32_VECT2_RSHIFT(home_distance_tmp, home_distance, INT32_POS_FRAC);
    INT32_VECT2_NORM(dist_to_home, home_distance_tmp);
  
    //Saturate the mission wp not to overflow max_dist_from_home
    //including a buffer zone before limits
    dist_to_home += BUFFER_ZONE_DIST;
    if (dist_to_home > MAX_DIST_FROM_HOME){
      home_distance.x = (home_distance.x * MAX_DIST_FROM_HOME) / dist_to_home;
      home_distance.y = (home_distance.y * MAX_DIST_FROM_HOME) / dist_to_home; 
    }
  
    tmp_enu_point.x = waypoints[WP_HOME].x + home_distance.x;
    tmp_enu_point.y = waypoints[WP_HOME].y + home_distance.y;
    
    ENU_FLOAT_OF_BFP(*point, tmp_enu_point);
    return FALSE; //local coordinate valid, able to compute point_of_lla
  }
  
  //there is no valid local coordinate
  return TRUE;
}

//Function that converts target wp from float point versions to int
bool_t mission_element_convert(struct _mission_element* target_element){
  struct _mission_element tmp_element = *target_element;
  uint8_t i = 0;

  switch (tmp_element.type){
    case MissionWP:
      ENU_BFP_OF_REAL(target_element->element.mission_wp.wp.wp_i, tmp_element.element.mission_wp.wp.wp_f);
      break;
    case MissionCircle:
      ENU_BFP_OF_REAL(target_element->element.mission_circle.center.center_i, tmp_element.element.mission_circle.center.center_f);
      break;
    case MissionSegment:
      ENU_BFP_OF_REAL(target_element->element.mission_segment.from.from_i, tmp_element.element.mission_segment.from.from_f);
      ENU_BFP_OF_REAL(target_element->element.mission_segment.to.to_i, tmp_element.element.mission_segment.to.to_f);
      break;
    case MissionPath:
      for(i = 0; i < 5; i++){
        ENU_BFP_OF_REAL(target_element->element.mission_path.path.path_i[i], tmp_element.element.mission_path.path.path_f[i]);
      }
      break;
    default:
      // invalid element type
      break;
  }

return TRUE;
}
