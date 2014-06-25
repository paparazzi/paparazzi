/*
 * Copyright (C) 2014 Paparazzi Team
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

#include "modules/mission/mission_rotorcraft.h"
#include <string.h>
#include "subsystems/navigation/common_nav.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"

//Buffer zone in [m] before MAX_DIST_FROM_HOME
#define BUFFER_ZONE_DIST 10
struct _mission mission;


void mission_init(void) {
  mission.insert_idx = 0;
  mission.current_idx = 0;
  mission.element_time = 0.;
}

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

struct _mission_element * mission_get(void) {
  if (mission.current_idx == mission.insert_idx) {
    return NULL;
  }
  return &(mission.elements[mission.current_idx]);
}

// Report function
void mission_status_report(void) {
  // build task list
  uint8_t task_list[MISSION_ELEMENT_NB];
  uint8_t i = mission.current_idx, j = 0;
  while (i != mission.insert_idx) {
    task_list[j++] = (uint8_t)mission.elements[i].type;
    i = (i+1)%MISSION_ELEMENT_NB;
  }
  if (j == 0) { task_list[j++] = 0; } // Dummy value if task list is empty
  //compute remaining time (or -1. if no time limit)
  float remaining_time = -1.;
  if (mission.elements[mission.current_idx].duration > 0.) {
    remaining_time = mission.elements[mission.current_idx].duration - mission.element_time;
  }

  // send status
  DOWNLINK_SEND_MISSION_STATUS(DefaultChannel, DefaultDevice, &remaining_time, j, task_list);
}

// Utility function: converts lla to local point
static inline bool_t mission_point_of_lla(struct EnuCoor_i *point, struct LlaCoor_i *lla) {
  if (stateIsLocalCoordinateValid()) {
    struct Int32Vect2 home_distance;
    struct Int32Vect2 home_distance_tmp;
    int32_t dist_to_home;

    //Compute ENU components from LLA with respect to ltp origin
    enu_of_lla_point_i(point, &state.ned_origin_i, lla);

    point->x = POS_BFP_OF_REAL(point->x)/100;
    point->y = POS_BFP_OF_REAL(point->y)/100;
    point->z = POS_BFP_OF_REAL(point->z)/100;

    //Bound the new waypoint with max distance from home
    VECT2_DIFF(home_distance, *point, waypoints[WP_HOME]);
  
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
  
    point->x = waypoints[WP_HOME].x + home_distance.x;
    point->y = waypoints[WP_HOME].y + home_distance.y;
    
    return FALSE;
  }
  
  //there is no valid local coordinate
  return TRUE;
}

///////////////////////
// Parsing functions //
///////////////////////

int mission_parse_GOTO_WP(void) {
  if (DL_MISSION_GOTO_WP_ac_id(dl_buffer) != AC_ID) return FALSE; // not for this aircraft
 
  struct EnuCoor_f tmp_wp;
  struct _mission_element me;
  
  tmp_wp.x = DL_MISSION_GOTO_WP_wp_east(dl_buffer);
  tmp_wp.y = DL_MISSION_GOTO_WP_wp_north(dl_buffer);
  tmp_wp.z = DL_MISSION_GOTO_WP_wp_alt(dl_buffer);

  me.type = MissionWP;
  me.duration = DL_MISSION_GOTO_WP_duration(dl_buffer);
  ENU_BFP_OF_REAL(me.element.mission_wp.wp, tmp_wp);

  enum MissionInsertMode insert = (enum MissionInsertMode) (DL_MISSION_GOTO_WP_insert(dl_buffer));
  return mission_insert(insert, &me);

}

int mission_parse_GOTO_WP_LLA(void) {
  if (DL_MISSION_GOTO_WP_LLA_ac_id(dl_buffer) != AC_ID) return FALSE; // not for this aircraft

  struct LlaCoor_f lla_tmp;
  struct LlaCoor_i lla;

  lla_tmp.lat = RadOfDeg(DL_MISSION_GOTO_WP_LLA_wp_lat(dl_buffer));
  lla_tmp.lon = RadOfDeg(DL_MISSION_GOTO_WP_LLA_wp_lon(dl_buffer));
  
  //wp_alt message is interpreted as the altitude above ltp origin point
  lla_tmp.alt = DL_MISSION_GOTO_WP_LLA_wp_alt(dl_buffer) + ( state.ned_origin_i.lla.alt ) / 1000 ; 

  //Expand the lla point to LlaCoor_i domain from LlaCoor_f
  LLA_BFP_OF_REAL(lla, lla_tmp);
  
  //Create a mission element with proper configuration and insert it to the array
  struct _mission_element me;
  me.type = MissionWP;
  if(mission_point_of_lla(&me.element.mission_wp.wp, &lla)) return FALSE; //there is no valid local coordinate
                                                                          //do not insert mission element
  me.duration = DL_MISSION_GOTO_WP_LLA_duration(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode) (DL_MISSION_GOTO_WP_LLA_insert(dl_buffer));
  return mission_insert(insert, &me);
}

int mission_parse_CIRCLE(void) {
  if (DL_MISSION_CIRCLE_ac_id(dl_buffer) != AC_ID) return FALSE; // not for this aircraft

  struct EnuCoor_f tmp_wp;
  tmp_wp.x = DL_MISSION_CIRCLE_center_east(dl_buffer);
  tmp_wp.y = DL_MISSION_CIRCLE_center_north(dl_buffer);
  tmp_wp.z = DL_MISSION_CIRCLE_center_alt(dl_buffer);

  //Create a mission element with proper configuration and insert it to the array
  struct _mission_element me;
  me.type = MissionCircle;
  
  me.element.mission_circle.radius = DL_MISSION_CIRCLE_radius(dl_buffer);
  me.duration = DL_MISSION_CIRCLE_duration(dl_buffer);
  ENU_BFP_OF_REAL(me.element.mission_circle.center, tmp_wp);

  enum MissionInsertMode insert = (enum MissionInsertMode) (DL_MISSION_CIRCLE_insert(dl_buffer));
  return mission_insert(insert, &me);
}

int mission_parse_CIRCLE_LLA(void) {
  if (DL_MISSION_CIRCLE_LLA_ac_id(dl_buffer) != AC_ID) return FALSE; // not for this aircraft

  struct LlaCoor_f lla_tmp;
  struct LlaCoor_i lla;
  lla_tmp.lat = RadOfDeg(DL_MISSION_CIRCLE_LLA_center_lat(dl_buffer));
  lla_tmp.lon = RadOfDeg(DL_MISSION_CIRCLE_LLA_center_lon(dl_buffer));
  
  //wp_alt message is interpreted as the altitude above ltp origin point
  lla_tmp.alt = DL_MISSION_CIRCLE_LLA_center_alt(dl_buffer) + ( state.ned_origin_i.lla.alt ) / 1000 ; 

  //Expand the lla point to LlaCoor_i domain from LlaCoor_f
  LLA_BFP_OF_REAL(lla, lla_tmp);

  //Create a mission element with proper configuration and insert it to the queue
  struct _mission_element me;
  
  me.type = MissionCircle;
  if(mission_point_of_lla(&me.element.mission_circle.center, &lla)) return FALSE; //there is no valid local coordinate
                                                                                  //do not insert mission element
 
  me.element.mission_circle.radius = DL_MISSION_CIRCLE_LLA_radius(dl_buffer);
  me.duration = DL_MISSION_CIRCLE_LLA_duration(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode) (DL_MISSION_CIRCLE_LLA_insert(dl_buffer));

  return mission_insert(insert, &me);

}

int mission_parse_SEGMENT(void) {
  if (DL_MISSION_SEGMENT_ac_id(dl_buffer) != AC_ID) return FALSE; // not for this aircraft

  struct EnuCoor_f from_tmp;
  struct EnuCoor_f to_tmp;

  from_tmp.x = DL_MISSION_SEGMENT_segment_east_1(dl_buffer);
  from_tmp.y = DL_MISSION_SEGMENT_segment_north_1(dl_buffer);
  from_tmp.z = DL_MISSION_SEGMENT_segment_alt(dl_buffer);
  to_tmp.x   = DL_MISSION_SEGMENT_segment_east_2(dl_buffer);
  to_tmp.y   = DL_MISSION_SEGMENT_segment_north_2(dl_buffer);
  to_tmp.z   = DL_MISSION_SEGMENT_segment_alt(dl_buffer);

  //Create a mission element with proper configuration and insert it to the queue
  struct _mission_element me;
  me.type = MissionSegment;
  ENU_BFP_OF_REAL(me.element.mission_segment.from, from_tmp);
  ENU_BFP_OF_REAL(me.element.mission_segment.to  , to_tmp)  ;
  me.duration = DL_MISSION_SEGMENT_duration(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode) (DL_MISSION_SEGMENT_insert(dl_buffer));

  return mission_insert(insert, &me);
}

int mission_parse_SEGMENT_LLA(void) {
  if (DL_MISSION_SEGMENT_LLA_ac_id(dl_buffer) != AC_ID) return FALSE; // not for this aircraft

  struct LlaCoor_f from_tmp, to_tmp;
  struct LlaCoor_i from_lla, to_lla;

  from_tmp.lat = RadOfDeg(DL_MISSION_SEGMENT_LLA_segment_lat_1(dl_buffer));
  from_tmp.lon = RadOfDeg(DL_MISSION_SEGMENT_LLA_segment_lon_1(dl_buffer));
  //wp_alt message is interpreted as the altitude above ltp origin point
  from_tmp.alt = DL_MISSION_SEGMENT_LLA_segment_alt(dl_buffer) + ( state.ned_origin_i.lla.alt ) / 1000 ;
  to_tmp.lat = RadOfDeg(DL_MISSION_SEGMENT_LLA_segment_lat_2(dl_buffer));
  to_tmp.lon = RadOfDeg(DL_MISSION_SEGMENT_LLA_segment_lon_2(dl_buffer));
  //wp_alt message is interpreted as the altitude above ltp origin point
  to_tmp.alt = DL_MISSION_SEGMENT_LLA_segment_alt(dl_buffer) + ( state.ned_origin_i.lla.alt ) / 1000 ;

  //Expand the lla point to LlaCoor_i domain from LlaCoor_f
  LLA_BFP_OF_REAL(from_lla, from_tmp);
  LLA_BFP_OF_REAL(to_lla  , to_tmp  );

  //Create a mission element with proper configuration and insert it to the queue
  struct _mission_element me;
  me.type = MissionSegment;
  if(mission_point_of_lla(&me.element.mission_segment.from, &from_lla)) return FALSE; //there is no valid local coordinate
                                                                                      //do not insert mission element

  if(mission_point_of_lla(&me.element.mission_segment.to  , &to_lla  )) return FALSE; //there is no valid local coordinate
                                                                                      //do not insert mission element

  me.duration = DL_MISSION_SEGMENT_LLA_duration(dl_buffer);
  enum MissionInsertMode insert = (enum MissionInsertMode) (DL_MISSION_SEGMENT_LLA_insert(dl_buffer));

  return mission_insert(insert, &me);
}

int mission_parse_PATH(void) {

  if (DL_MISSION_PATH_ac_id(dl_buffer) != AC_ID) return FALSE; // not for this aircraft

  struct EnuCoor_f path_tmp[5];
  uint8_t i = 0;

  path_tmp[0].x = DL_MISSION_PATH_point_east_1(dl_buffer);
  path_tmp[0].y = DL_MISSION_PATH_point_north_1(dl_buffer);
  path_tmp[0].z = DL_MISSION_PATH_path_alt(dl_buffer);
  path_tmp[1].x = DL_MISSION_PATH_point_east_2(dl_buffer);
  path_tmp[1].y = DL_MISSION_PATH_point_north_2(dl_buffer);
  path_tmp[1].z = DL_MISSION_PATH_path_alt(dl_buffer);
  path_tmp[2].x = DL_MISSION_PATH_point_east_3(dl_buffer);
  path_tmp[2].y = DL_MISSION_PATH_point_north_3(dl_buffer);
  path_tmp[2].z = DL_MISSION_PATH_path_alt(dl_buffer);
  path_tmp[3].x = DL_MISSION_PATH_point_east_4(dl_buffer);
  path_tmp[3].y = DL_MISSION_PATH_point_north_4(dl_buffer);
  path_tmp[3].z = DL_MISSION_PATH_path_alt(dl_buffer);
  path_tmp[4].x = DL_MISSION_PATH_point_east_5(dl_buffer);
  path_tmp[4].y = DL_MISSION_PATH_point_north_5(dl_buffer);
  path_tmp[4].z = DL_MISSION_PATH_path_alt(dl_buffer);

  //Create a mission element with proper configuration and insert it to the queue
  struct _mission_element me;
  me.type = MissionPath;

  for(i = 0; i < 5; i++){
    ENU_BFP_OF_REAL(me.element.mission_path.path[i], path_tmp[i]);
  }

  me.element.mission_path.nb = DL_MISSION_PATH_nb(dl_buffer);

  if (me.element.mission_path.nb > MISSION_PATH_NB) me.element.mission_path.nb = MISSION_PATH_NB;
  me.element.mission_path.path_idx = 0;
  me.duration = DL_MISSION_PATH_duration(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode) (DL_MISSION_PATH_insert(dl_buffer));

  return mission_insert(insert, &me);
}

int mission_parse_PATH_LLA(void) {
  if (DL_MISSION_PATH_LLA_ac_id(dl_buffer) != AC_ID) return FALSE; // not for this aircraft

  struct LlaCoor_f path_tmp[MISSION_PATH_NB];
  struct LlaCoor_i path_lla[MISSION_PATH_NB];
  uint8_t i = 0;

  path_tmp[0].lat = RadOfDeg(DL_MISSION_PATH_LLA_point_lat_1(dl_buffer));
  path_tmp[0].lon = RadOfDeg(DL_MISSION_PATH_LLA_point_lon_1(dl_buffer));
  path_tmp[0].alt = DL_MISSION_PATH_LLA_path_alt(dl_buffer) + ( state.ned_origin_i.lla.alt ) / 1000 ;
  path_tmp[1].lat = RadOfDeg(DL_MISSION_PATH_LLA_point_lat_2(dl_buffer));
  path_tmp[1].lon = RadOfDeg(DL_MISSION_PATH_LLA_point_lon_2(dl_buffer));
  path_tmp[1].alt = DL_MISSION_PATH_LLA_path_alt(dl_buffer) + ( state.ned_origin_i.lla.alt ) / 1000 ;
  path_tmp[2].lat = RadOfDeg(DL_MISSION_PATH_LLA_point_lat_3(dl_buffer));
  path_tmp[2].lon = RadOfDeg(DL_MISSION_PATH_LLA_point_lon_3(dl_buffer));
  path_tmp[2].alt = DL_MISSION_PATH_LLA_path_alt(dl_buffer) + ( state.ned_origin_i.lla.alt ) / 1000 ;
  path_tmp[3].lat = RadOfDeg(DL_MISSION_PATH_LLA_point_lat_4(dl_buffer));
  path_tmp[3].lon = RadOfDeg(DL_MISSION_PATH_LLA_point_lon_4(dl_buffer));
  path_tmp[3].alt = DL_MISSION_PATH_LLA_path_alt(dl_buffer) + ( state.ned_origin_i.lla.alt ) / 1000 ;
  path_tmp[4].lat = RadOfDeg(DL_MISSION_PATH_LLA_point_lat_5(dl_buffer));
  path_tmp[4].lon = RadOfDeg(DL_MISSION_PATH_LLA_point_lon_5(dl_buffer));
  path_tmp[4].alt = DL_MISSION_PATH_LLA_path_alt(dl_buffer) + ( state.ned_origin_i.lla.alt ) / 1000 ;

  //Expand the lla point to LlaCoor_i domain from LlaCoor_f
  for(i = 0; i < MISSION_PATH_NB; i++){
    LLA_BFP_OF_REAL(path_lla[i], path_tmp[i]);
  }

  //Create a mission element with proper configuration and insert it to the queue
  struct _mission_element me;
  me.type = MissionPath;
  me.element.mission_path.nb = DL_MISSION_PATH_LLA_nb(dl_buffer);

  if (me.element.mission_path.nb > MISSION_PATH_NB) me.element.mission_path.nb = MISSION_PATH_NB;
  for (i = 0; i < me.element.mission_path.nb; i++) {
       if(mission_point_of_lla(&me.element.mission_path.path[i], &path_lla[i])) return FALSE; //there is no valid local coordinate
                                                                                              //do not insert mission element
  }
  me.element.mission_path.path_idx = 0;
  me.duration = DL_MISSION_PATH_LLA_duration(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode) (DL_MISSION_PATH_LLA_insert(dl_buffer));

  return mission_insert(insert, &me);
}

int mission_parse_GOTO_MISSION(void) {
  if (DL_GOTO_MISSION_ac_id(dl_buffer) != AC_ID) return FALSE; // not for this aircraft

  uint8_t mission_id = DL_GOTO_MISSION_mission_id(dl_buffer);
  if (mission_id < MISSION_ELEMENT_NB) {
    //reset element time
    mission.element_time = 0.;
    mission.current_idx = mission_id;
  }
  else return FALSE;

  return TRUE;
}

int mission_parse_NEXT_MISSION(void) {
  if (DL_NEXT_MISSION_ac_id(dl_buffer) != AC_ID) return FALSE; // not for this aircraft

  if (mission.current_idx == mission.insert_idx) return FALSE; // already at the last position

  //reset element time
  mission.element_time = 0.;
  // increment current index
  mission.current_idx = (mission.current_idx + 1) % MISSION_ELEMENT_NB;
  return TRUE;
}

int mission_parse_END_MISSION(void) {
  if (DL_END_MISSION_ac_id(dl_buffer) != AC_ID) return FALSE; // not for this aircraft

  //reset element time
  mission.element_time = 0.;
  // set current index to insert index (last position)
  mission.current_idx = mission.insert_idx;
  return TRUE;
}
