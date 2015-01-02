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

/** @file modules/mission/mission_common.c
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

void mission_init(void)
{
  mission.insert_idx = 0;
  mission.current_idx = 0;
  mission.element_time = 0.;
}


// Insert element
bool_t mission_insert(enum MissionInsertMode insert, struct _mission_element *element)
{
  uint8_t tmp;
  // convert element if needed, return FALSE if failed
  if (!mission_element_convert(element)) { return FALSE; }

  switch (insert) {
    case Append:
      tmp = (mission.insert_idx + 1) % MISSION_ELEMENT_NB;
      if (tmp == mission.current_idx) { return FALSE; } // no room to insert element
      memcpy(&mission.elements[mission.insert_idx], element, sizeof(struct _mission_element)); // add element
      mission.insert_idx = tmp; // move insert index
      break;
    case Prepend:
      if (mission.current_idx == 0) { tmp = MISSION_ELEMENT_NB - 1; }
      else { tmp = mission.current_idx - 1; }
      if (tmp == mission.insert_idx) { return FALSE; } // no room to inser element
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


// Weak implementation of mission_element_convert (leave element unchanged)
bool_t __attribute__((weak)) mission_element_convert(struct _mission_element *el __attribute__((unused))) { return TRUE; }


// Get element
struct _mission_element *mission_get(void)
{
  if (mission.current_idx == mission.insert_idx) {
    return NULL;
  }
  return &(mission.elements[mission.current_idx]);
}


// Report function
void mission_status_report(void)
{
  // build task list
  uint8_t task_list[MISSION_ELEMENT_NB];
  uint8_t i = mission.current_idx, j = 0;
  while (i != mission.insert_idx) {
    task_list[j++] = (uint8_t)mission.elements[i].type;
    i = (i + 1) % MISSION_ELEMENT_NB;
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


///////////////////////
// Parsing functions //
///////////////////////

int mission_parse_GOTO_WP(void)
{
  if (DL_MISSION_GOTO_WP_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  struct _mission_element me;
  me.type = MissionWP;
  me.element.mission_wp.wp.wp_f.x = DL_MISSION_GOTO_WP_wp_east(dl_buffer);
  me.element.mission_wp.wp.wp_f.y = DL_MISSION_GOTO_WP_wp_north(dl_buffer);
  me.element.mission_wp.wp.wp_f.z = DL_MISSION_GOTO_WP_wp_alt(dl_buffer);
  me.duration = DL_MISSION_GOTO_WP_duration(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_GOTO_WP_insert(dl_buffer));

  return mission_insert(insert, &me);
}

int mission_parse_GOTO_WP_LLA(void)
{
  if (DL_MISSION_GOTO_WP_LLA_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  struct LlaCoor_i lla;
  lla.lat = DL_MISSION_GOTO_WP_LLA_wp_lat(dl_buffer);
  lla.lon = DL_MISSION_GOTO_WP_LLA_wp_lon(dl_buffer);
  lla.alt = DL_MISSION_GOTO_WP_LLA_wp_alt(dl_buffer);

  struct _mission_element me;
  me.type = MissionWP;
  // if there is no valid local coordinate, do not insert mission element
  if (!mission_point_of_lla(&me.element.mission_wp.wp.wp_f, &lla)) { return FALSE; }
  me.duration = DL_MISSION_GOTO_WP_LLA_duration(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_GOTO_WP_LLA_insert(dl_buffer));

  return mission_insert(insert, &me);
}

int mission_parse_CIRCLE(void)
{
  if (DL_MISSION_CIRCLE_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  struct _mission_element me;
  me.type = MissionCircle;
  me.element.mission_circle.center.center_f.x = DL_MISSION_CIRCLE_center_east(dl_buffer);
  me.element.mission_circle.center.center_f.y = DL_MISSION_CIRCLE_center_north(dl_buffer);
  me.element.mission_circle.center.center_f.z = DL_MISSION_CIRCLE_center_alt(dl_buffer);
  me.element.mission_circle.radius = DL_MISSION_CIRCLE_radius(dl_buffer);
  me.duration = DL_MISSION_CIRCLE_duration(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_CIRCLE_insert(dl_buffer));

  return mission_insert(insert, &me);
}

int mission_parse_CIRCLE_LLA(void)
{
  if (DL_MISSION_CIRCLE_LLA_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  struct LlaCoor_i lla;
  lla.lat = DL_MISSION_CIRCLE_LLA_center_lat(dl_buffer);
  lla.lon = DL_MISSION_CIRCLE_LLA_center_lon(dl_buffer);
  lla.alt = DL_MISSION_CIRCLE_LLA_center_alt(dl_buffer);

  struct _mission_element me;
  me.type = MissionCircle;
  // if there is no valid local coordinate, do not insert mission element
  if (!mission_point_of_lla(&me.element.mission_circle.center.center_f, &lla)) { return FALSE; }
  me.element.mission_circle.radius = DL_MISSION_CIRCLE_LLA_radius(dl_buffer);
  me.duration = DL_MISSION_CIRCLE_LLA_duration(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_CIRCLE_LLA_insert(dl_buffer));

  return mission_insert(insert, &me);
}

int mission_parse_SEGMENT(void)
{
  if (DL_MISSION_SEGMENT_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  struct _mission_element me;
  me.type = MissionSegment;
  me.element.mission_segment.from.from_f.x = DL_MISSION_SEGMENT_segment_east_1(dl_buffer);
  me.element.mission_segment.from.from_f.y = DL_MISSION_SEGMENT_segment_north_1(dl_buffer);
  me.element.mission_segment.from.from_f.z = DL_MISSION_SEGMENT_segment_alt(dl_buffer);
  me.element.mission_segment.to.to_f.x = DL_MISSION_SEGMENT_segment_east_2(dl_buffer);
  me.element.mission_segment.to.to_f.y = DL_MISSION_SEGMENT_segment_north_2(dl_buffer);
  me.element.mission_segment.to.to_f.z = DL_MISSION_SEGMENT_segment_alt(dl_buffer);
  me.duration = DL_MISSION_SEGMENT_duration(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_SEGMENT_insert(dl_buffer));

  return mission_insert(insert, &me);
}

int mission_parse_SEGMENT_LLA(void)
{
  if (DL_MISSION_SEGMENT_LLA_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  struct LlaCoor_i from_lla, to_lla;
  from_lla.lat = DL_MISSION_SEGMENT_LLA_segment_lat_1(dl_buffer);
  from_lla.lon = DL_MISSION_SEGMENT_LLA_segment_lon_1(dl_buffer);
  from_lla.alt = DL_MISSION_SEGMENT_LLA_segment_alt(dl_buffer);
  to_lla.lat = DL_MISSION_SEGMENT_LLA_segment_lat_2(dl_buffer);
  to_lla.lon = DL_MISSION_SEGMENT_LLA_segment_lon_2(dl_buffer);
  to_lla.alt = DL_MISSION_SEGMENT_LLA_segment_alt(dl_buffer);

  struct _mission_element me;
  me.type = MissionSegment;
  // if there is no valid local coordinate, do not insert mission element
  if (!mission_point_of_lla(&me.element.mission_segment.from.from_f, &from_lla)) { return FALSE; }
  if (!mission_point_of_lla(&me.element.mission_segment.to.to_f, &to_lla)) { return FALSE; }
  me.duration = DL_MISSION_SEGMENT_LLA_duration(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_SEGMENT_LLA_insert(dl_buffer));

  return mission_insert(insert, &me);
}

int mission_parse_PATH(void)
{
  if (DL_MISSION_PATH_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  struct _mission_element me;
  me.type = MissionPath;
  me.element.mission_path.path.path_f[0].x = DL_MISSION_PATH_point_east_1(dl_buffer);
  me.element.mission_path.path.path_f[0].y = DL_MISSION_PATH_point_north_1(dl_buffer);
  me.element.mission_path.path.path_f[0].z = DL_MISSION_PATH_path_alt(dl_buffer);
  me.element.mission_path.path.path_f[1].x = DL_MISSION_PATH_point_east_2(dl_buffer);
  me.element.mission_path.path.path_f[1].y = DL_MISSION_PATH_point_north_2(dl_buffer);
  me.element.mission_path.path.path_f[1].z = DL_MISSION_PATH_path_alt(dl_buffer);
  me.element.mission_path.path.path_f[2].x = DL_MISSION_PATH_point_east_3(dl_buffer);
  me.element.mission_path.path.path_f[2].y = DL_MISSION_PATH_point_north_3(dl_buffer);
  me.element.mission_path.path.path_f[2].z = DL_MISSION_PATH_path_alt(dl_buffer);
  me.element.mission_path.path.path_f[3].x = DL_MISSION_PATH_point_east_4(dl_buffer);
  me.element.mission_path.path.path_f[3].y = DL_MISSION_PATH_point_north_4(dl_buffer);
  me.element.mission_path.path.path_f[3].z = DL_MISSION_PATH_path_alt(dl_buffer);
  me.element.mission_path.path.path_f[4].x = DL_MISSION_PATH_point_east_5(dl_buffer);
  me.element.mission_path.path.path_f[4].y = DL_MISSION_PATH_point_north_5(dl_buffer);
  me.element.mission_path.path.path_f[4].z = DL_MISSION_PATH_path_alt(dl_buffer);
  me.element.mission_path.nb = DL_MISSION_PATH_nb(dl_buffer);
  if (me.element.mission_path.nb > MISSION_PATH_NB) { me.element.mission_path.nb = MISSION_PATH_NB; }
  me.element.mission_path.path_idx = 0;
  me.duration = DL_MISSION_PATH_duration(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_PATH_insert(dl_buffer));

  return mission_insert(insert, &me);
}

int mission_parse_PATH_LLA(void)
{
  if (DL_MISSION_PATH_LLA_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  struct LlaCoor_i lla[MISSION_PATH_NB];
  lla[0].lat = DL_MISSION_PATH_LLA_point_lat_1(dl_buffer);
  lla[0].lon = DL_MISSION_PATH_LLA_point_lon_1(dl_buffer);
  lla[0].alt = DL_MISSION_PATH_LLA_path_alt(dl_buffer);
  lla[1].lat = DL_MISSION_PATH_LLA_point_lat_2(dl_buffer);
  lla[1].lon = DL_MISSION_PATH_LLA_point_lon_2(dl_buffer);
  lla[1].alt = DL_MISSION_PATH_LLA_path_alt(dl_buffer);
  lla[2].lat = DL_MISSION_PATH_LLA_point_lat_3(dl_buffer);
  lla[2].lon = DL_MISSION_PATH_LLA_point_lon_3(dl_buffer);
  lla[2].alt = DL_MISSION_PATH_LLA_path_alt(dl_buffer);
  lla[3].lat = DL_MISSION_PATH_LLA_point_lat_4(dl_buffer);
  lla[3].lon = DL_MISSION_PATH_LLA_point_lon_4(dl_buffer);
  lla[3].alt = DL_MISSION_PATH_LLA_path_alt(dl_buffer);
  lla[4].lat = DL_MISSION_PATH_LLA_point_lat_5(dl_buffer);
  lla[4].lon = DL_MISSION_PATH_LLA_point_lon_5(dl_buffer);
  lla[4].alt = DL_MISSION_PATH_LLA_path_alt(dl_buffer);

  struct _mission_element me;
  me.type = MissionPath;
  uint8_t i;
  me.element.mission_path.nb = DL_MISSION_PATH_LLA_nb(dl_buffer);
  if (me.element.mission_path.nb > MISSION_PATH_NB) { me.element.mission_path.nb = MISSION_PATH_NB; }
  for (i = 0; i < me.element.mission_path.nb; i++) {
    // if there is no valid local coordinate, do not insert mission element
    if (!mission_point_of_lla(&me.element.mission_path.path.path_f[i], &lla[i])) { return FALSE; }
  }
  me.element.mission_path.path_idx = 0;
  me.duration = DL_MISSION_PATH_LLA_duration(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_PATH_LLA_insert(dl_buffer));

  return mission_insert(insert, &me);
}

int mission_parse_GOTO_MISSION(void)
{
  if (DL_GOTO_MISSION_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  uint8_t mission_id = DL_GOTO_MISSION_mission_id(dl_buffer);
  if (mission_id < MISSION_ELEMENT_NB) {
    mission.current_idx = mission_id;
  } else { return FALSE; }

  return TRUE;
}

int mission_parse_NEXT_MISSION(void)
{
  if (DL_NEXT_MISSION_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  if (mission.current_idx == mission.insert_idx) { return FALSE; } // already at the last position

  // increment current index
  mission.current_idx = (mission.current_idx + 1) % MISSION_ELEMENT_NB;
  return TRUE;
}

int mission_parse_END_MISSION(void)
{
  if (DL_END_MISSION_ac_id(dl_buffer) != AC_ID) { return FALSE; } // not for this aircraft

  // set current index to insert index (last position)
  mission.current_idx = mission.insert_idx;
  return TRUE;
}

