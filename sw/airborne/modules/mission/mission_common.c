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
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "modules/datalink/datalink.h"
#include "modules/datalink/downlink.h"

// Check for unique ID by default
#ifndef MISSION_CHECK_UNIQUE_ID
#define MISSION_CHECK_UNIQUE_ID TRUE
#endif

struct _mission mission = { 0 };

void mission_init(void)
{
  mission.insert_idx = 0;
  mission.current_idx = 0;
  mission.element_time = 0.;

  for (int i = 0; i < MISSION_REGISTER_NB; i++) {
    mission.registered[i].cb = NULL;
    memset(mission.registered[i].type, '\0', MISSION_TYPE_SIZE);
  }
}


// Insert element
bool mission_insert(enum MissionInsertMode insert, struct _mission_element *element)
{
  uint8_t tmp;

#if MISSION_CHECK_UNIQUE_ID
  // check that the new element id is not already in the list
  if (mission_get_from_index(element->index) != NULL) {
    return false;
  }
#endif

  switch (insert) {
    case Append:
      tmp = (mission.insert_idx + 1) % MISSION_ELEMENT_NB;
      if (tmp == mission.current_idx) { return false; } // no room to insert element
      mission.elements[mission.insert_idx] = *element; // add element
      mission.insert_idx = tmp; // move insert index
      break;
    case Prepend:
      if (mission.current_idx == 0) { tmp = MISSION_ELEMENT_NB - 1; }
      else { tmp = mission.current_idx - 1; }
      if (tmp == mission.insert_idx) { return false; } // no room to inser element
      mission.elements[tmp] = *element; // add element
      mission.current_idx = tmp; // move current index
      mission.element_time = 0.; // reset timer
      break;
    case ReplaceCurrent:
      // current element can always be modified, index are not changed
      mission.elements[mission.current_idx] = *element;
      mission.element_time = 0.; // reset timer
      break;
    case ReplaceAll:
      // reset queue and index
      mission.elements[0] = *element;
      mission.current_idx = 0;
      mission.insert_idx = 1;
      mission.element_time = 0.; // reset timer
      break;
    case ReplaceNexts:
      tmp = (mission.current_idx + 1) % MISSION_ELEMENT_NB;
      mission.elements[tmp] = *element;
      mission.insert_idx = (mission.current_idx + 2) % MISSION_ELEMENT_NB;
      break;
    default:
      // unknown insertion mode
      return false;
  }
  return true;

}

// Register new callback
bool mission_register(mission_custom_cb cb, char *type)
{
  for (int i = 0; i < MISSION_REGISTER_NB; i++) {
    if (str_equal(mission.registered[i].type, type)) {
      return false; // identifier already registered
    }
    if (mission.registered[i].cb == NULL) {
      strncpy(mission.registered[i].type, type, MISSION_TYPE_SIZE-1);
      mission.registered[i].cb = cb;
      return true;
    }
  }
  return false; // no more room to register callbacks
}

// Returns a pointer to a register struct with matching types, NULL if not found
static struct _mission_registered *mission_get_registered(char *type)
{
  for (int i = 0; i < MISSION_REGISTER_NB; i++) {
    if (str_equal(mission.registered[i].type, type)) {
      return &(mission.registered[i]);
    }
  }
  return NULL; // not found
}

// Returns a pointer to a mission element struct with matching index, NULL if not found
struct _mission_element *mission_get_from_index(uint8_t index)
{
  uint8_t i = mission.current_idx;
  while (i != mission.insert_idx) {
    if (mission.elements[i].index == index) {
      return &mission.elements[i]; // return first next element with matching index
    }
    i++;
    if (i == MISSION_ELEMENT_NB) {
      i = 0;
    }
  }
  return NULL; // not found
}

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
  // build index list
  uint8_t index_list[MISSION_ELEMENT_NB];
  uint8_t i = mission.current_idx, j = 0;
  while (i != mission.insert_idx) {
    index_list[j++] = mission.elements[i].index;
    i = (i + 1) % MISSION_ELEMENT_NB;
  }
  if (j == 0) { index_list[j++] = 0; } // Dummy value if index list is empty
  //compute remaining time (or -1. if no time limit)
  float remaining_time = -1.;
  if (mission.elements[mission.current_idx].duration > 0.) {
    remaining_time = mission.elements[mission.current_idx].duration - mission.element_time;
  }

  // send status
  DOWNLINK_SEND_MISSION_STATUS(DefaultChannel, DefaultDevice, &remaining_time, j, index_list);
}


///////////////////////
// Parsing functions //
///////////////////////

int mission_parse_GOTO_WP(uint8_t *buf)
{
  if (DL_MISSION_GOTO_WP_ac_id(buf) != AC_ID) { return false; } // not for this aircraft

  struct _mission_element me;
  me.type = MissionWP;
  me.element.mission_wp.wp.x = DL_MISSION_GOTO_WP_wp_east(buf);
  me.element.mission_wp.wp.y = DL_MISSION_GOTO_WP_wp_north(buf);
  me.element.mission_wp.wp.z = DL_MISSION_GOTO_WP_wp_alt(buf);
  me.duration = DL_MISSION_GOTO_WP_duration(buf);
  me.index = DL_MISSION_GOTO_WP_index(buf);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_GOTO_WP_insert(buf));

  return mission_insert(insert, &me);
}

int mission_parse_GOTO_WP_LLA(uint8_t *buf)
{
  if (DL_MISSION_GOTO_WP_LLA_ac_id(buf) != AC_ID) { return false; } // not for this aircraft

  struct LlaCoor_i lla;
  lla.lat = DL_MISSION_GOTO_WP_LLA_wp_lat(buf);
  lla.lon = DL_MISSION_GOTO_WP_LLA_wp_lon(buf);
  lla.alt = DL_MISSION_GOTO_WP_LLA_wp_alt(buf);

  struct _mission_element me;
  me.type = MissionWP;
  // if there is no valid local coordinate, do not insert mission element
  if (!mission_point_of_lla(&me.element.mission_wp.wp, &lla)) { return false; }
  me.duration = DL_MISSION_GOTO_WP_LLA_duration(buf);
  me.index = DL_MISSION_GOTO_WP_LLA_index(buf);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_GOTO_WP_LLA_insert(buf));

  return mission_insert(insert, &me);
}

int mission_parse_CIRCLE(uint8_t *buf)
{
  if (DL_MISSION_CIRCLE_ac_id(buf) != AC_ID) { return false; } // not for this aircraft

  struct _mission_element me;
  me.type = MissionCircle;
  me.element.mission_circle.center.x = DL_MISSION_CIRCLE_center_east(buf);
  me.element.mission_circle.center.y = DL_MISSION_CIRCLE_center_north(buf);
  me.element.mission_circle.center.z = DL_MISSION_CIRCLE_center_alt(buf);
  me.element.mission_circle.radius = DL_MISSION_CIRCLE_radius(buf);
  me.duration = DL_MISSION_CIRCLE_duration(buf);
  me.index = DL_MISSION_CIRCLE_index(buf);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_CIRCLE_insert(buf));

  return mission_insert(insert, &me);
}

int mission_parse_CIRCLE_LLA(uint8_t *buf)
{
  if (DL_MISSION_CIRCLE_LLA_ac_id(buf) != AC_ID) { return false; } // not for this aircraft

  struct LlaCoor_i lla;
  lla.lat = DL_MISSION_CIRCLE_LLA_center_lat(buf);
  lla.lon = DL_MISSION_CIRCLE_LLA_center_lon(buf);
  lla.alt = DL_MISSION_CIRCLE_LLA_center_alt(buf);

  struct _mission_element me;
  me.type = MissionCircle;
  // if there is no valid local coordinate, do not insert mission element
  if (!mission_point_of_lla(&me.element.mission_circle.center, &lla)) { return false; }
  me.element.mission_circle.radius = DL_MISSION_CIRCLE_LLA_radius(buf);
  me.duration = DL_MISSION_CIRCLE_LLA_duration(buf);
  me.index = DL_MISSION_CIRCLE_LLA_index(buf);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_CIRCLE_LLA_insert(buf));

  return mission_insert(insert, &me);
}

int mission_parse_SEGMENT(uint8_t *buf)
{
  if (DL_MISSION_SEGMENT_ac_id(buf) != AC_ID) { return false; } // not for this aircraft

  struct _mission_element me;
  me.type = MissionSegment;
  me.element.mission_segment.from.x = DL_MISSION_SEGMENT_segment_east_1(buf);
  me.element.mission_segment.from.y = DL_MISSION_SEGMENT_segment_north_1(buf);
  me.element.mission_segment.from.z = DL_MISSION_SEGMENT_segment_alt(buf);
  me.element.mission_segment.to.x = DL_MISSION_SEGMENT_segment_east_2(buf);
  me.element.mission_segment.to.y = DL_MISSION_SEGMENT_segment_north_2(buf);
  me.element.mission_segment.to.z = DL_MISSION_SEGMENT_segment_alt(buf);
  me.duration = DL_MISSION_SEGMENT_duration(buf);
  me.index = DL_MISSION_SEGMENT_index(buf);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_SEGMENT_insert(buf));

  return mission_insert(insert, &me);
}

int mission_parse_SEGMENT_LLA(uint8_t *buf)
{
  if (DL_MISSION_SEGMENT_LLA_ac_id(buf) != AC_ID) { return false; } // not for this aircraft

  struct LlaCoor_i from_lla, to_lla;
  from_lla.lat = DL_MISSION_SEGMENT_LLA_segment_lat_1(buf);
  from_lla.lon = DL_MISSION_SEGMENT_LLA_segment_lon_1(buf);
  from_lla.alt = DL_MISSION_SEGMENT_LLA_segment_alt(buf);
  to_lla.lat = DL_MISSION_SEGMENT_LLA_segment_lat_2(buf);
  to_lla.lon = DL_MISSION_SEGMENT_LLA_segment_lon_2(buf);
  to_lla.alt = DL_MISSION_SEGMENT_LLA_segment_alt(buf);

  struct _mission_element me;
  me.type = MissionSegment;
  // if there is no valid local coordinate, do not insert mission element
  if (!mission_point_of_lla(&me.element.mission_segment.from, &from_lla)) { return false; }
  if (!mission_point_of_lla(&me.element.mission_segment.to, &to_lla)) { return false; }
  me.duration = DL_MISSION_SEGMENT_LLA_duration(buf);
  me.index = DL_MISSION_SEGMENT_LLA_index(buf);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_SEGMENT_LLA_insert(buf));

  return mission_insert(insert, &me);
}

int mission_parse_PATH(uint8_t *buf)
{
  if (DL_MISSION_PATH_ac_id(buf) != AC_ID) { return false; } // not for this aircraft

  struct _mission_element me;
  me.type = MissionPath;
  me.element.mission_path.path[0].x = DL_MISSION_PATH_point_east_1(buf);
  me.element.mission_path.path[0].y = DL_MISSION_PATH_point_north_1(buf);
  me.element.mission_path.path[0].z = DL_MISSION_PATH_path_alt(buf);
  me.element.mission_path.path[1].x = DL_MISSION_PATH_point_east_2(buf);
  me.element.mission_path.path[1].y = DL_MISSION_PATH_point_north_2(buf);
  me.element.mission_path.path[1].z = DL_MISSION_PATH_path_alt(buf);
  me.element.mission_path.path[2].x = DL_MISSION_PATH_point_east_3(buf);
  me.element.mission_path.path[2].y = DL_MISSION_PATH_point_north_3(buf);
  me.element.mission_path.path[2].z = DL_MISSION_PATH_path_alt(buf);
  me.element.mission_path.path[3].x = DL_MISSION_PATH_point_east_4(buf);
  me.element.mission_path.path[3].y = DL_MISSION_PATH_point_north_4(buf);
  me.element.mission_path.path[3].z = DL_MISSION_PATH_path_alt(buf);
  me.element.mission_path.path[4].x = DL_MISSION_PATH_point_east_5(buf);
  me.element.mission_path.path[4].y = DL_MISSION_PATH_point_north_5(buf);
  me.element.mission_path.path[4].z = DL_MISSION_PATH_path_alt(buf);
  me.element.mission_path.nb = DL_MISSION_PATH_nb(buf);
  if (me.element.mission_path.nb > MISSION_PATH_NB) { me.element.mission_path.nb = MISSION_PATH_NB; }
  me.element.mission_path.path_idx = 0;
  me.duration = DL_MISSION_PATH_duration(buf);
  me.index = DL_MISSION_PATH_index(buf);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_PATH_insert(buf));

  return mission_insert(insert, &me);
}

int mission_parse_PATH_LLA(uint8_t *buf)
{
  if (DL_MISSION_PATH_LLA_ac_id(buf) != AC_ID) { return false; } // not for this aircraft

  struct LlaCoor_i lla[MISSION_PATH_NB];
  lla[0].lat = DL_MISSION_PATH_LLA_point_lat_1(buf);
  lla[0].lon = DL_MISSION_PATH_LLA_point_lon_1(buf);
  lla[0].alt = DL_MISSION_PATH_LLA_path_alt(buf);
  lla[1].lat = DL_MISSION_PATH_LLA_point_lat_2(buf);
  lla[1].lon = DL_MISSION_PATH_LLA_point_lon_2(buf);
  lla[1].alt = DL_MISSION_PATH_LLA_path_alt(buf);
  lla[2].lat = DL_MISSION_PATH_LLA_point_lat_3(buf);
  lla[2].lon = DL_MISSION_PATH_LLA_point_lon_3(buf);
  lla[2].alt = DL_MISSION_PATH_LLA_path_alt(buf);
  lla[3].lat = DL_MISSION_PATH_LLA_point_lat_4(buf);
  lla[3].lon = DL_MISSION_PATH_LLA_point_lon_4(buf);
  lla[3].alt = DL_MISSION_PATH_LLA_path_alt(buf);
  lla[4].lat = DL_MISSION_PATH_LLA_point_lat_5(buf);
  lla[4].lon = DL_MISSION_PATH_LLA_point_lon_5(buf);
  lla[4].alt = DL_MISSION_PATH_LLA_path_alt(buf);

  struct _mission_element me;
  me.type = MissionPath;
  uint8_t i;
  me.element.mission_path.nb = DL_MISSION_PATH_LLA_nb(buf);
  if (me.element.mission_path.nb > MISSION_PATH_NB) { me.element.mission_path.nb = MISSION_PATH_NB; }
  for (i = 0; i < me.element.mission_path.nb; i++) {
    // if there is no valid local coordinate, do not insert mission element
    if (!mission_point_of_lla(&me.element.mission_path.path[i], &lla[i])) { return false; }
  }
  me.element.mission_path.path_idx = 0;
  me.duration = DL_MISSION_PATH_LLA_duration(buf);
  me.index = DL_MISSION_PATH_LLA_index(buf);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_PATH_LLA_insert(buf));

  return mission_insert(insert, &me);
}

int mission_parse_CUSTOM(uint8_t *buf)
{
  if (DL_MISSION_CUSTOM_ac_id(buf) != AC_ID) { return false; } // not for this aircraft

  struct _mission_element me;
  me.type = MissionCustom;
  me.element.mission_custom.reg = mission_get_registered(DL_MISSION_CUSTOM_type(buf));
  if (me.element.mission_custom.reg == NULL) {
    return false; // unknown type
  }
  me.element.mission_custom.nb = DL_MISSION_CUSTOM_params_length(buf);
  for (int i = 0; i < me.element.mission_custom.nb; i++) {
    me.element.mission_custom.params[i] = DL_MISSION_CUSTOM_params(buf)[i];
  }
  me.duration = DL_MISSION_CUSTOM_duration(buf);
  me.index = DL_MISSION_CUSTOM_index(buf);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_CUSTOM_insert(buf));

  return mission_insert(insert, &me);
}

int mission_parse_UPDATE(uint8_t *buf)
{
  if (DL_MISSION_UPDATE_ac_id(buf) != AC_ID) { return false; } // not for this aircraft

  struct _mission_element *me = mission_get_from_index(DL_MISSION_UPDATE_index(buf));
  if (me == NULL) {
    return false; // unknown type
  }

  float duration = DL_MISSION_UPDATE_duration(buf);
  if (duration > -2.f) { // no update should in fact be -9
    me->duration = duration; // update
  }

  uint8_t nb = DL_MISSION_UPDATE_params_length(buf);
  float params[MISSION_CUSTOM_MAX];
  memcpy(params, DL_MISSION_UPDATE_params(buf), nb*sizeof(float));

  switch (me->type) {
    case MissionCustom:
      return me->element.mission_custom.reg->cb(nb, params, MissionUpdate);
    case MissionWP:
    case MissionCircle:
    case MissionSegment:
    case MissionPath:
    default:
      // TODO handle update param for standard patterns
      break;
  }
  return true;
}

int mission_parse_GOTO_MISSION(uint8_t *buf)
{
  if (DL_GOTO_MISSION_ac_id(buf) != AC_ID) { return false; } // not for this aircraft

  uint8_t mission_id = DL_GOTO_MISSION_mission_id(buf);
  if (mission_id < MISSION_ELEMENT_NB) {
    // reset timer
    mission.element_time = 0.;
    // set current index
    mission.current_idx = mission_id;
  } else { return false; }

  return true;
}

int mission_parse_NEXT_MISSION(uint8_t *buf)
{
  if (DL_NEXT_MISSION_ac_id(buf) != AC_ID) { return false; } // not for this aircraft

  if (mission.current_idx == mission.insert_idx) { return false; } // already at the last position

  // reset timer
  mission.element_time = 0.;
  // increment current index
  mission.current_idx = (mission.current_idx + 1) % MISSION_ELEMENT_NB;
  return true;
}

int mission_parse_END_MISSION(uint8_t *buf)
{
  if (DL_END_MISSION_ac_id(buf) != AC_ID) { return false; } // not for this aircraft

  // reset timer
  mission.element_time = 0.;
  // set current index to insert index (last position)
  mission.current_idx = mission.insert_idx;
  return true;
}
