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

/** @file mission_msg.h
 *  @brief the new messages for mission planner library
 */

#ifndef MISSION_H
#define MISSION_H

#include "std.h"
#include "subsystems/navigation/common_nav.h"

#include <math.h>

#include "subsystems/datalink/downlink.h"

#include "state.h"
#include "autopilot.h"
#include "subsystems/gps.h"
#include "generated/airframe.h"
#include "dl_protocol.h"

enum MissionType {
  MissionWP,
  MissionCircle,
  MissionSegment,
  MissionPath,
  MissionSurvey,
  MissionEight,
  MissionOval
};

struct _mission_wp {
  struct point wp;
};

struct _mission_circle {
  struct point center;
  float radius;
};

struct _mission_segment {
  struct point from;
  struct point to;
};

#define MISSION_PATH_NB 5
struct _mission_path {
  //struct point[MISSION_PATH_NB] path;
  struct point path[MISSION_PATH_NB];
};

struct _mission_element {
  //char *mission_name;
  enum MissionType type;
  union {
    struct _mission_wp mission_wp;
    struct _mission_circle mission_circle;
    struct _mission_segment mission_segment;
    struct _mission_path mission_path;
  } element;
  uint16_t duration;
  //bool_t mission_finish;
};

#define MISSION_ELEMENT_NB 20
struct _mission {
  //struct _mission_element[MISSION_ELEMENT_NB] elements;
  struct _mission_element elements[MISSION_ELEMENT_NB];
  uint8_t mission_insert_idx;
  uint8_t mission_extract_idx;
  //bool_t mission_nav_finish = FALSE;
};

extern struct _mission mission;

//extern bool_t mission_nav_finish;
extern void mission_init(void);

extern int mission_msg_GOTO_WP(float x, float y, float a, uint16_t d);

extern int mission_msg_SEGMENT(float x1, float y1, float x2, float y2, float a, uint16_t d);

extern int mission_msg_CIRCLE(float x, float y, float radius, float a, uint16_t d);

//extern int mission_msg_PATH_init( void );
extern int i_path, i_point;
extern int mission_msg_PATH(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4, float x5, float y5, float a, uint16_t d);

extern int goto_mission_msg(uint8_t ac_id, uint8_t mission_id);



#define ParseMissionGotoWp() { \
  if (DL_MISSION_GOTO_WP_ac_id(dl_buffer) == AC_ID) { \
    uint8_t ac_id = DL_MISSION_GOTO_WP_ac_id(dl_buffer); \
    float wp_east = DL_MISSION_GOTO_WP_wp_east(dl_buffer); \
    float wp_north = DL_MISSION_GOTO_WP_wp_north(dl_buffer); \
    float wp_alt = DL_MISSION_GOTO_WP_wp_alt(dl_buffer); \
    float duration = DL_MISSION_GOTO_WP_duration(dl_buffer); \
    mission_msg_GOTO_WP(wp_east, wp_north, wp_alt, duration); \
  } \
}

#define ParseMissionPath() { \
  if (DL_MISSION_PATH_ac_id(dl_buffer) == AC_ID) { \
    uint8_t ac_id = DL_MISSION_PATH_ac_id(dl_buffer); \
    float point_east_1 = DL_MISSION_PATH_point_east_1(dl_buffer); \
    float point_north_1 = DL_MISSION_PATH_point_north_1(dl_buffer); \
    float point_east_2 = DL_MISSION_PATH_point_east_2(dl_buffer); \
    float point_north_2 = DL_MISSION_PATH_point_north_2(dl_buffer); \
    float point_east_3 = DL_MISSION_PATH_point_east_3(dl_buffer); \
    float point_north_3 = DL_MISSION_PATH_point_north_3(dl_buffer); \
    float point_east_4 = DL_MISSION_PATH_point_east_4(dl_buffer); \
    float point_north_4 = DL_MISSION_PATH_point_north_4(dl_buffer); \
    float point_east_5 = DL_MISSION_PATH_point_east_5(dl_buffer); \
    float point_north_5 = DL_MISSION_PATH_point_north_5(dl_buffer); \
    float path_alt = DL_MISSION_PATH_path_alt(dl_buffer); \
    float duration = DL_MISSION_PATH_duration(dl_buffer); \
    mission_msg_PATH(point_east_1, point_north_1, point_east_2, point_north_2, point_east_3, point_north_3, point_east_4, point_north_4, point_east_5, point_north_5, path_alt, duration); \
  } \
}


#define ParseMissionSegment() { \
  if (DL_MISSION_SEGMENT_ac_id(dl_buffer) == AC_ID) { \
    uint8_t ac_id = DL_MISSION_SEGMENT_ac_id(dl_buffer); \
    float segment_east_1 = DL_MISSION_SEGMENT_segment_east_1(dl_buffer); \
    float segment_north_1 = DL_MISSION_SEGMENT_segment_north_1(dl_buffer); \
    float segment_east_2 = DL_MISSION_SEGMENT_segment_east_2(dl_buffer); \
    float segment_north_2 = DL_MISSION_SEGMENT_segment_north_2(dl_buffer); \
    float segment_alt = DL_MISSION_SEGMENT_segment_alt(dl_buffer); \
    float duration = DL_MISSION_SEGMENT_duration(dl_buffer); \
    mission_msg_SEGMENT(segment_east_1, segment_north_1, segment_east_2, segment_north_2, segment_alt, duration); \
  } \
}

#define ParseMissionCircle() { \
  if (DL_MISSION_CIRCLE_ac_id(dl_buffer) == AC_ID) { \
    uint8_t ac_id = DL_MISSION_CIRCLE_ac_id(dl_buffer); \
    float center_east = DL_MISSION_CIRCLE_center_east(dl_buffer); \
    float center_north = DL_MISSION_CIRCLE_center_north(dl_buffer); \
    float radius = DL_MISSION_CIRCLE_radius(dl_buffer); \
    float center_alt = DL_MISSION_CIRCLE_center_alt(dl_buffer); \
    float duration = DL_MISSION_CIRCLE_duration(dl_buffer); \
    mission_msg_CIRCLE(center_east, center_north, radius, center_alt, duration); \
  } \
}

#define ParseGotoMission() { \
    uint8_t ac_id = DL_GOTO_MISSION_ac_id(dl_buffer); \
    uint8_t mission_id = DL_GOTO_MISSION_mission_id(dl_buffer); \
    goto_mission_msg(ac_id, mission_id); \
}

#endif // MISSION
