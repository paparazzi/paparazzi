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
  struct point[MISSION_PATH_NB] path;
};

struct _mission_element {
  enum MissionType type;
  union {
    struct _mission_wp mission_wp;
    struct _mission_circle mission_circle;
    struct _mission_segment mission_segment;
    struct _mission_path mission_path;
  } element;
  uint16_t duration;
};

#define MISSION_ELEMENT_NB 20
struct _mission {
  struct _mission_element[MISSION_ELEMENT_NB] mission_tasks;
  uint8_t mission_insert_idx;
  uint8_t mission_extract_idx;

};

extern struct _mission mission;


extern void mission_msg_init(void);

extern int mission_msg_GOTO_WP(float x, float y);

extern int mission_msg_SEGMENT(float x1, float y1, float x2, float y2);

extern int mission_msg_PATH_init( void );
extern int mission_msg_PATH(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4, float x5, float y5);



#define ParseMissionGotoWp() { \
  if (DL_MISSION_GOTO_WP_ac_id(dl_buffer) == AC_ID) { \
    uint8_t ac_id = DL_MISSION_GOTO_WP_ac_id(dl_buffer); \
    float wp_east = DL_MISSION_GOTO_WP_wp_east(dl_buffer); \
    float wp_north = DL_MISSION_GOTO_WP_wp_north(dl_buffer); \
    mission_msg_GOTO_WP(wp_east, wp_north); \
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
    mission_msg_PATH(point_east_1, point_north_1, point_east_2, point_north_2, point_east_3, point_north_3, point_east_4, point_north_4, point_east_5, point_north_5); \
  } \
}


#define ParseMissionSegment() { \
  if (DL_MISSION_SEGMENT_ac_id(dl_buffer) == AC_ID) { \
    uint8_t ac_id = DL_MISSION_SEGMENT_ac_id(dl_buffer); \
    float segment_east_1 = DL_MISSION_SEGMENT_segment_east_1(dl_buffer); \
    float segment_north_1 = DL_MISSION_SEGMENT_segment_north_1(dl_buffer); \
    float segment_east_2 = DL_MISSION_SEGMENT_segment_east_2(dl_buffer); \
    float segment_north_2 = DL_MISSION_SEGMENT_segment_north_2(dl_buffer); \
    mission_msg_SEGMENT(segment_east_1, segment_north_1, segment_east_2, segment_north_2); \
  } \
}


/*
#define ParseMissionGotoWp() { \
    uint8_t ac_id = DL_MISSION_GOTO_WP_ac_id(dl_buffer); \
    float wp_east = DL_MISSION_GOTO_WP_wp_east(dl_buffer); \
    float wp_north = DL_MISSION_GOTO_WP_wp_north(dl_buffer); \
    fly_to_xy(wp_east, wp_north); \
}
*/

#endif // MISSION
