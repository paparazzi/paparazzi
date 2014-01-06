/*
 * Copyright (C) 2013 Gautier Hattenberger
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

/** @file modules/mission/mission.h
 *  @brief mission planner library
 *
 *  Provide the generic interface for the mission control
 *  Handle the parsing of datalink messages
 */

#ifndef MISSION_H
#define MISSION_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

enum MissionType {
  MissionWP = 1,
  MissionCircle = 2,
  MissionSegment = 3,
  MissionPath = 4,
  MissionSurvey = 5,
  MissionEight = 6,
  MissionOval = 7
};

enum MissionInsertMode {
  Append,         ///< add at the last position
  Prepend,        ///< add before the current element
  ReplaceCurrent, ///< replace current element
  ReplaceAll      ///< remove all elements and add the new one
};

struct _mission_wp {
  struct EnuCoor_f wp;
};

struct _mission_circle {
  struct EnuCoor_f center;
  float radius;
};

struct _mission_segment {
  struct EnuCoor_f from;
  struct EnuCoor_f to;
};

#define MISSION_PATH_NB 5
struct _mission_path {
  struct EnuCoor_f path[MISSION_PATH_NB];
  uint8_t path_idx;
  uint8_t nb;
};

struct _mission_element {
  enum MissionType type;
  union {
    struct _mission_wp mission_wp;
    struct _mission_circle mission_circle;
    struct _mission_segment mission_segment;
    struct _mission_path mission_path;
  } element;
  float duration; ///< time to spend in the element (<= 0 to disable)
};

#define MISSION_ELEMENT_NB 20
struct _mission {
  struct _mission_element elements[MISSION_ELEMENT_NB];
  float element_time;   ///< time in second spend in the current element
  uint8_t insert_idx;   ///< inserstion index
  uint8_t current_idx;  ///< current mission element index
};

extern struct _mission mission;

/** Init mission structure
 */
extern void mission_init(void);

/** Insert a mission element according to the insertion mode
 * @param insert insertion mode
 * @param element mission element structure
 * @return return TRUE if insertion is succesful, FALSE otherwise
 */
extern bool_t mission_insert(enum MissionInsertMode insert, struct _mission_element * element);

/** Get current mission element
 * @return return a pointer to the next mission element or NULL if no more elements
 */
extern struct _mission_element * mission_get(void);

/** Run mission
 *
 * This function should be implemented into a dedicated file since
 * navigation functions are different for different firmwares
 *
 * Currently, this function should be called from the flight plan
 *
 * @return return TRUE when the mission is running, FALSE when it is finished
 */
extern int mission_run(void);

/** Report mission status
 *
 * Send mission status over datalink
 */
extern void mission_status_report(void);

/** Parsing functions called when a mission message is received
 */
extern int mission_parse_GOTO_WP(void);
extern int mission_parse_GOTO_WP_LLA(void);
extern int mission_parse_CIRCLE(void);
extern int mission_parse_CIRCLE_LLA(void);
extern int mission_parse_SEGMENT(void);
extern int mission_parse_SEGMENT_LLA(void);
extern int mission_parse_PATH(void);
extern int mission_parse_PATH_LLA(void);
extern int mission_parse_SURVEY(void);
extern int mission_parse_SURVEY_LLA(void);
extern int mission_parse_GOTO_MISSION(void);
extern int mission_parse_NEXT_MISSION(void);
extern int mission_parse_END_MISSION(void);

/** Status report messages
 * @todo
 */

#endif // MISSION
