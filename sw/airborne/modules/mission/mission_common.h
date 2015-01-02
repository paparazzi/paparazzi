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

/** @file modules/mission/mission_common.h
 *  @brief mission planner library
 *
 *  Provide the generic interface for the mission control
 *  Handle the parsing of datalink messages
 */

#ifndef MISSION_COMMON_H
#define MISSION_COMMON_H

#include "std.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_int.h"

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
  union {
    struct EnuCoor_f wp_f;
    struct EnuCoor_i wp_i;
  } wp;
};

struct _mission_circle {
  union {
    struct EnuCoor_f center_f;
    struct EnuCoor_i center_i;
  } center;

  float radius;
};

struct _mission_segment {
  union {
    struct EnuCoor_f from_f;
    struct EnuCoor_i from_i;
  } from;

  union {
    struct EnuCoor_f to_f;
    struct EnuCoor_i to_i;
  } to;
};

#define MISSION_PATH_NB 5
struct _mission_path {
  union {
    struct EnuCoor_f path_f[MISSION_PATH_NB];
    struct EnuCoor_i path_i[MISSION_PATH_NB];
  } path;

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

/** Max number of elements in the tasks' list
 *  can be redefined
 */
#ifndef MISSION_ELEMENT_NB
#define MISSION_ELEMENT_NB 20
#endif

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
extern bool_t mission_insert(enum MissionInsertMode insert, struct _mission_element *element);

/** Convert mission element's points format if needed
 * @param el pointer to the mission element
 * @return return TRUE if conversion is succesful, FALSE otherwise
 */
extern bool_t mission_element_convert(struct _mission_element *el);

/** Get current mission element
 * @return return a pointer to the next mission element or NULL if no more elements
 */
extern struct _mission_element *mission_get(void);

/** Get the ENU component of LLA mission point
 * This function is firmware specific.
 * @param point pointer to the output ENU point (float)
 * @param lla pointer to the input LLA coordinates (int)
 * @return TRUE if conversion is succesful, FALSE otherwise
 */
extern bool_t mission_point_of_lla(struct EnuCoor_f *point, struct LlaCoor_i *lla);

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

#endif // MISSION_COMMON_H

