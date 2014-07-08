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

/** @file modules/mission/mission_fw_nav.c
 *  @brief mission navigation for fixedwing aircraft
 *
 *  Implement specific navigation routines for the mission control
 *  of a fixedwing aircraft
 */

#include <stdio.h>
#include "modules/mission/mission_common.h"
#include "firmwares/fixedwing/autopilot.h"
#include "firmwares/fixedwing/nav.h"

// navigation time step
const float dt_navigation = 1.0 / ((float)NAVIGATION_FREQUENCY);

// dirty hack to comply with nav_approaching_xy function
struct EnuCoor_f last_wp_f = { 0., 0., 0. };

/** Navigation function to a single waypoint
 */
static inline bool_t mission_nav_wp(struct _mission_wp * wp) {
  if (nav_approaching_xy(wp->wp.wp_f.x, wp->wp.wp_f.y, last_wp_f.x, last_wp_f.y, CARROT)) {
    last_wp_f = wp->wp.wp_f; // store last wp
    return FALSE; // end of mission element
  }
  // set navigation command
  fly_to_xy(wp->wp.wp_f.x, wp->wp.wp_f.y);
  NavVerticalAutoThrottleMode(0.);
  NavVerticalAltitudeMode(wp->wp.wp_f.z, 0.);
  return TRUE;
}

/** Navigation function on a circle
 */
static inline bool_t mission_nav_circle(struct _mission_circle * circle) {
  nav_circle_XY(circle->center.center_f.x, circle->center.center_f.y, circle->radius);
  NavVerticalAutoThrottleMode(0.);
  NavVerticalAltitudeMode(circle->center.center_f.z, 0.);
  return TRUE;
}

/** Navigation function along a segment
 */
static inline bool_t mission_nav_segment(struct _mission_segment * segment) {
  if (nav_approaching_xy(segment->to.to_f.x, segment->to.to_f.y, segment->from.from_f.x, segment->from.from_f.y, CARROT)) {
    last_wp_f = segment->to.to_f;
    return FALSE; // end of mission element
  }
  nav_route_xy(segment->from.from_f.x, segment->from.from_f.y, segment->to.to_f.x, segment->to.to_f.y);
  NavVerticalAutoThrottleMode(0.);
  NavVerticalAltitudeMode(segment->to.to_f.z, 0.); // both altitude should be the same anyway
  return TRUE;
}

/** Navigation function along a path
 */
static inline bool_t mission_nav_path(struct _mission_path * path) {
  if (path->nb == 0) {
    return FALSE; // nothing to do
  }
  if (path->nb == 1) {
    // handle as a single waypoint
    struct _mission_wp wp;
    wp.wp.wp_f = path->path.path_f[0];
    return mission_nav_wp(&wp);
  }
  if (path->path_idx == path->nb-1) {
    last_wp_f = path->path.path_f[path->path_idx]; // store last wp
    return FALSE; // end of path
  }
  // normal case
  struct EnuCoor_f from_f = path->path.path_f[path->path_idx];
  struct EnuCoor_f to_f = path->path.path_f[path->path_idx+1];
  nav_route_xy(from_f.x, from_f.y, to_f.x, to_f.y);
  NavVerticalAutoThrottleMode(0.);
  NavVerticalAltitudeMode(to_f.z, 0.); // both altitude should be the same anyway
  if (nav_approaching_xy(to_f.x, to_f.y, from_f.x, from_f.y, CARROT)) {
    path->path_idx++; // go to next segment
  }
  return TRUE;
}


int mission_run() {
  // current element
  struct _mission_element * el = NULL;
  if ((el = mission_get()) == NULL) {
    // TODO do something special like a waiting circle before ending the mission ?
    return FALSE; // end of mission
  }

  bool_t el_running = FALSE;
  switch (el->type){
    case MissionWP:
      el_running = mission_nav_wp(&(el->element.mission_wp));
      break;
    case MissionCircle:
      el_running = mission_nav_circle(&(el->element.mission_circle));
      break;
    case MissionSegment:
      el_running = mission_nav_segment(&(el->element.mission_segment));
      break;
    case MissionPath:
      el_running = mission_nav_path(&(el->element.mission_path));
      break;
    default:
      // invalid type or pattern not yet handled
      break;
  }

  // increment element_time
  mission.element_time += dt_navigation;

  // test if element is finished or element time is elapsed
  if (!el_running || (el->duration > 0. && mission.element_time >= el->duration)) {
    // reset timer
    mission.element_time = 0.;
    // go to next element
    mission.current_idx++;
  }

  return TRUE;
}
