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
#include "autopilot.h"
#include "firmwares/fixedwing/nav.h"
#include "modules/nav/common_nav.h"
#include "generated/flight_plan.h"

/// Utility function: converts lla (int) to local point (float)
bool mission_point_of_lla(struct EnuCoor_f *point, struct LlaCoor_i *lla)
{
  /// TODO: don't convert to float, either use double or do completely in fixed point
  struct LlaCoor_f lla_f;
  LLA_FLOAT_OF_BFP(lla_f, *lla);

  /* Computes from (lat, long) in the referenced UTM zone */
  struct UtmCoor_f utm;
  utm.zone = nav_utm_zone0;
  utm_of_lla_f(&utm, &lla_f);

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
  point->z = lla_f.alt;

  return true;
}

// navigation time step
static const float dt_navigation = 1.0 / ((float)NAVIGATION_FREQUENCY);

// dirty hack to comply with nav_approaching_xy function
struct EnuCoor_f last_wp_f = { 0., 0., 0. };

/** Navigation function to a single waypoint
 */
static inline bool mission_nav_wp(struct _mission_wp *wp)
{
  if (nav_approaching_xy(wp->wp.wp_f.x, wp->wp.wp_f.y, last_wp_f.x, last_wp_f.y, CARROT)) {
    last_wp_f = wp->wp.wp_f; // store last wp
    return false; // end of mission element
  }
  // set navigation command
  fly_to_xy(wp->wp.wp_f.x, wp->wp.wp_f.y);
  NavVerticalAutoThrottleMode(0.);
  NavVerticalAltitudeMode(wp->wp.wp_f.z, 0.);
  return true;
}

/** Navigation function on a circle
 */
static inline bool mission_nav_circle(struct _mission_circle *circle)
{
  nav_circle_XY(circle->center.center_f.x, circle->center.center_f.y, circle->radius);
  NavVerticalAutoThrottleMode(0.);
  NavVerticalAltitudeMode(circle->center.center_f.z, 0.);
  return true;
}

/** Navigation function along a segment
 */
static inline bool mission_nav_segment(struct _mission_segment *segment)
{
  if (nav_approaching_xy(segment->to.to_f.x, segment->to.to_f.y, segment->from.from_f.x, segment->from.from_f.y,
                         CARROT)) {
    last_wp_f = segment->to.to_f;
    return false; // end of mission element
  }
  nav_route_xy(segment->from.from_f.x, segment->from.from_f.y, segment->to.to_f.x, segment->to.to_f.y);
  NavVerticalAutoThrottleMode(0.);
  NavVerticalAltitudeMode(segment->to.to_f.z, 0.); // both altitude should be the same anyway
  return true;
}

/** Navigation function along a path
 */
static inline bool mission_nav_path(struct _mission_path *path)
{
  if (path->nb == 0) {
    return false; // nothing to do
  }
  if (path->nb == 1) {
    // handle as a single waypoint
    struct _mission_wp wp;
    wp.wp.wp_f = path->path.path_f[0];
    return mission_nav_wp(&wp);
  }
  if (path->path_idx == path->nb - 1) {
    last_wp_f = path->path.path_f[path->path_idx]; // store last wp
    return false; // end of path
  }
  // normal case
  struct EnuCoor_f from_f = path->path.path_f[path->path_idx];
  struct EnuCoor_f to_f = path->path.path_f[path->path_idx + 1];
  nav_route_xy(from_f.x, from_f.y, to_f.x, to_f.y);
  NavVerticalAutoThrottleMode(0.);
  NavVerticalAltitudeMode(to_f.z, 0.); // both altitude should be the same anyway
  if (nav_approaching_xy(to_f.x, to_f.y, from_f.x, from_f.y, CARROT)) {
    path->path_idx++; // go to next segment
  }
  return true;
}

/** Call custom navigation function
 */
static inline bool mission_nav_custom(struct _mission_custom *custom, bool init)
{
  if (init) {
    return custom->reg->cb(custom->nb, custom->params, MissionInit);
  } else {
    return custom->reg->cb(custom->nb, custom->params, MissionRun);
  }
}

/** Implement waiting pattern
 *  Only called when MISSION_WAIT_TIMEOUT is not 0
 */
#ifndef MISSION_WAIT_TIMEOUT
#define MISSION_WAIT_TIMEOUT 120 // wait 2 minutes before ending mission
#endif

static bool mission_wait_started = false;
#if MISSION_WAIT_TIMEOUT
static float mission_wait_time = 0.f;
static struct _mission_circle mission_wait_circle;
static bool mission_wait_pattern(void) {
  if (!mission_wait_started) {
    mission_wait_circle.center.center_f = *stateGetPositionEnu_f();
    mission_wait_circle.center.center_f.z += GetAltRef();
    mission_wait_circle.radius = DEFAULT_CIRCLE_RADIUS;
    mission_wait_time = 0.f;
    mission_wait_started = true;
  }
  mission_nav_circle(&mission_wait_circle);
  mission_wait_time += dt_navigation;
  return (mission_wait_time < (float)MISSION_WAIT_TIMEOUT); // keep flying until TIMEOUT
}
#else
static bool mission_wait_pattern(void) {
  return false; // no TIMEOUT, end mission now
}
#endif

int mission_run()
{
  // current element
  struct _mission_element *el = NULL;
  if ((el = mission_get()) == NULL) {
    return mission_wait_pattern();
  }
  mission_wait_started = false;

  bool el_running = false;
  switch (el->type) {
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
    case MissionCustom:
      el_running = mission_nav_custom(&(el->element.mission_custom), mission.element_time < dt_navigation);
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
    mission.current_idx = (mission.current_idx + 1) % MISSION_ELEMENT_NB;
  }

  return true;
}
