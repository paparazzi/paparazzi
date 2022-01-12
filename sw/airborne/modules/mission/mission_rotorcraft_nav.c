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

/** @file modules/mission/mission_rotorcraft_nav.c
 *  @brief mission navigation for rotorcrafts
 *
 *  Implement specific navigation routines for the mission control
 *  of a rotorcraft
 */

#include <stdio.h>
#include "modules/mission/mission_common.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"


//Buffer zone in [m] before MAX_DIST_FROM_HOME
#define BUFFER_ZONE_DIST 10

/// Utility function: converts lla (int) to local point (float)
bool mission_point_of_lla(struct EnuCoor_f *point, struct LlaCoor_i *lla)
{
  // return FALSE if there is no valid local coordinate system
  if (!state.ned_initialized_i) {
    return false;
  }

  // change geoid alt to ellipsoid alt
  lla->alt = lla->alt - state.ned_origin_i.hmsl + state.ned_origin_i.lla.alt;

  //Compute ENU components from LLA with respect to ltp origin
  struct EnuCoor_i tmp_enu_point_i;
  enu_of_lla_point_i(&tmp_enu_point_i, &state.ned_origin_i, lla);
  struct EnuCoor_f tmp_enu_point_f;
  // result of enu_of_lla_point_i is in cm, convert to float in m
  VECT3_SMUL(tmp_enu_point_f, tmp_enu_point_i, 0.01);

  //Bound the new waypoint with max distance from home
  struct FloatVect2 home;
  home.x = waypoint_get_x(WP_HOME);
  home.y = waypoint_get_y(WP_HOME);
  struct FloatVect2 vect_from_home;
  VECT2_DIFF(vect_from_home, tmp_enu_point_f, home);
  //Saturate the mission wp not to overflow max_dist_from_home
  //including a buffer zone before limits
  float dist_to_home = float_vect2_norm(&vect_from_home);
  dist_to_home += BUFFER_ZONE_DIST;
  if (dist_to_home > MAX_DIST_FROM_HOME) {
    VECT2_SMUL(vect_from_home, vect_from_home, (MAX_DIST_FROM_HOME / dist_to_home));
  }
  // set new point
  VECT2_SUM(*point, home, vect_from_home);
  point->z = tmp_enu_point_f.z;

  return true;
}

//Function that converts target wp from float point versions to int
bool mission_element_convert(struct _mission_element *el)
{
  struct _mission_element tmp_element = *el;
  uint8_t i = 0;

  switch (tmp_element.type) {
    case MissionWP:
      ENU_BFP_OF_REAL(el->element.mission_wp.wp.wp_i, tmp_element.element.mission_wp.wp.wp_f);
      break;
    case MissionCircle:
      ENU_BFP_OF_REAL(el->element.mission_circle.center.center_i, tmp_element.element.mission_circle.center.center_f);
      break;
    case MissionSegment:
      ENU_BFP_OF_REAL(el->element.mission_segment.from.from_i, tmp_element.element.mission_segment.from.from_f);
      ENU_BFP_OF_REAL(el->element.mission_segment.to.to_i, tmp_element.element.mission_segment.to.to_f);
      break;
    case MissionPath:
      for (i = 0; i < 5; i++) {
        ENU_BFP_OF_REAL(el->element.mission_path.path.path_i[i], tmp_element.element.mission_path.path.path_f[i]);
      }
      break;
    default:
      // invalid element type
      return false;
      break;
  }

  return true;
}

// navigation time step
static const float dt_navigation = 1.0 / ((float)NAVIGATION_FREQUENCY);

//  last_mission_wp, last target wp from mission elements, not used actively and kept for future implementations
struct EnuCoor_i last_mission_wp = { 0., 0., 0. };

/** Navigation function to a single waypoint
*/
static inline bool mission_nav_wp(struct _mission_element *el)
{
  struct EnuCoor_i *target_wp = &(el->element.mission_wp.wp.wp_i);

  //Check proximity and wait for 'duration' seconds in proximity circle if desired
  if (nav_approaching_from(target_wp, NULL, CARROT)) {
    last_mission_wp = *target_wp;

    if (el->duration > 0.) {
      if (nav_check_wp_time(target_wp, el->duration)) { return false; }
    } else { return false; }

  }
  //Go to Mission Waypoint
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  VECT3_COPY(navigation_target, *target_wp);
  NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
  NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(target_wp->z), 0.);

  return true;
}

/** Navigation function on a circle
*/
static inline bool mission_nav_circle(struct _mission_element *el)
{
  struct EnuCoor_i *center_wp = &(el->element.mission_circle.center.center_i);
  int32_t radius = el->element.mission_circle.radius;

  //Draw the desired circle for a 'duration' time
  horizontal_mode = HORIZONTAL_MODE_CIRCLE;
  nav_circle(center_wp, POS_BFP_OF_REAL(radius));
  NavVerticalAutoThrottleMode(RadOfDeg(0.0));
  NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(center_wp->z), 0.);

  if (el->duration > 0. && mission.element_time >= el->duration) {
    return false;
  }

  return true;
}

/** Navigation function along a segment
*/
static inline bool mission_nav_segment(struct _mission_element *el)
{
  struct EnuCoor_i *from_wp = &(el->element.mission_segment.from.from_i);
  struct EnuCoor_i *to_wp   = &(el->element.mission_segment.to.to_i);

  //Check proximity and wait for 'duration' seconds in proximity circle if desired
  if (nav_approaching_from(to_wp, from_wp, CARROT)) {
    last_mission_wp = *to_wp;

    if (el->duration > 0.) {
      if (nav_check_wp_time(to_wp, el->duration)) { return false; }
    } else { return false; }
  }

  //Route Between from-to
  horizontal_mode = HORIZONTAL_MODE_ROUTE;
  nav_route(from_wp, to_wp);
  NavVerticalAutoThrottleMode(RadOfDeg(0.0));
  NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(to_wp->z), 0.);

  return true;
}


/** Navigation function along a path
*/
static inline bool mission_nav_path(struct _mission_element *el)
{
  if (el->element.mission_path.nb == 0) {
    return false; // nothing to do
  }

  if (el->element.mission_path.path_idx == 0) { //first wp of path
    el->element.mission_wp.wp.wp_i = el->element.mission_path.path.path_i[0];
    if (!mission_nav_wp(el)) { el->element.mission_path.path_idx++; }
  }

  else if (el->element.mission_path.path_idx < el->element.mission_path.nb) { //standart wp of path

    struct EnuCoor_i *from_wp = &(el->element.mission_path.path.path_i[(el->element.mission_path.path_idx) - 1]);
    struct EnuCoor_i *to_wp   = &(el->element.mission_path.path.path_i[el->element.mission_path.path_idx]);

    //Check proximity and wait for t seconds in proximity circle if desired
    if (nav_approaching_from(to_wp, from_wp, CARROT)) {
      last_mission_wp = *to_wp;

      if (el->duration > 0.) {
        if (nav_check_wp_time(to_wp, el->duration)) {
          el->element.mission_path.path_idx++;
        }
      } else { el->element.mission_path.path_idx++; }
    }
    //Route Between from-to
    horizontal_mode = HORIZONTAL_MODE_ROUTE;
    nav_route(from_wp, to_wp);
    NavVerticalAutoThrottleMode(RadOfDeg(0.0));
    NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(from_wp->z), 0.);
  } else { return false; } //end of path

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
#define MISSION_WAIT_TIMEOUT 30 // wait 30 seconds before ending mission
#endif

static bool mission_wait_started = false;
#if MISSION_WAIT_TIMEOUT
static float mission_wait_time = 0.f;
static struct _mission_element mission_wait_wp;
static bool mission_wait_pattern(void) {
  if (!mission_wait_started) {
    mission_wait_wp.element.mission_wp.wp.wp_i = *stateGetPositionEnu_i();
    mission_wait_time = 0.f;
    mission_wait_started = true;
  }
  mission_nav_wp(&mission_wait_wp);
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
      el_running = mission_nav_wp(el);
      break;
    case MissionCircle:
      el_running = mission_nav_circle(el);
      break;
    case MissionSegment:
      el_running = mission_nav_segment(el);
      break;
    case MissionPath:
      el_running = mission_nav_path(el);
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

  if (!el_running) {
    // reset timer
    mission.element_time = 0.;
    // go to next element
    mission.current_idx = (mission.current_idx + 1) % MISSION_ELEMENT_NB;
  }
  return true;
}

