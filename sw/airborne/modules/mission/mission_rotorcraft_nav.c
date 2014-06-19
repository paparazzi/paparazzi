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
#include "modules/mission/mission_rotorcraft.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/navigation.h"

// navigation time step
const float dt_navigation = 1.0 / ((float)NAV_FREQ);

//  last_mission_wp, last target wp from mission elements, not used actively and kept for future implementations
struct EnuCoor_i last_mission_wp = { 0., 0., 0. };

/** Navigation function to a single waypoint
 */
static inline bool_t mission_nav_wp(struct _mission_element * el) {
  struct EnuCoor_i target_wp = el->element.mission_wp.wp;
  if(nav_approaching_from(&(target_wp), NULL, CARROT)){
    last_mission_wp = target_wp;
    
    if(el->duration > 0.){ 
      if(nav_check_wp_time(&target_wp, el->duration)) return FALSE;
    }
    else return FALSE;

  }
  
  //Go to Mission Waypoint
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT; 
  INT32_VECT3_COPY(navigation_target, target_wp);
  NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
  NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(target_wp.z), 0.);

return TRUE;
}

/** Navigation function on a circle
 */


static inline bool_t mission_nav_circle(struct _mission_element * el) {
  struct EnuCoor_i center_wp = el->element.mission_circle.center;
  int32_t radius = el->element.mission_circle.radius;
  NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
  NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(center_wp.z), 0.);

//  NavCircleWaypoint(center_wp, el->element.mission_circle.radius);
  horizontal_mode = HORIZONTAL_MODE_CIRCLE; \
  nav_circle(&center_wp, POS_BFP_OF_REAL(radius));
  if (el->duration > 0. && mission.element_time >= el->duration){
    return FALSE;
  }   

   

  return TRUE;
}


/** Navigation function along a segment
 */

static inline bool_t mission_nav_segment(struct _mission_element * el) {

  struct EnuCoor_i from_wp = el->element.mission_segment.from;
  struct EnuCoor_i to_wp   = el->element.mission_segment.to  ;
  if(nav_approaching_from(&(to_wp), &(from_wp), CARROT)){
    last_mission_wp = to_wp;
    
    if(el->duration > 0.){ 
      if(nav_check_wp_time(&to_wp, el->duration)) return FALSE;
    }
    else return FALSE;

  }
  
  //Route Between from-to
   horizontal_mode = HORIZONTAL_MODE_ROUTE; 
   nav_route(&from_wp, &to_wp); 
   NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
   NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(to_wp.z), 0.);

return TRUE;
 /* 
 if (nav_approaching_xy(segment->to.x, segment->to.y, segment->from.x, segment->from.y, CARROT)) {
    last_mission_wp = segment->to;
    return FALSE; // end of mission element
  }
  nav_route_xy(segment->from.x, segment->from.y, segment->to.x, segment->to.y);
  NavVerticalAutoThrottleMode(0.);
  NavVerticalAltitudeMode(segment->to.z, 0.); // both altitude should be the same anyway
  return TRUE;
*/

}


/** Navigation function along a path
 */
/*
static inline bool_t mission_nav_path(struct _mission_path * path) {
  if (path->nb == 0) {
    return FALSE; // nothing to do
  }
  if (path->nb == 1) {
    // handle as a single waypoint
    struct _mission_wp wp = { path->path[0] };
    return mission_nav_wp(&wp);
  }
  if (path->path_idx == path->nb-1) {
    last_mission_wp = path->path[path->path_idx]; // store last wp
    return FALSE; // end of path
  }
  // normal case
  struct EnuCoor_f from = path->path[path->path_idx];
  struct EnuCoor_f to = path->path[path->path_idx+1];
  nav_route_xy(from.x, from.y, to.x, to.y);
  NavVerticalAutoThrottleMode(0.);
  NavVerticalAltitudeMode(to.z, 0.); // both altitude should be the same anyway
  if (nav_approaching_xy(to.x, to.y, from.x, from.y, CARROT)) {
    path->path_idx++; // go to next segment
  }
  return TRUE;
}
*/

int mission_run() {
  // current element

  
  struct _mission_element * el = NULL;
  if ((el = mission_get()) == NULL) {
    // TODO do something special like a waiting circle before ending the mission ?
    return FALSE; // end of mission
  }
//GotoBlock(9);
  bool_t el_running = FALSE;
  switch (el->type){
    case MissionWP:
      //el_running = mission_nav_wp(&(el->element.mission_wp));
      el_running = mission_nav_wp(el);
      //nav_goto_block(9);
      break;
    case MissionCircle:
      //el_running = mission_nav_circle(&(el->element.mission_circle));
      el_running = mission_nav_circle(el);
      break;
    case MissionSegment:
      //el_running = mission_nav_segment(&(el->element.mission_segment));
      el_running = mission_nav_segment(el);
      break;
    case MissionPath:
      //el_running = mission_nav_path(&(el->element.mission_path));
      break;
    default:
      // invalid type or pattern not yet handled
      break;
  }

  // increment element_time
  mission.element_time += dt_navigation;

 //el->duration = -1;
  // test if element is finished or element time is elapsed
  //if (!el_running || (el->duration > 0. && mission.element_time >= el->duration)) { //duration is cancelled
    if (!el_running) {
    // reset timer
    mission.element_time = 0.;
    // go to next element
    mission.current_idx++;
    //nav_goto_block(9);
  }

  return TRUE;
}


