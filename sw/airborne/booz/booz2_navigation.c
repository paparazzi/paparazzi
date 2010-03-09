/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 */

#define NAV_C

#include "booz2_navigation.h"

#include "booz2_gps.h"
#include "booz2_ins.h"

#include "booz2_autopilot.h"
#include "flight_plan.h"

#include "booz_fms.h"

#include "math/pprz_algebra_int.h"

const uint8_t nb_waypoint = NB_WAYPOINT;
struct EnuCoor_f waypoints_float[NB_WAYPOINT] = WAYPOINTS;
struct EnuCoor_i waypoints[NB_WAYPOINT];

struct EnuCoor_i booz2_navigation_target;
struct EnuCoor_i booz2_navigation_carrot;

struct EnuCoor_i nav_last_point;

uint16_t stage_time, block_time;

uint8_t nav_stage, nav_block;
uint8_t last_block, last_stage;
uint8_t last_wp __attribute__ ((unused));

int32_t ground_alt;

uint8_t horizontal_mode;
uint8_t nav_segment_start, nav_segment_end;
uint8_t nav_circle_centre;
int32_t nav_circle_radius, nav_circle_qdr, nav_circle_radians;

int32_t nav_roll, nav_pitch;
int32_t nav_heading, nav_course;
float nav_radius;

#ifndef DEFAULT_CIRCLE_RADIUS
#define DEFAULT_CIRCLE_RADIUS 0.
#endif

uint8_t vertical_mode;
uint32_t nav_throttle;
int32_t nav_climb, nav_altitude, nav_flight_altitude;
float flight_altitude;

static inline void nav_set_altitude( void );

#define CLOSE_TO_WAYPOINT (15 << 8)
#define ARRIVED_AT_WAYPOINT (3 << 8)
#define CARROT_DIST (12 << 8)

void booz2_nav_init(void) {
  // init int32 waypoints
  uint8_t i = 0;
  for (i = 0; i < nb_waypoint; i++) {
    waypoints[i].x = POS_BFP_OF_REAL(waypoints_float[i].x);
    waypoints[i].y = POS_BFP_OF_REAL(waypoints_float[i].y);
    waypoints[i].z = POS_BFP_OF_REAL((waypoints_float[i].z - GROUND_ALT));
  }
  nav_block = 0;
  nav_stage = 0;
  ground_alt = POS_BFP_OF_REAL(GROUND_ALT);
  nav_altitude = POS_BFP_OF_REAL(SECURITY_HEIGHT);
  nav_flight_altitude = nav_altitude;
  flight_altitude = SECURITY_ALT;
  INT32_VECT3_COPY( booz2_navigation_target, waypoints[WP_HOME]);
  INT32_VECT3_COPY( booz2_navigation_carrot, waypoints[WP_HOME]);

  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  vertical_mode = VERTICAL_MODE_ALT;

  nav_roll = 0;
  nav_pitch = 0;
  nav_heading = 0;
  nav_course = 0;
  nav_radius = DEFAULT_CIRCLE_RADIUS;
  nav_throttle = 0;
  nav_climb = 0;

}

void booz2_nav_run(void) {

  /* compute a vector to the waypoint */
  struct Int32Vect2 path_to_waypoint;
  VECT2_DIFF(path_to_waypoint, booz2_navigation_target, booz_ins_enu_pos);

  /* saturate it */
  VECT2_STRIM(path_to_waypoint, -(1<<15), (1<<15));

  int32_t dist_to_waypoint;
  INT32_VECT2_NORM(dist_to_waypoint, path_to_waypoint);

  if (dist_to_waypoint < CLOSE_TO_WAYPOINT) {
    VECT2_COPY( booz2_navigation_carrot, booz2_navigation_target);
  }
  else {
    struct Int32Vect2 path_to_carrot;
    VECT2_SMUL(path_to_carrot, path_to_waypoint, CARROT_DIST);
    VECT2_SDIV(path_to_carrot, path_to_carrot, dist_to_waypoint);
    VECT2_SUM(booz2_navigation_carrot, path_to_carrot, booz_ins_enu_pos);
  }

  nav_set_altitude();
}

void nav_circle(uint8_t wp_center, int32_t radius) {
  if (radius == 0) {
    VECT2_COPY(booz2_navigation_target, waypoints[wp_center]);
  }
  else {
    struct Int32Vect2 pos_diff;
    VECT2_DIFF(pos_diff, booz_ins_enu_pos,waypoints[wp_center]);
    // go back to half metric precision or values are too large
    //INT32_VECT2_RSHIFT(pos_diff,pos_diff,INT32_POS_FRAC/2);
    // store last qdr
    int32_t last_qdr = nav_circle_qdr;
    // compute qdr
    INT32_ATAN2(nav_circle_qdr, pos_diff.y, pos_diff.x);
    // increment circle radians
    int32_t angle_diff = nav_circle_qdr - last_qdr;
    INT32_ANGLE_NORMALIZE(angle_diff);
    nav_circle_radians += angle_diff;

    // direction of rotation
    int8_t sign_radius = radius > 0 ? 1 : -1;
    // absolute radius
    int32_t abs_radius = abs(radius);
    // carrot_angle
    int32_t carrot_angle = ((CARROT_DIST<<INT32_ANGLE_FRAC) / abs_radius);
    Bound(carrot_angle, (INT32_ANGLE_PI / 16), INT32_ANGLE_PI_4);
    carrot_angle = nav_circle_qdr - sign_radius * carrot_angle;
    int32_t s_carrot, c_carrot;
    PPRZ_ITRIG_SIN(s_carrot, carrot_angle);	
    PPRZ_ITRIG_COS(c_carrot, carrot_angle);	
    // compute setpoint
    VECT2_ASSIGN(pos_diff, abs_radius * c_carrot, abs_radius * s_carrot);
    INT32_VECT2_RSHIFT(pos_diff, pos_diff, INT32_TRIG_FRAC);
    VECT2_SUM(booz2_navigation_target, waypoints[wp_center], pos_diff);
  }
  nav_circle_centre = wp_center;
  nav_circle_radius = radius;
  horizontal_mode = HORIZONTAL_MODE_CIRCLE;
}


//#include "stdio.h"
void nav_route(uint8_t wp_start, uint8_t wp_end) {
  struct Int32Vect2 wp_diff,pos_diff;
  VECT2_DIFF(wp_diff, waypoints[wp_end],waypoints[wp_start]);
  VECT2_DIFF(pos_diff, booz_ins_enu_pos,waypoints[wp_start]);
  // go back to metric precision or values are too large
  INT32_VECT2_RSHIFT(wp_diff,wp_diff,INT32_POS_FRAC);
  INT32_VECT2_RSHIFT(pos_diff,pos_diff,INT32_POS_FRAC);
  int32_t leg_length;
  int32_t leg_length2 = Max((wp_diff.x * wp_diff.x + wp_diff.y * wp_diff.y),1);
  INT32_SQRT(leg_length,leg_length2);
  int32_t nav_leg_progress = (pos_diff.x * wp_diff.x + pos_diff.y * wp_diff.y) / leg_length;
  int32_t progress = Max((CARROT_DIST >> INT32_POS_FRAC), 0);
  nav_leg_progress += progress;
  int32_t prog_2 = leg_length;// + progress / 2;
  Bound(nav_leg_progress, 0, prog_2);
  struct Int32Vect2 progress_pos;
  VECT2_SMUL(progress_pos, wp_diff, nav_leg_progress);
  VECT2_SDIV(progress_pos, progress_pos, leg_length);
  INT32_VECT2_LSHIFT(progress_pos,progress_pos,INT32_POS_FRAC);
  VECT2_SUM(booz2_navigation_target,waypoints[wp_start],progress_pos);
  //printf("target %d %d | p %d %d | s %d %d | l %d %d %d\n",
  //    booz2_navigation_target.x,
  //    booz2_navigation_target.y,
  //    progress_pos.x,
  //    progress_pos.y,
  //    waypoints[wp_start].x,
  //    waypoints[wp_start].y,
  //    leg_length, leg_length2, nav_leg_progress);
  //fflush(stdout);

  nav_segment_start = wp_start;
  nav_segment_end = wp_end;
  horizontal_mode = HORIZONTAL_MODE_ROUTE;
}

bool_t nav_approaching_from(uint8_t wp_idx, uint8_t from_idx) {
  int32_t dist_to_point;
  struct Int32Vect2 diff;
  static uint8_t time_at_wp = 0;
  VECT2_DIFF(diff, waypoints[wp_idx], booz_ins_enu_pos);
  INT32_VECT2_RSHIFT(diff,diff,INT32_POS_FRAC);
  INT32_VECT2_NORM(dist_to_point, diff);
  //printf("dist %d | %d %d\n", dist_to_point,diff.x,diff.y);
  //fflush(stdout);
  //if (dist_to_point < (ARRIVED_AT_WAYPOINT >> INT32_POS_FRAC)) return TRUE;
  if (dist_to_point < (ARRIVED_AT_WAYPOINT >> INT32_POS_FRAC)) time_at_wp++;
  else time_at_wp = 0;
  if (time_at_wp > 20) return TRUE;
  if (from_idx > 0 && from_idx < NB_WAYPOINT) {
    struct Int32Vect2 from_diff;
    VECT2_DIFF(from_diff, waypoints[wp_idx],waypoints[from_idx]);
    INT32_VECT2_RSHIFT(from_diff,from_diff,INT32_POS_FRAC);
    return (diff.x * from_diff.x + diff.y * from_diff.y < 0);
  }
  else return FALSE;
}

static inline void nav_set_altitude( void ) {
  static int32_t last_nav_alt = 0;
  if (abs(nav_altitude - last_nav_alt) > (POS_BFP_OF_REAL(0.2))) {
    nav_flight_altitude = nav_altitude;
    last_nav_alt = nav_altitude;
  }
}

/** Reset the geographic reference to the current GPS fix */
unit_t nav_reset_reference( void ) {
  booz_ins_ltp_initialised = FALSE;
  booz_ins_vff_realign = TRUE;
#ifdef USE_HFF
  booz_ins_hff_realign = TRUE;
#endif
  return 0;
}

unit_t nav_reset_alt( void ) {
  booz_ins_vff_realign = TRUE;

#ifdef USE_GPS
  booz_ins_ltp_def.lla.alt = booz_gps_state.lla_pos.alt;
  booz_ins_ltp_def.hmsl = booz_gps_state.hmsl;
#endif

  return 0;
}

void nav_init_stage( void ) {
  INT32_VECT3_COPY(nav_last_point, booz_ins_enu_pos);
  stage_time = 0;
  nav_circle_radians = 0;
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
}

void nav_init_block(void) {
  if (nav_block >= NB_BLOCK)
    nav_block=NB_BLOCK-1;
  nav_stage = 0;
  block_time = 0;
  InitStage();
}

void nav_goto_block(uint8_t b) {
  if (b != nav_block) { /* To avoid a loop in a the current block */
    last_block = nav_block;
    last_stage = nav_stage;
  }
  GotoBlock(b);
}

#include <stdio.h>
void nav_periodic_task() {
  RunOnceEvery(16, { stage_time++;  block_time++; });

  /* from flight_plan.h */
  auto_nav();

  /* run carrot loop */
  booz2_nav_run();

  ground_alt = POS_BFP_OF_REAL((float)booz_ins_ltp_def.hmsl / 100.);

}

#include "downlink.h"
#include "messages.h"
#include "uart.h"
void nav_move_waypoint(uint8_t wp_id, struct EnuCoor_i * new_pos) {
  if (wp_id < nb_waypoint) {
    INT32_VECT3_COPY(waypoints[wp_id],(*new_pos));
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, &wp_id, &(new_pos->x), &(new_pos->y), &(new_pos->z));
  }
}

void nav_update_wp_from_fms(uint8_t _wp) {
  if (fms.enabled && _wp < nb_waypoint) {
    int32_t s_heading, c_heading;
    PPRZ_ITRIG_SIN(s_heading, nav_heading);
    PPRZ_ITRIG_COS(c_heading, nav_heading);
    struct Int32Vect3 dpos;
    dpos.x = (NavFmsMaxHSpeed * fms.input.h_sp.speed.x >> (INT32_SPEED_FRAC - INT32_POS_FRAC)) / (BOOZ2_AP_NAV_RATE * 128);
    dpos.y = (NavFmsMaxHSpeed * fms.input.h_sp.speed.y >> (INT32_SPEED_FRAC - INT32_POS_FRAC)) / (BOOZ2_AP_NAV_RATE * 128);
    dpos.z = (NavFmsMaxVSpeed * fms.input.v_sp.climb >> (INT32_SPEED_FRAC - INT32_POS_FRAC)) / (BOOZ2_AP_NAV_RATE * 128);
    waypoints[_wp].x += (s_heading * dpos.x + c_heading * dpos.y) >> INT32_TRIG_FRAC;
    waypoints[_wp].y += (c_heading * dpos.x - s_heading * dpos.y) >> INT32_TRIG_FRAC;
    waypoints[_wp].z += dpos.z;
    int32_t dheading = (NavFmsMaxHeadingRate * fms.input.h_sp.speed.z >> (INT32_RATE_FRAC - INT32_ANGLE_FRAC)) / (BOOZ2_AP_NAV_RATE * 128);
    if (dheading != 0) {
      nav_heading += dheading;
      INT32_COURSE_NORMALIZE(nav_heading);
    }
    RunOnceEvery(10,DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, &_wp, &(waypoints[_wp].x), &(waypoints[_wp].y), &(waypoints[_wp].z)));
  }
}

bool_t nav_detect_ground(void) {
  if (!booz2_autopilot_detect_ground) return FALSE;
  booz2_autopilot_detect_ground = FALSE;
  return TRUE;
}

void nav_home(void) {}

