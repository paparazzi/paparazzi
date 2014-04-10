/*
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

/**
 * @file firmwares/rotorcraft/navigation.c
 *
 * Rotorcraft navigation functions.
 */


#define NAV_C

#include "firmwares/rotorcraft/navigation.h"

#include "pprz_debug.h"
#include "subsystems/gps.h" // needed by auto_nav from the flight plan
#include "subsystems/ins.h"
#include "state.h"

#include "firmwares/rotorcraft/autopilot.h"
#include "generated/modules.h"
#include "generated/flight_plan.h"

/* for default GUIDANCE_H_USE_REF */
#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "math/pprz_algebra_int.h"

#include "subsystems/datalink/downlink.h"
#include "messages.h"
#include "mcu_periph/uart.h"

const uint8_t nb_waypoint = NB_WAYPOINT;
struct EnuCoor_i waypoints[NB_WAYPOINT];

struct EnuCoor_i navigation_target;
struct EnuCoor_i navigation_carrot;

struct EnuCoor_i nav_last_point;

uint8_t last_wp __attribute__ ((unused));

int32_t ground_alt;

/** Maximum distance from HOME waypoint before going into failsafe mode */
#ifndef FAILSAFE_MODE_DISTANCE
#define FAILSAFE_MODE_DISTANCE (1.5*MAX_DIST_FROM_HOME)
#endif

const float max_dist_from_home = MAX_DIST_FROM_HOME;
const float max_dist2_from_home = MAX_DIST_FROM_HOME * MAX_DIST_FROM_HOME;
float failsafe_mode_dist2 = FAILSAFE_MODE_DISTANCE * FAILSAFE_MODE_DISTANCE;
float dist2_to_home;
bool_t too_far_from_home;

uint8_t horizontal_mode;
uint8_t nav_segment_start, nav_segment_end;
uint8_t nav_circle_centre;
int32_t nav_circle_radius, nav_circle_qdr, nav_circle_radians;

int32_t nav_leg_progress;
int32_t nav_leg_length;

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

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_nav_status(void) {
  DOWNLINK_SEND_ROTORCRAFT_NAV_STATUS(DefaultChannel, DefaultDevice,
      &block_time, &stage_time,
      &nav_block, &nav_stage,
      &horizontal_mode);
  if (horizontal_mode == HORIZONTAL_MODE_ROUTE) {
    float sx = POS_FLOAT_OF_BFP(waypoints[nav_segment_start].x);
    float sy = POS_FLOAT_OF_BFP(waypoints[nav_segment_start].y);
    float ex = POS_FLOAT_OF_BFP(waypoints[nav_segment_end].x);
    float ey = POS_FLOAT_OF_BFP(waypoints[nav_segment_end].y);
    DOWNLINK_SEND_SEGMENT(DefaultChannel, DefaultDevice, &sx, &sy, &ex, &ey);
  }
  else if (horizontal_mode == HORIZONTAL_MODE_CIRCLE) {
    float cx = POS_FLOAT_OF_BFP(waypoints[nav_circle_centre].x);
    float cy = POS_FLOAT_OF_BFP(waypoints[nav_circle_centre].y);
    float r = POS_FLOAT_OF_BFP(nav_circle_radius);
    DOWNLINK_SEND_CIRCLE(DefaultChannel, DefaultDevice, &cx, &cy, &r);
  }
}

static void send_wp_moved(void) {
  static uint8_t i;
  i++; if (i >= nb_waypoint) i = 0;
  DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice,
      &i,
      &(waypoints[i].x),
      &(waypoints[i].y),
      &(waypoints[i].z));
}
#endif

void nav_init(void) {
  // convert to
  const struct EnuCoor_f wp_tmp_float[NB_WAYPOINT] = WAYPOINTS;
  // init int32 waypoints
  uint8_t i = 0;
  for (i = 0; i < nb_waypoint; i++) {
    waypoints[i].x = POS_BFP_OF_REAL(wp_tmp_float[i].x);
    waypoints[i].y = POS_BFP_OF_REAL(wp_tmp_float[i].y);
    waypoints[i].z = POS_BFP_OF_REAL((wp_tmp_float[i].z - GROUND_ALT));
  }
  nav_block = 0;
  nav_stage = 0;
  ground_alt = POS_BFP_OF_REAL(GROUND_ALT);
  nav_altitude = POS_BFP_OF_REAL(SECURITY_HEIGHT);
  nav_flight_altitude = nav_altitude;
  flight_altitude = SECURITY_ALT;
  INT32_VECT3_COPY(navigation_target, waypoints[WP_HOME]);
  INT32_VECT3_COPY(navigation_carrot, waypoints[WP_HOME]);

  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  vertical_mode = VERTICAL_MODE_ALT;

  nav_roll = 0;
  nav_pitch = 0;
  nav_heading = 0;
  nav_course = 0;
  nav_radius = DEFAULT_CIRCLE_RADIUS;
  nav_throttle = 0;
  nav_climb = 0;
  nav_leg_progress = 0;
  nav_leg_length = 1;

  too_far_from_home = FALSE;
  dist2_to_home = 0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "ROTORCRAFT_NAV_STATUS", send_nav_status);
  register_periodic_telemetry(DefaultPeriodic, "WP_MOVED", send_wp_moved);
#endif
}

static inline void nav_advance_carrot(void) {
  struct EnuCoor_i *pos = stateGetPositionEnu_i();
  /* compute a vector to the waypoint */
  struct Int32Vect2 path_to_waypoint;
  VECT2_DIFF(path_to_waypoint, navigation_target, *pos);

  /* saturate it */
  VECT2_STRIM(path_to_waypoint, -(1<<15), (1<<15));

  int32_t dist_to_waypoint;
  INT32_VECT2_NORM(dist_to_waypoint, path_to_waypoint);

  if (dist_to_waypoint < CLOSE_TO_WAYPOINT) {
    VECT2_COPY(navigation_carrot, navigation_target);
  }
  else {
    struct Int32Vect2 path_to_carrot;
    VECT2_SMUL(path_to_carrot, path_to_waypoint, CARROT_DIST);
    VECT2_SDIV(path_to_carrot, path_to_carrot, dist_to_waypoint);
    VECT2_SUM(navigation_carrot, path_to_carrot, *pos);
  }
}

void nav_run(void) {

#if GUIDANCE_H_USE_REF
  // if GUIDANCE_H_USE_REF, CARROT_DIST is not used
  VECT2_COPY(navigation_carrot, navigation_target);
#else
  nav_advance_carrot();
#endif

  nav_set_altitude();
}

void nav_circle(uint8_t wp_center, int32_t radius) {
  if (radius == 0) {
    VECT2_COPY(navigation_target, waypoints[wp_center]);
  }
  else {
    struct Int32Vect2 pos_diff;
    VECT2_DIFF(pos_diff, *stateGetPositionEnu_i(), waypoints[wp_center]);
    // go back to half metric precision or values are too large
    //INT32_VECT2_RSHIFT(pos_diff,pos_diff,INT32_POS_FRAC/2);
    // store last qdr
    int32_t last_qdr = nav_circle_qdr;
    // compute qdr
    INT32_ATAN2(nav_circle_qdr, pos_diff.y, pos_diff.x);
    // increment circle radians
    if (nav_circle_radians != 0) {
      int32_t angle_diff = nav_circle_qdr - last_qdr;
      INT32_ANGLE_NORMALIZE(angle_diff);
      nav_circle_radians += angle_diff;
    }
    else {
      // Smallest angle to increment at next step
      nav_circle_radians = 1;
    }

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
    VECT2_SUM(navigation_target, waypoints[wp_center], pos_diff);
  }
  nav_circle_centre = wp_center;
  nav_circle_radius = radius;
  horizontal_mode = HORIZONTAL_MODE_CIRCLE;
}


//#include "stdio.h"
void nav_route(uint8_t wp_start, uint8_t wp_end) {
  struct Int32Vect2 wp_diff,pos_diff;
  VECT2_DIFF(wp_diff, waypoints[wp_end],waypoints[wp_start]);
  VECT2_DIFF(pos_diff, *stateGetPositionEnu_i(), waypoints[wp_start]);
  // go back to metric precision or values are too large
  INT32_VECT2_RSHIFT(wp_diff,wp_diff,INT32_POS_FRAC);
  INT32_VECT2_RSHIFT(pos_diff,pos_diff,INT32_POS_FRAC);
  int32_t leg_length2 = Max((wp_diff.x * wp_diff.x + wp_diff.y * wp_diff.y),1);
  INT32_SQRT(nav_leg_length,leg_length2);
  nav_leg_progress = (pos_diff.x * wp_diff.x + pos_diff.y * wp_diff.y) / nav_leg_length;
  int32_t progress = Max((CARROT_DIST >> INT32_POS_FRAC), 0);
  nav_leg_progress += progress;
  int32_t prog_2 = nav_leg_length;// + progress / 2;
  Bound(nav_leg_progress, 0, prog_2);
  struct Int32Vect2 progress_pos;
  VECT2_SMUL(progress_pos, wp_diff, nav_leg_progress);
  VECT2_SDIV(progress_pos, progress_pos, nav_leg_length);
  INT32_VECT2_LSHIFT(progress_pos,progress_pos,INT32_POS_FRAC);
  VECT2_SUM(navigation_target,waypoints[wp_start],progress_pos);
  //printf("target %d %d | p %d %d | s %d %d | l %d %d %d\n",
  //    navigation_target.x,
  //    navigation_target.y,
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
  VECT2_DIFF(diff, waypoints[wp_idx], *stateGetPositionEnu_i());
  INT32_VECT2_RSHIFT(diff, diff, INT32_POS_FRAC);
  INT32_VECT2_NORM(dist_to_point, diff);
  //printf("dist %d | %d %d\n", dist_to_point,diff.x,diff.y);
  //fflush(stdout);
  //if (dist_to_point < (ARRIVED_AT_WAYPOINT >> INT32_POS_FRAC)) return TRUE;
  if (dist_to_point < (ARRIVED_AT_WAYPOINT >> INT32_POS_FRAC)) time_at_wp++;
  else time_at_wp = 0;
  if (time_at_wp > 20) return TRUE;
  if (from_idx > 0 && from_idx < NB_WAYPOINT) {
    struct Int32Vect2 from_diff;
    VECT2_DIFF(from_diff, waypoints[wp_idx], waypoints[from_idx]);
    INT32_VECT2_RSHIFT(from_diff, from_diff, INT32_POS_FRAC);
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
  ins_reset_local_origin();
  return 0;
}

unit_t nav_reset_alt( void ) {
  ins_reset_altitude_ref();
  return 0;
}

void nav_init_stage( void ) {
  INT32_VECT3_COPY(nav_last_point, *stateGetPositionEnu_i());
  stage_time = 0;
  nav_circle_radians = 0;
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
}

#include <stdio.h>
void nav_periodic_task(void) {
  RunOnceEvery(16, { stage_time++;  block_time++; });

  compute_dist2_to_home();
  /* from flight_plan.h */
  auto_nav();

  /* run carrot loop */
  nav_run();

  ground_alt = POS_BFP_OF_REAL(state.ned_origin_f.hmsl);
}

void nav_move_waypoint_lla(uint8_t wp_id, struct LlaCoor_i* new_lla_pos) {
  if (stateIsLocalCoordinateValid()) {
    struct EnuCoor_i enu;
    enu_of_lla_point_i(&enu, &state.ned_origin_i, new_lla_pos);
    enu.x = POS_BFP_OF_REAL(enu.x)/100;
    enu.y = POS_BFP_OF_REAL(enu.y)/100;
    enu.z = POS_BFP_OF_REAL(enu.z)/100;
    nav_move_waypoint(wp_id, &enu);
  }
}

void nav_move_waypoint(uint8_t wp_id, struct EnuCoor_i * new_pos) {
  if (wp_id < nb_waypoint) {
    INT32_VECT3_COPY(waypoints[wp_id],(*new_pos));
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &(new_pos->x),
                               &(new_pos->y), &(new_pos->z));
  }
}

void navigation_update_wp_from_speed(uint8_t wp, struct Int16Vect3 speed_sp, int16_t heading_rate_sp ) {
  //  MY_ASSERT(wp < nb_waypoint); FIXME
  int32_t s_heading, c_heading;
  PPRZ_ITRIG_SIN(s_heading, nav_heading);
  PPRZ_ITRIG_COS(c_heading, nav_heading);
  // FIXME : scale POS to SPEED
  struct Int32Vect3 delta_pos;
  VECT3_SDIV(delta_pos, speed_sp, NAV_FREQ); /* fixme :make sure the division is really a >> */
  INT32_VECT3_RSHIFT(delta_pos, delta_pos, (INT32_SPEED_FRAC-INT32_POS_FRAC));
  waypoints[wp].x += (s_heading * delta_pos.x + c_heading * delta_pos.y) >> INT32_TRIG_FRAC;
  waypoints[wp].y += (c_heading * delta_pos.x - s_heading * delta_pos.y) >> INT32_TRIG_FRAC;
  waypoints[wp].z += delta_pos.z;
  int32_t delta_heading = heading_rate_sp / NAV_FREQ;
  delta_heading = delta_heading >> (INT32_SPEED_FRAC-INT32_POS_FRAC);
  nav_heading += delta_heading;

  INT32_COURSE_NORMALIZE(nav_heading);
  RunOnceEvery(10,DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp, &(waypoints[wp].x), &(waypoints[wp].y), &(waypoints[wp].z)));
}

bool_t nav_detect_ground(void) {
  if (!autopilot_ground_detected) return FALSE;
  autopilot_ground_detected = FALSE;
  return TRUE;
}

bool_t nav_is_in_flight(void) {
  return autopilot_in_flight;
}

/** Home mode navigation */
void nav_home(void) {
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  INT32_VECT3_COPY(navigation_target, waypoints[WP_HOME]);

  vertical_mode = VERTICAL_MODE_ALT;
  nav_altitude = waypoints[WP_HOME].z;
  nav_flight_altitude = nav_altitude;

  compute_dist2_to_home();

  /* run carrot loop */
  nav_run();
}

/** Computes squared distance to the HOME waypoint potentially sets
 * #too_far_from_home
 */
void compute_dist2_to_home(void) {
  struct EnuCoor_i* pos = stateGetPositionEnu_i();
  struct Int32Vect2 home_d;
  VECT2_DIFF(home_d, waypoints[WP_HOME], *pos);
  INT32_VECT2_RSHIFT(home_d, home_d, INT32_POS_FRAC);
  dist2_to_home = (float)(home_d.x * home_d.x + home_d.y * home_d.y);
  too_far_from_home = dist2_to_home > max_dist2_from_home;
}
