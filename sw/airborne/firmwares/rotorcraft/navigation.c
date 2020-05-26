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

#include "autopilot.h"
#include "generated/modules.h"
#include "generated/flight_plan.h"

/* for default GUIDANCE_H_USE_REF */
#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "math/pprz_algebra_int.h"

#include "subsystems/datalink/downlink.h"
#include "pprzlink/messages.h"
#include "mcu_periph/uart.h"



/** default nav_circle_radius in meters */
#ifndef DEFAULT_CIRCLE_RADIUS
#define DEFAULT_CIRCLE_RADIUS 5.
#endif

#ifndef NAV_CLIMB_VSPEED
#define NAV_CLIMB_VSPEED 0.5
#endif

#ifndef NAV_DESCEND_VSPEED
#define NAV_DESCEND_VSPEED -0.8
#endif

/** minimum horizontal distance to waypoint to mark as arrived */
#ifndef ARRIVED_AT_WAYPOINT
#define ARRIVED_AT_WAYPOINT 3.0
#endif

/** Maximum distance from HOME waypoint before going into failsafe mode */
#ifndef FAILSAFE_MODE_DISTANCE
#define FAILSAFE_MODE_DISTANCE (1.5*MAX_DIST_FROM_HOME)
#endif

#define CLOSE_TO_WAYPOINT (15 << INT32_POS_FRAC)
#define CARROT_DIST (12 << INT32_POS_FRAC)

const float max_dist_from_home = MAX_DIST_FROM_HOME;
const float max_dist2_from_home = MAX_DIST_FROM_HOME * MAX_DIST_FROM_HOME;
float failsafe_mode_dist2 = FAILSAFE_MODE_DISTANCE * FAILSAFE_MODE_DISTANCE;
float dist2_to_home;
bool too_far_from_home;

float dist2_to_wp;

struct EnuCoor_i navigation_target;
struct EnuCoor_i navigation_carrot;

struct EnuCoor_i nav_last_point;

uint8_t last_wp UNUSED;

bool exception_flag[10] = {0}; //exception flags that can be used in the flight plan

uint8_t horizontal_mode;

int32_t nav_leg_progress;
uint32_t nav_leg_length;

bool nav_survey_active;

int32_t nav_roll, nav_pitch;
int32_t nav_heading;
int32_t nav_cmd_roll, nav_cmd_pitch, nav_cmd_yaw;
float nav_radius;
float nav_climb_vspeed, nav_descend_vspeed;

uint8_t vertical_mode;
uint32_t nav_throttle;
int32_t nav_climb, nav_altitude, nav_flight_altitude;
float flight_altitude;

/* nav_circle variables */
struct EnuCoor_i nav_circle_center;
int32_t nav_circle_radius, nav_circle_qdr, nav_circle_radians;

/* nav_route variables */
struct EnuCoor_i nav_segment_start, nav_segment_end;


static inline void nav_set_altitude(void);


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

void set_exception_flag(uint8_t flag_num)
{
  exception_flag[flag_num] = 1;
}

static void send_segment(struct transport_tx *trans, struct link_device *dev)
{
  float sx = POS_FLOAT_OF_BFP(nav_segment_start.x);
  float sy = POS_FLOAT_OF_BFP(nav_segment_start.y);
  float ex = POS_FLOAT_OF_BFP(nav_segment_end.x);
  float ey = POS_FLOAT_OF_BFP(nav_segment_end.y);
  pprz_msg_send_SEGMENT(trans, dev, AC_ID, &sx, &sy, &ex, &ey);
}

static void send_circle(struct transport_tx *trans, struct link_device *dev)
{
  float cx = POS_FLOAT_OF_BFP(nav_circle_center.x);
  float cy = POS_FLOAT_OF_BFP(nav_circle_center.y);
  float r = POS_FLOAT_OF_BFP(nav_circle_radius);
  pprz_msg_send_CIRCLE(trans, dev, AC_ID, &cx, &cy, &r);
}

static void send_nav_status(struct transport_tx *trans, struct link_device *dev)
{
  float dist_home = sqrtf(dist2_to_home);
  float dist_wp = sqrtf(dist2_to_wp);
  pprz_msg_send_ROTORCRAFT_NAV_STATUS(trans, dev, AC_ID,
                                      &block_time, &stage_time,
                                      &dist_home, &dist_wp,
                                      &nav_block, &nav_stage,
                                      &horizontal_mode);
  if (horizontal_mode == HORIZONTAL_MODE_ROUTE) {
    send_segment(trans, dev);
  } else if (horizontal_mode == HORIZONTAL_MODE_CIRCLE) {
    send_circle(trans, dev);
  }
}

static void send_wp_moved(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t i;
  i++;
  if (i >= nb_waypoint) { i = 0; }
  pprz_msg_send_WP_MOVED_ENU(trans, dev, AC_ID,
                             &i,
                             &(waypoints[i].enu_i.x),
                             &(waypoints[i].enu_i.y),
                             &(waypoints[i].enu_i.z));
}
#endif

void nav_init(void)
{
  waypoints_init();

  nav_block = 0;
  nav_stage = 0;
  nav_altitude = POS_BFP_OF_REAL(SECURITY_HEIGHT);
  nav_flight_altitude = nav_altitude;
  flight_altitude = SECURITY_ALT;
  VECT3_COPY(navigation_target, waypoints[WP_HOME].enu_i);
  VECT3_COPY(navigation_carrot, waypoints[WP_HOME].enu_i);

  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  vertical_mode = VERTICAL_MODE_ALT;

  nav_roll = 0;
  nav_pitch = 0;
  nav_heading = 0;
  nav_cmd_roll = 0;
  nav_cmd_pitch = 0;
  nav_cmd_yaw = 0;
  nav_radius = DEFAULT_CIRCLE_RADIUS;
  nav_climb_vspeed = NAV_CLIMB_VSPEED;
  nav_descend_vspeed = NAV_DESCEND_VSPEED;
  nav_throttle = 0;
  nav_climb = 0;
  nav_leg_progress = 0;
  nav_leg_length = 1;

  too_far_from_home = false;
  dist2_to_home = 0;
  dist2_to_wp = 0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_NAV_STATUS, send_nav_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WP_MOVED, send_wp_moved);
#endif

  // generated init function
  auto_nav_init();
}

static inline void UNUSED nav_advance_carrot(void)
{
  struct EnuCoor_i *pos = stateGetPositionEnu_i();
  /* compute a vector to the waypoint */
  struct Int32Vect2 path_to_waypoint;
  VECT2_DIFF(path_to_waypoint, navigation_target, *pos);

  /* saturate it */
  VECT2_STRIM(path_to_waypoint, -(1 << 15), (1 << 15));

  int32_t dist_to_waypoint = int32_vect2_norm(&path_to_waypoint);

  if (dist_to_waypoint < CLOSE_TO_WAYPOINT) {
    VECT2_COPY(navigation_carrot, navigation_target);
  } else {
    struct Int32Vect2 path_to_carrot;
    VECT2_SMUL(path_to_carrot, path_to_waypoint, CARROT_DIST);
    VECT2_SDIV(path_to_carrot, path_to_carrot, dist_to_waypoint);
    VECT2_SUM(navigation_carrot, path_to_carrot, *pos);
  }
}

void nav_run(void)
{

#if GUIDANCE_H_USE_REF
  // if GUIDANCE_H_USE_REF, CARROT_DIST is not used
  VECT2_COPY(navigation_carrot, navigation_target);
#else
  nav_advance_carrot();
#endif

  nav_set_altitude();
}


bool nav_approaching_from(struct EnuCoor_i *wp, struct EnuCoor_i *from, int16_t approaching_time)
{
  float dist_to_point;
  struct Int32Vect2 diff;
  struct EnuCoor_i *pos = stateGetPositionEnu_i();

  /* if an approaching_time is given, estimate diff after approching_time secs */
  if (approaching_time > 0) {
    struct Int32Vect2 estimated_pos;
    struct Int32Vect2 estimated_progress;
    struct EnuCoor_i *speed = stateGetSpeedEnu_i();
    VECT2_SMUL(estimated_progress, *speed, approaching_time);
    INT32_VECT2_RSHIFT(estimated_progress, estimated_progress, (INT32_SPEED_FRAC - INT32_POS_FRAC));
    VECT2_SUM(estimated_pos, *pos, estimated_progress);
    VECT2_DIFF(diff, *wp, estimated_pos);
  }
  /* else use current position */
  else {
    VECT2_DIFF(diff, *wp, *pos);
  }
  /* compute distance of estimated/current pos to target wp
   * POS_FRAC resolution
   * convert to float to compute the norm without overflow in 32bit
   */
  struct FloatVect2 diff_f = {POS_FLOAT_OF_BFP(diff.x), POS_FLOAT_OF_BFP(diff.y)};
  dist_to_point = float_vect2_norm(&diff_f);

  /* return TRUE if we have arrived */
  if (dist_to_point < ARRIVED_AT_WAYPOINT) {
    return true;
  }

  /* if coming from a valid waypoint */
  if (from != NULL) {
    /* return TRUE if normal line at the end of the segment is crossed */
    struct Int32Vect2 from_diff;
    VECT2_DIFF(from_diff, *wp, *from);
    struct FloatVect2 from_diff_f = {POS_FLOAT_OF_BFP(from_diff.x), POS_FLOAT_OF_BFP(from_diff.y)};
    return (diff_f.x * from_diff_f.x + diff_f.y * from_diff_f.y < 0);
  }

  return false;
}

bool nav_check_wp_time(struct EnuCoor_i *wp, uint16_t stay_time)
{
  uint16_t time_at_wp;
  float dist_to_point;
  static uint16_t wp_entry_time = 0;
  static bool wp_reached = false;
  static struct EnuCoor_i wp_last = { 0, 0, 0 };
  struct Int32Vect2 diff;

  if ((wp_last.x != wp->x) || (wp_last.y != wp->y)) {
    wp_reached = false;
    wp_last = *wp;
  }

  VECT2_DIFF(diff, *wp, *stateGetPositionEnu_i());
  struct FloatVect2 diff_f = {POS_FLOAT_OF_BFP(diff.x), POS_FLOAT_OF_BFP(diff.y)};
  dist_to_point = float_vect2_norm(&diff_f);
  if (dist_to_point < ARRIVED_AT_WAYPOINT) {
    if (!wp_reached) {
      wp_reached = true;
      wp_entry_time = autopilot.flight_time;
      time_at_wp = 0;
    } else {
      time_at_wp = autopilot.flight_time - wp_entry_time;
    }
  } else {
    time_at_wp = 0;
    wp_reached = false;
  }
  if (time_at_wp > stay_time) {
    INT_VECT3_ZERO(wp_last);
    return true;
  }
  return false;
}

static inline void nav_set_altitude(void)
{
  static int32_t last_nav_alt = 0;
  if (abs(nav_altitude - last_nav_alt) > (POS_BFP_OF_REAL(0.2))) {
    nav_flight_altitude = nav_altitude;
    last_nav_alt = nav_altitude;
  }
}


/** Reset the geographic reference to the current GPS fix */
void nav_reset_reference(void)
{
  ins_reset_local_origin();
  /* update local ENU coordinates of global waypoints */
  waypoints_localize_all();
}

void nav_reset_alt(void)
{
  ins_reset_altitude_ref();
  waypoints_localize_all();
}

void nav_init_stage(void)
{
  VECT3_COPY(nav_last_point, *stateGetPositionEnu_i());
  stage_time = 0;
  nav_circle_radians = 0;
}

#include <stdio.h>
void nav_periodic_task(void)
{
  RunOnceEvery(NAV_FREQ, { stage_time++;  block_time++; });

  nav_survey_active = false;

  dist2_to_wp = 0;

  /* from flight_plan.h */
  auto_nav();

  /* run carrot loop */
  nav_run();
}

void navigation_update_wp_from_speed(uint8_t wp, struct Int16Vect3 speed_sp, int16_t heading_rate_sp)
{
  //  MY_ASSERT(wp < nb_waypoint); FIXME
  int32_t s_heading, c_heading;
  PPRZ_ITRIG_SIN(s_heading, nav_heading);
  PPRZ_ITRIG_COS(c_heading, nav_heading);
  // FIXME : scale POS to SPEED
  struct Int32Vect3 delta_pos;
  VECT3_SDIV(delta_pos, speed_sp, NAV_FREQ); /* fixme :make sure the division is really a >> */
  INT32_VECT3_RSHIFT(delta_pos, delta_pos, (INT32_SPEED_FRAC - INT32_POS_FRAC));
  waypoints[wp].enu_i.x += (s_heading * delta_pos.x + c_heading * delta_pos.y) >> INT32_TRIG_FRAC;
  waypoints[wp].enu_i.y += (c_heading * delta_pos.x - s_heading * delta_pos.y) >> INT32_TRIG_FRAC;
  waypoints[wp].enu_i.z += delta_pos.z;
  int32_t delta_heading = heading_rate_sp / NAV_FREQ;
  delta_heading = delta_heading >> (INT32_SPEED_FRAC - INT32_POS_FRAC);
  nav_heading += delta_heading;

  INT32_COURSE_NORMALIZE(nav_heading);
  RunOnceEvery(10, DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp,
               &(waypoints[wp].enu_i.x),
               &(waypoints[wp].enu_i.y),
               &(waypoints[wp].enu_i.z)));
}

bool nav_detect_ground(void)
{
  if (!autopilot.ground_detected) { return false; }
  autopilot.ground_detected = false;
  return true;
}

bool nav_is_in_flight(void)
{
  return autopilot_in_flight();
}

/** Home mode navigation */
void nav_home(void)
{
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  VECT3_COPY(navigation_target, waypoints[WP_HOME].enu_i);

  vertical_mode = VERTICAL_MODE_ALT;
  nav_altitude = waypoints[WP_HOME].enu_i.z;
  nav_flight_altitude = nav_altitude;

  dist2_to_wp = dist2_to_home;

  /* run carrot loop */
  nav_run();
}

/** Set manual roll, pitch and yaw without stabilization
 *
 * @param[in] roll command in pprz scale (int32_t)
 * @param[in] pitch command in pprz scale (int32_t)
 * @param[in] yaw command in pprz scale (int32_t)
 *
 * This function allows to directly set commands from the flight plan,
 * if in nav_manual mode.
 * This is for instance useful for helicopters during the spinup
 */
void nav_set_manual(int32_t roll, int32_t pitch, int32_t yaw)
{
  horizontal_mode = HORIZONTAL_MODE_MANUAL;
  nav_cmd_roll = roll;
  nav_cmd_pitch = pitch;
  nav_cmd_yaw = yaw;
}

/** Returns squared horizontal distance to given point */
float get_dist2_to_point(struct EnuCoor_i *p)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  struct FloatVect2 pos_diff;
  pos_diff.x = POS_FLOAT_OF_BFP(p->x) - pos->x;
  pos_diff.y = POS_FLOAT_OF_BFP(p->y) - pos->y;
  return pos_diff.x * pos_diff.x + pos_diff.y * pos_diff.y;
}

/** Returns squared horizontal distance to given waypoint */
float get_dist2_to_waypoint(uint8_t wp_id)
{
  return get_dist2_to_point(&waypoints[wp_id].enu_i);
}

/** Computes squared distance to the HOME waypoint potentially sets
 * #too_far_from_home
 */
void compute_dist2_to_home(void)
{
  dist2_to_home = get_dist2_to_waypoint(WP_HOME);
  too_far_from_home = dist2_to_home > max_dist2_from_home;
#ifdef InGeofenceSector
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  too_far_from_home = too_far_from_home || !(InGeofenceSector(pos->x, pos->y));
#endif
}

/** Set nav_heading in radians. */
void nav_set_heading_rad(float rad)
{
  nav_heading = ANGLE_BFP_OF_REAL(rad);
  INT32_COURSE_NORMALIZE(nav_heading);
}

/** Set nav_heading in degrees. */
void nav_set_heading_deg(float deg)
{
  nav_set_heading_rad(RadOfDeg(deg));
}

/** Set heading to point towards x,y position in local coordinates */
void nav_set_heading_towards(float x, float y)
{
  struct FloatVect2 target = {x, y};
  struct FloatVect2 pos_diff;
  VECT2_DIFF(pos_diff, target, *stateGetPositionEnu_f());
  // don't change heading if closer than 0.5m to target
  if (VECT2_NORM2(pos_diff) > 0.25) {
    float heading_f = atan2f(pos_diff.x, pos_diff.y);
    nav_heading = ANGLE_BFP_OF_REAL(heading_f);
  }
}

/** Set heading in the direction of a waypoint */
void nav_set_heading_towards_waypoint(uint8_t wp)
{
  nav_set_heading_towards(WaypointX(wp), WaypointY(wp));
}

/** Set heading in the direction of the target*/
void nav_set_heading_towards_target(void)
{
  nav_set_heading_towards(POS_FLOAT_OF_BFP(navigation_target.x),
                          POS_FLOAT_OF_BFP(navigation_target.y));
}

/** Set heading to the current yaw angle */
void nav_set_heading_current(void)
{
  nav_heading = stateGetNedToBodyEulers_i()->psi;
}

void nav_set_failsafe(void)
{
  autopilot_set_mode(AP_MODE_FAILSAFE);
}


/***********************************************************
 * built in navigation routines
 **********************************************************/

void nav_circle(struct EnuCoor_i *wp_center, int32_t radius)
{
  if (radius == 0) {
    VECT2_COPY(navigation_target, *wp_center);
    dist2_to_wp = get_dist2_to_point(wp_center);
  } else {
    struct Int32Vect2 pos_diff;
    VECT2_DIFF(pos_diff, *stateGetPositionEnu_i(), *wp_center);
    // go back to half metric precision or values are too large
    //INT32_VECT2_RSHIFT(pos_diff,pos_diff,INT32_POS_FRAC/2);
    // store last qdr
    int32_t last_qdr = nav_circle_qdr;
    // compute qdr
    nav_circle_qdr = int32_atan2(pos_diff.y, pos_diff.x);
    // increment circle radians
    if (nav_circle_radians != 0) {
      int32_t angle_diff = nav_circle_qdr - last_qdr;
      INT32_ANGLE_NORMALIZE(angle_diff);
      nav_circle_radians += angle_diff;
    } else {
      // Smallest angle to increment at next step
      nav_circle_radians = 1;
    }

    // direction of rotation
    int8_t sign_radius = radius > 0 ? 1 : -1;
    // absolute radius
    int32_t abs_radius = abs(radius);
    // carrot_angle
    int32_t carrot_angle = ((CARROT_DIST << INT32_ANGLE_FRAC) / abs_radius);
    Bound(carrot_angle, (INT32_ANGLE_PI / 16), INT32_ANGLE_PI_4);
    carrot_angle = nav_circle_qdr - sign_radius * carrot_angle;
    int32_t s_carrot, c_carrot;
    PPRZ_ITRIG_SIN(s_carrot, carrot_angle);
    PPRZ_ITRIG_COS(c_carrot, carrot_angle);
    // compute setpoint
    VECT2_ASSIGN(pos_diff, abs_radius * c_carrot, abs_radius * s_carrot);
    INT32_VECT2_RSHIFT(pos_diff, pos_diff, INT32_TRIG_FRAC);
    VECT2_SUM(navigation_target, *wp_center, pos_diff);
  }
  nav_circle_center = *wp_center;
  nav_circle_radius = radius;
  horizontal_mode = HORIZONTAL_MODE_CIRCLE;
}


void nav_route(struct EnuCoor_i *wp_start, struct EnuCoor_i *wp_end)
{
  struct Int32Vect2 wp_diff, pos_diff, wp_diff_prec;
  VECT2_DIFF(wp_diff, *wp_end, *wp_start);
  VECT2_DIFF(pos_diff, *stateGetPositionEnu_i(), *wp_start);
  // go back to metric precision or values are too large
  VECT2_COPY(wp_diff_prec, wp_diff);
  INT32_VECT2_RSHIFT(wp_diff, wp_diff, INT32_POS_FRAC);
  INT32_VECT2_RSHIFT(pos_diff, pos_diff, INT32_POS_FRAC);
  uint32_t leg_length2 = Max((wp_diff.x * wp_diff.x + wp_diff.y * wp_diff.y), 1);
  nav_leg_length = int32_sqrt(leg_length2);
  nav_leg_progress = (pos_diff.x * wp_diff.x + pos_diff.y * wp_diff.y) / (int32_t)nav_leg_length;
  int32_t progress = Max((CARROT_DIST >> INT32_POS_FRAC), 0);
  nav_leg_progress += progress;
  int32_t prog_2 = nav_leg_length;
  Bound(nav_leg_progress, 0, prog_2);
  struct Int32Vect2 progress_pos;
  VECT2_SMUL(progress_pos, wp_diff_prec, ((float)nav_leg_progress) / nav_leg_length);
  VECT2_SUM(navigation_target, *wp_start, progress_pos);

  nav_segment_start = *wp_start;
  nav_segment_end = *wp_end;
  horizontal_mode = HORIZONTAL_MODE_ROUTE;

  dist2_to_wp = get_dist2_to_point(wp_end);
}


/************** Oval Navigation **********************************************/

#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif

enum oval_status { OR12, OC2, OR21, OC1 };
enum oval_status oval_status;
uint8_t nav_oval_count;

void nav_oval_init(void)
{
  oval_status = OC2;
  nav_oval_count = 0;
}

/**
 * Navigation along a figure O. One side leg is defined by waypoints [p1] and [p2].
 * The navigation goes through 4 states: OC1 (half circle next to [p1]),
 * OR21 (route [p2] to [p1], OC2 (half circle next to [p2]) and OR12 (opposite leg).
 * Initial state is the route along the desired segment (OC2).
 */
void nav_oval(uint8_t p1, uint8_t p2, float radius)
{
  radius = - radius; /* Historical error ? */
  int32_t alt = waypoints[p1].enu_i.z;
  waypoints[p2].enu_i.z = alt;

  float p2_p1_x = waypoints[p1].enu_f.x - waypoints[p2].enu_f.x;
  float p2_p1_y = waypoints[p1].enu_f.y - waypoints[p2].enu_f.y;
  float d = sqrtf(p2_p1_x * p2_p1_x + p2_p1_y * p2_p1_y);

  /* Unit vector from p1 to p2 */
  int32_t u_x = POS_BFP_OF_REAL(p2_p1_x / d);
  int32_t u_y = POS_BFP_OF_REAL(p2_p1_y / d);

  /* The half circle centers and the other leg */
  struct EnuCoor_i p1_center = { waypoints[p1].enu_i.x + radius * -u_y,
           waypoints[p1].enu_i.y + radius * u_x,
           alt
  };
  struct EnuCoor_i p1_out = { waypoints[p1].enu_i.x + 2 * radius * -u_y,
           waypoints[p1].enu_i.y + 2 * radius * u_x,
           alt
  };

  struct EnuCoor_i p2_in = { waypoints[p2].enu_i.x + 2 * radius * -u_y,
           waypoints[p2].enu_i.y + 2 * radius * u_x,
           alt
  };
  struct EnuCoor_i p2_center = { waypoints[p2].enu_i.x + radius * -u_y,
           waypoints[p2].enu_i.y + radius * u_x,
           alt
  };

  int32_t qdr_out_2 = INT32_ANGLE_PI - int32_atan2_2(u_y, u_x);
  int32_t qdr_out_1 = qdr_out_2 + INT32_ANGLE_PI;
  if (radius < 0) {
    qdr_out_2 += INT32_ANGLE_PI;
    qdr_out_1 += INT32_ANGLE_PI;
  }
  int32_t qdr_anticipation = ANGLE_BFP_OF_REAL(radius > 0 ? -15 : 15);

  switch (oval_status) {
    case OC1 :
      nav_circle(&p1_center, POS_BFP_OF_REAL(-radius));
      if (NavQdrCloseTo(INT32_DEG_OF_RAD(qdr_out_1) - qdr_anticipation)) {
        oval_status = OR12;
        InitStage();
        LINE_START_FUNCTION;
      }
      return;

    case OR12:
      nav_route(&p1_out, &p2_in);
      if (nav_approaching_from(&p2_in, &p1_out, CARROT)) {
        oval_status = OC2;
        nav_oval_count++;
        InitStage();
        LINE_STOP_FUNCTION;
      }
      return;

    case OC2 :
      nav_circle(&p2_center, POS_BFP_OF_REAL(-radius));
      if (NavQdrCloseTo(INT32_DEG_OF_RAD(qdr_out_2) - qdr_anticipation)) {
        oval_status = OR21;
        InitStage();
        LINE_START_FUNCTION;
      }
      return;

    case OR21:
      nav_route(&waypoints[p2].enu_i, &waypoints[p1].enu_i);
      if (nav_approaching_from(&waypoints[p1].enu_i, &waypoints[p2].enu_i, CARROT)) {
        oval_status = OC1;
        InitStage();
        LINE_STOP_FUNCTION;
      }
      return;

    default: /* Should not occur !!! Doing nothing */
      return;
  }
}
/*
#ifdef TRAFFIC_INFO
#include "modules/multi/traffic_info.h"

void nav_follow(uint8_t ac_id, uint32_t distance, uint32_t height)
{
    struct EnuCoor_i* target = acInfoGetPositionEnu_i(ac_id);


    float alpha = M_PI / 2 - acInfoGetCourse(ac_id);
    float ca = cosf(alpha), sa = sinf(alpha);
    target->x += - distance * ca;
    target->y += - distance * sa;
    target->z = (Max(target->z + height, SECURITY_HEIGHT)); // todo add ground height to check

    ENU_OF_TO_NED(navigation_target, *target);
}
#else*/
void nav_follow(uint8_t  __attribute__((unused)) _ac_id, uint32_t  __attribute__((unused)) distance,
                uint32_t  __attribute__((unused)) height) {}
//#endif
