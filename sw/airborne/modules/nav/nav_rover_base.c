/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file "modules/nav/nav_rover_base.c"
 * @author 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Basic navigation functions for Rovers
 */

#include "modules/nav/nav_rover_base.h"

struct RoverNavBase nav_rover_base;

/** Implement basic nav function
 */

static void nav_goto(struct EnuCoor_f *wp)
{
  nav_rover_base.goto_wp.to = *wp;
  nav_rover_base.goto_wp.dist2_to_wp = get_dist2_to_point(wp);
  nav.mode = NAV_MODE_WAYPOINT;
}

static void nav_route(struct EnuCoor_f *wp_start, struct EnuCoor_f *wp_end)
{
  struct FloatVect2 wp_diff, pos_diff;
  VECT2_DIFF(wp_diff, *wp_end, *wp_start);
  VECT2_DIFF(pos_diff, *stateGetPositionEnu_f(), *wp_start);
  // leg length
  float leg_length2 = Max((wp_diff.x * wp_diff.x + wp_diff.y * wp_diff.y), 0.1f);
  nav_rover_base.goto_wp.leg_length = sqrtf(leg_length2);
  // leg progress
  nav_rover_base.goto_wp.leg_progress = (pos_diff.x * wp_diff.x + pos_diff.y * wp_diff.y) / nav_rover_base.goto_wp.leg_length;
  nav_rover_base.goto_wp.leg_progress += Max(CARROT_DIST, 0.f);
  // next pos on leg
  struct FloatVect2 progress_pos;
  VECT2_SMUL(progress_pos, wp_diff, nav_rover_base.goto_wp.leg_progress / nav_rover_base.goto_wp.leg_length);
  VECT2_SUM(nav.target, *wp_start, progress_pos);

  nav_rover_base.goto_wp.from = *wp_start;
  nav_rover_base.goto_wp.to = *wp_end;
  nav_rover_base.goto_wp.dist2_to_wp = get_dist2_to_point(wp_end);
  nav.mode = NAV_MODE_ROUTE;
}

static bool nav_approaching(struct EnuCoor_f *wp, struct EnuCoor_f *from, float approaching_time)
{
  float dist_to_point;
  struct FloatVect2 diff;
  struct EnuCoor_f *pos = stateGetPositionEnu_f();

  /* if an approaching_time is given, estimate diff after approching_time secs */
  if (approaching_time > 0.f) {
    struct FloatVect2 estimated_pos;
    struct FloatVect2 estimated_progress;
    struct EnuCoor_f *speed = stateGetSpeedEnu_f();
    VECT2_SMUL(estimated_progress, *speed, approaching_time);
    VECT2_SUM(estimated_pos, *pos, estimated_progress);
    VECT2_DIFF(diff, *wp, estimated_pos);
  }
  /* else use current position */
  else {
    VECT2_DIFF(diff, *wp, *pos);
  }
  /* compute distance of estimated/current pos to target wp
   */
  //nav.dist_to_point = float_vect2_norm(&diff); FIXME
  dist_to_point = float_vect2_norm(&diff);

  /* return TRUE if we have arrived */
  if (dist_to_point < ARRIVED_AT_WAYPOINT) {
    return true;
  }

  /* if coming from a valid waypoint */
  if (from != NULL) {
    /* return TRUE if normal line at the end of the segment is crossed */
    struct FloatVect2 from_diff;
    VECT2_DIFF(from_diff, *wp, *from);
    return (diff.x * from_diff.x + diff.y * from_diff.y < 0.f);
  }

  return false;
}

static void nav_circle(struct EnuCoor_f *wp_center, float radius)
{
  struct FloatVect2 pos_diff;
  VECT2_DIFF(pos_diff, *stateGetPositionEnu_f(), *wp_center);
  // store last qdr
  float last_qdr = nav_rover_base.circle.qdr;
  // compute qdr
  nav_rover_base.circle.qdr = atan2f(pos_diff.y, pos_diff.x);
  // increment circle radians
  float trigo_diff = nav_rover_base.circle.qdr - last_qdr;
  NormRadAngle(trigo_diff);
  nav_rover_base.circle.radians += trigo_diff;

  // direction of rotation
  float sign_radius = radius > 0.f ? 1.f : -1.f;
  // absolute radius
  float abs_radius = fabsf(radius);
  if (abs_radius > 0.1f) {
    // carrot_angle
    float carrot_angle = CARROT_DIST / abs_radius;
    carrot_angle = Min(carrot_angle, M_PI / 4);
    carrot_angle = Max(carrot_angle, M_PI / 16);
    carrot_angle = nav_rover_base.circle.qdr - sign_radius * carrot_angle;
    // compute setpoint
    VECT2_ASSIGN(pos_diff, abs_radius * cosf(carrot_angle), abs_radius * sinf(carrot_angle));
    VECT2_SUM(nav.target, *wp_center, pos_diff);
  }
  else {
    // radius is too small, direct to center
    VECT2_COPY(nav.target, *wp_center);
  }

  nav_rover_base.circle.center = *wp_center;
  nav_rover_base.circle.radius = radius;
  nav.mode = NAV_MODE_CIRCLE;
}

/**
 * Navigation along a figure O. One side leg is defined by waypoints [p1] and [p2].
 * The navigation goes through 4 states: OC1 (half circle next to [p1]),
 * OR21 (route [p2] to [p1], OC2 (half circle next to [p2]) and OR12 (opposite leg).
 * Initial state is the route along the desired segment (OC2).
 */
#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif

static void _nav_oval_init(void)
{
  nav_rover_base.oval.status = OC2;
  nav_rover_base.oval.count = 0;
}

static void nav_oval(struct EnuCoor_f *wp1, struct EnuCoor_f *wp2, float radius)
{
  (void) wp1;
  (void) wp2;
  (void) radius;
#if 0
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
#endif
}

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

#if ROVER_BASE_SEND_TRAJECTORY
static void send_segment(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SEGMENT(trans, dev, AC_ID,
      &nav_rover_base.goto_wp.from.x,
      &nav_rover_base.goto_wp.from.y,
      &nav_rover_base.goto_wp.to.x,
      &nav_rover_base.goto_wp.to.y);
}

static void send_circle(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_CIRCLE(trans, dev, AC_ID,
      &nav_rover_base.circle.center.x,
      &nav_rover_base.circle.center.y,
      &nav_rover_base.circle.radius);
}
#endif // ROVER_BASE_SEND_TRAJECTORY

static void send_nav_status(struct transport_tx *trans, struct link_device *dev)
{
  float dist_home = sqrtf(nav.dist2_to_home);
  float dist_wp = sqrtf(nav_rover_base.goto_wp.dist2_to_wp);
  pprz_msg_send_ROTORCRAFT_NAV_STATUS(trans, dev, AC_ID,
                                      &block_time, &stage_time,
                                      &dist_home, &dist_wp,
                                      &nav_block, &nav_stage,
                                      &nav.mode);
#if ROVER_BASE_SEND_TRAJECTORY
  if (nav.mode == NAV_MODE_ROUTE) {
    send_segment(trans, dev);
  } else if (nav.mode == NAV_MODE_CIRCLE) {
    send_circle(trans, dev);
  }
#endif // ROVER_BASE_SEND_TRAJECTORY
}
#endif

/** Init and register nav functions
 */
void nav_rover_init(void)
{
  nav_rover_base.circle.radius = DEFAULT_CIRCLE_RADIUS;
  nav_rover_base.goto_wp.leg_progress = 0.f;
  nav_rover_base.goto_wp.leg_length = 1.f;

  // register nav functions
  nav_register_goto_wp(nav_goto, nav_route, nav_approaching);
  nav_register_circle(nav_circle);
  nav_register_oval(_nav_oval_init, nav_oval);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_NAV_STATUS, send_nav_status);
#endif

}


