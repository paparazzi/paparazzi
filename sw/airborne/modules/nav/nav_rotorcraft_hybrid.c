/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/nav/nav_rotorcraft_hybrid.c"
 * Specific navigation functions for hybrid aircraft
 */

#include "modules/nav/nav_rotorcraft_hybrid.h"
#include "firmwares/rotorcraft/navigation.h"
#include "math/pprz_isa.h"
#include "generated/flight_plan.h"

// if NAV_HYBRID_MAX_BANK is not defined, set it to 30 degrees.
#ifndef NAV_HYBRID_MAX_BANK
float nav_hybrid_max_bank = 0.52f;
#else
float nav_hybrid_max_bank = NAV_HYBRID_MAX_BANK;
#endif

// Max ground speed that will be commanded
#ifndef NAV_HYBRID_MAX_AIRSPEED
#define NAV_HYBRID_MAX_AIRSPEED 15.0f
#endif

#ifndef NAV_HYBRID_SPEED_MARGIN
#define NAV_HYBRID_SPEED_MARGIN 10.0f
#endif

#define NAV_MAX_SPEED (NAV_HYBRID_MAX_AIRSPEED + NAV_HYBRID_SPEED_MARGIN)
float nav_max_speed = NAV_MAX_SPEED;

#ifndef NAV_HYBRID_GOTO_MAX_SPEED
#define NAV_HYBRID_GOTO_MAX_SPEED NAV_MAX_SPEED
#endif

#ifndef NAV_HYBRID_MAX_DECELERATION
#define NAV_HYBRID_MAX_DECELERATION 1.0
#endif

#ifndef NAV_HYBRID_MAX_ACCELERATION
#define NAV_HYBRID_MAX_ACCELERATION 4.0
#endif

#ifndef NAV_HYBRID_SOFT_ACCELERATION
#define NAV_HYBRID_SOFT_ACCELERATION NAV_HYBRID_MAX_ACCELERATION
#endif

float nav_max_deceleration_sp      = NAV_HYBRID_MAX_DECELERATION;  //Maximum deceleration allowed for the set-point
float nav_max_acceleration_sp      = NAV_HYBRID_MAX_ACCELERATION;  //Maximum acceleration allowed for the set-point
float nav_hybrid_soft_acceleration = NAV_HYBRID_SOFT_ACCELERATION; //Soft acceleration limit allowed for the set-point (Equal to the maximum acceleration by default)
float nav_hybrid_max_acceleration  = NAV_HYBRID_MAX_ACCELERATION;  //Maximum general limit in acceleration allowed for the hybrid navigation

#ifndef NAV_HYBRID_MAX_EXPECTED_WIND
#define NAV_HYBRID_MAX_EXPECTED_WIND 5.0f
#endif
float nav_hybrid_max_expected_wind = NAV_HYBRID_MAX_EXPECTED_WIND;

#ifdef NAV_HYBRID_LINE_GAIN
float nav_hybrid_line_gain = NAV_HYBRID_LINE_GAIN;
#else
float nav_hybrid_line_gain = 1.0f;
#endif

#ifndef NAV_HYBRID_NAV_LINE_DIST
#define NAV_HYBRID_NAV_LINE_DIST 50.f
#endif

#ifndef NAV_HYBRID_NAV_CIRCLE_DIST
#define NAV_HYBRID_NAV_CIRCLE_DIST 40.f
#endif

#ifdef GUIDANCE_INDI_POS_GAIN
float nav_hybrid_pos_gain = GUIDANCE_INDI_POS_GAIN;
#elif defined(NAV_HYBRID_POS_GAIN)
float nav_hybrid_pos_gain = NAV_HYBRID_POS_GAIN;
#else
float nav_hybrid_pos_gain = 1.0f;
#endif

#if defined(NAV_HYBRID_POS_GAIN) && defined(GUIDANCE_INDI_POS_GAIN)
#warning "NAV_HYBRID_POS_GAIN and GUIDANCE_INDI_POS_GAIN serve similar purpose and are both defined, using GUIDANCE_INDI_POS_GAIN"
#endif

#ifndef GUIDANCE_INDI_HYBRID
bool force_forward = 0.0f;
#endif

/* When using external vision, run the nav functions in position mode for better tracking*/
#ifndef NAV_HYBRID_EXT_VISION_SETPOINT_MODE
#define NAV_HYBRID_EXT_VISION_SETPOINT_MODE FALSE
#endif

/* Use max target groundspeed and max acceleration to limit circle radius*/
#ifndef NAV_HYBRID_LIMIT_CIRCLE_RADIUS
#define NAV_HYBRID_LIMIT_CIRCLE_RADIUS FALSE
#endif

static float max_speed_for_deceleration(float dist_to_wp);

/** Implement basic nav function for the hybrid case
 */

static void nav_hybrid_goto(struct EnuCoor_f *wp)
{
  nav_max_acceleration_sp = nav_hybrid_soft_acceleration;
  nav_rotorcraft_base.goto_wp.to = *wp;
  nav_rotorcraft_base.goto_wp.dist2_to_wp = get_dist2_to_point(wp);
  VECT2_COPY(nav.target, *wp);

  // Calculate position error
  struct FloatVect2 pos_error;
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  VECT2_DIFF(pos_error, nav.target, *pos);

  struct FloatVect2 speed_sp;
  VECT2_SMUL(speed_sp, pos_error, nav_hybrid_pos_gain);

  // Bound the setpoint velocity vector
  float max_h_speed = nav_max_speed;
  if (!force_forward) {
    // If not in force_forward, compute speed based on deceleration and nav_max_speed
    // Calculate distance to waypoint
    float dist_to_wp = float_vect2_norm(&pos_error);
    max_h_speed = max_speed_for_deceleration(dist_to_wp);
  }
  float_vect2_bound_in_2d(&speed_sp, max_h_speed);

  VECT2_COPY(nav.speed, speed_sp);
  nav.horizontal_mode = NAV_HORIZONTAL_MODE_WAYPOINT;

  // In optitrack tests use position mode for more precise hovering
#if NAV_HYBRID_EXT_VISION_SETPOINT_MODE
    nav.setpoint_mode = NAV_SETPOINT_MODE_POS;
#else
    nav.setpoint_mode = NAV_SETPOINT_MODE_SPEED;
#endif
}

static void nav_hybrid_route(struct EnuCoor_f *wp_start, struct EnuCoor_f *wp_end)
{
  struct FloatVect2 wp_diff, pos_diff;
  VECT2_DIFF(wp_diff, *wp_end, *wp_start);
  VECT2_DIFF(pos_diff, *wp_end, *stateGetPositionEnu_f());

  // Calculate magnitude of the desired speed vector based on distance to waypoint
  float dist_to_target = float_vect2_norm(&pos_diff);
  float desired_speed;
  if (force_forward) {
    desired_speed = nav_max_speed;
  } else {
    desired_speed = dist_to_target * nav_hybrid_pos_gain;
    float max_h_speed = max_speed_for_deceleration(dist_to_target);
    Bound(desired_speed, 0.0f, max_h_speed);
  }

  // Calculate length of line segment
  float length_line = Max(float_vect2_norm(&wp_diff), 0.01f);
  // Normal vector to the line, with length of the line
  struct FloatVect2 normalv;
  VECT2_ASSIGN(normalv, -wp_diff.y, wp_diff.x);
  // Length of normal vector is the same as of the line segment
  float length_normalv = length_line; // >= 0.01
  // Distance along the normal vector
  float dist_to_line = (pos_diff.x * normalv.x + pos_diff.y * normalv.y) / length_normalv;
  // Normal vector scaled to be the distance to the line
  struct FloatVect2 v_to_line, v_along_line;
  v_to_line.x = dist_to_line * normalv.x / length_normalv * nav_hybrid_line_gain;
  v_to_line.y = dist_to_line * normalv.y / length_normalv * nav_hybrid_line_gain;
  // The distance that needs to be traveled along the line
  v_along_line.x = wp_diff.x / length_line * NAV_HYBRID_NAV_LINE_DIST;
  v_along_line.y = wp_diff.y / length_line * NAV_HYBRID_NAV_LINE_DIST;
  // Calculate the desired direction to converge to the line
  struct FloatVect2 direction;
  VECT2_SMUL(direction, v_along_line, (1.f / (1.f + fabsf(dist_to_line) * 0.05f)));
  VECT2_ADD(direction, v_to_line);
  float length_direction = Max(float_vect2_norm(&direction), 0.01f);
  // Scale to have the desired speed
  VECT2_SMUL(nav.speed, direction, desired_speed / length_direction);
  // final target position, should be on the line, for display
  VECT2_SUM(nav.target, *stateGetPositionEnu_f(), direction);

  nav_rotorcraft_base.goto_wp.from = *wp_start;
  nav_rotorcraft_base.goto_wp.to = *wp_end;
  nav_rotorcraft_base.goto_wp.dist2_to_wp = get_dist2_to_point(wp_end);
  nav.horizontal_mode = NAV_HORIZONTAL_MODE_ROUTE;
  nav.setpoint_mode = NAV_SETPOINT_MODE_SPEED;
}

/**
 * Calculate max speed when decelerating at MAX capacity a_max
 * distance travelled d = 1/2 a_max t^2
 * The time in which it does this is: T = V / a_max
 * The maximum speed at which to fly to still allow arriving with zero
 * speed at the waypoint given maximum deceleration is: V = sqrt(2 * a_max * d)
 */
static float max_speed_for_deceleration(float dist_to_wp) {
  float max_speed_decel2 = fabsf(2.f * dist_to_wp * nav_max_deceleration_sp); // dist_to_wp can only be positive, but just in case
  float max_speed_decel = sqrtf(max_speed_decel2);
  // Bound the setpoint velocity vector
  float max_h_speed = Min(nav_max_speed, max_speed_decel);
  return max_h_speed;
}

static bool nav_hybrid_approaching(struct EnuCoor_f *wp, struct EnuCoor_f *from, float approaching_time)
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

static void nav_hybrid_circle(struct EnuCoor_f *wp_center, float radius)
{
  struct FloatVect2 pos_diff;
  float desired_speed;

  VECT2_DIFF(pos_diff, *stateGetPositionEnu_f(), *wp_center);
  // direction of rotation
  float sign_radius = (radius > 0.f) ? 1.f : (radius < 0.f) ? -1.f : 0.f;
  // absolute radius
  float abs_radius = fabsf(radius);
#if NAV_HYBRID_LIMIT_CIRCLE_RADIUS
  float min_radius =  pow(nav_max_speed+nav_hybrid_max_expected_wind,2) / (nav_hybrid_max_acceleration * 0.8);
  abs_radius = Max(abs_radius, min_radius);
#endif
  if (abs_radius > 0.1f) {
    // store last qdr
    float last_qdr = nav_rotorcraft_base.circle.qdr;
    // compute qdr
    nav_rotorcraft_base.circle.qdr = atan2f(pos_diff.y, pos_diff.x);
    // increment circle radians
    float trigo_diff = nav_rotorcraft_base.circle.qdr - last_qdr;
    NormRadAngle(trigo_diff);
    nav_rotorcraft_base.circle.radians += trigo_diff;
    // progress angle
    float progress_angle = NAV_HYBRID_NAV_CIRCLE_DIST / abs_radius;
    Bound(progress_angle, M_PI / 16.f, M_PI / 4.f);
    float alpha = nav_rotorcraft_base.circle.qdr - sign_radius * progress_angle;
    // final target position, should be on the circle, for display
    nav.target.x = wp_center->x + cosf(alpha) * abs_radius;
    nav.target.y = wp_center->y + sinf(alpha) * abs_radius;
  }
  else {
    // radius is too small, direct to center
    VECT2_COPY(nav.target, *wp_center);
  }
  // compute desired speed
  float radius_diff = fabsf(float_vect2_norm(&pos_diff) - abs_radius);
  if (radius_diff > NAV_HYBRID_NAV_CIRCLE_DIST) {
    // far from circle, speed proportional to diff
    desired_speed = radius_diff * nav_hybrid_pos_gain;
    nav_max_acceleration_sp = nav_hybrid_soft_acceleration;
  } else {
    // close to circle, speed function of radius for a feasible turn
    // 0.8 * MAX_BANK gives some margins for the turns
    desired_speed = sqrtf(PPRZ_ISA_GRAVITY * abs_radius * tanf(0.8f * nav_hybrid_max_bank));
    nav_max_acceleration_sp = nav_hybrid_max_acceleration ;
  }
  if (force_forward) {
    desired_speed = nav_max_speed;
    nav_max_acceleration_sp = nav_hybrid_max_acceleration ;
  }
  Bound(desired_speed, 0.0f, nav_max_speed);
  // compute speed vector from target position
  struct FloatVect2 speed_unit;
  VECT2_DIFF(speed_unit, nav.target, *stateGetPositionEnu_f());
  float_vect2_normalize(&speed_unit);
  VECT2_SMUL(nav.speed, speed_unit, desired_speed);
  nav_rotorcraft_base.circle.center = *wp_center;
  nav_rotorcraft_base.circle.radius = sign_radius * abs_radius;
  nav.horizontal_mode = NAV_HORIZONTAL_MODE_CIRCLE;
  nav.setpoint_mode = NAV_SETPOINT_MODE_SPEED;
}

/** Init and register nav functions
 *
 * For hybrid vehicle nav
 * Init should be called after the normal rotorcraft nav_init
 * as we are reusing some of the functions and overwritting others
 */
void nav_rotorcraft_hybrid_init(void)
{
  nav_rotorcraft_base.circle.radius = DEFAULT_CIRCLE_RADIUS;
  nav_rotorcraft_base.goto_wp.leg_progress = 0.f;
  nav_rotorcraft_base.goto_wp.leg_length = 1.f;

  // register nav functions
  nav_register_goto_wp(nav_hybrid_goto, nav_hybrid_route, nav_hybrid_approaching);
  nav_register_circle(nav_hybrid_circle);
}


