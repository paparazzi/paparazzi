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
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h" // strong dependency for now
#include "math/pprz_isa.h"

// Max ground speed that will be commanded
#ifndef GUIDANCE_INDI_NAV_SPEED_MARGIN
#define GUIDANCE_INDI_NAV_SPEED_MARGIN 10.0f
#endif
#define NAV_MAX_SPEED (GUIDANCE_INDI_MAX_AIRSPEED + GUIDANCE_INDI_NAV_SPEED_MARGIN)
float nav_max_speed = NAV_MAX_SPEED;

#ifndef MAX_DECELERATION
#define MAX_DECELERATION 1.f
#endif

#ifdef GUIDANCE_INDI_LINE_GAIN
static float guidance_indi_line_gain = GUIDANCE_INDI_LINE_GAIN;
#else
static float guidance_indi_line_gain = 1.0f;
#endif

#ifndef GUIDANCE_INDI_NAV_LINE_DIST
#define GUIDANCE_INDI_NAV_LINE_DIST 50.f
#endif

#ifndef GUIDANCE_INDI_NAV_CIRCLE_DIST
#define GUIDANCE_INDI_NAV_CIRCLE_DIST 40.f
#endif

/** Implement basic nav function for the hybrid case
 */

static void nav_hybrid_goto(struct EnuCoor_f *wp)
{
  nav_rotorcraft_base.goto_wp.to = *wp;
  nav_rotorcraft_base.goto_wp.dist2_to_wp = get_dist2_to_point(wp);
  VECT2_COPY(nav.target, *wp);

  // Calculate position error
  struct FloatVect2 pos_error;
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  VECT2_DIFF(pos_error, nav.target, *pos);

  struct FloatVect2 speed_sp;
  VECT2_SMUL(speed_sp, pos_error, gih_params.pos_gain);

  if (force_forward) {
    float_vect2_scale_in_2d(&speed_sp, nav_max_speed);
  } else {
    // Calculate distance to waypoint
    float dist_to_wp = float_vect2_norm(&pos_error);
    // Calculate max speed to decelerate from
    float max_speed_decel2 = fabsf(2.f * dist_to_wp * MAX_DECELERATION); // dist_to_wp can only be positive, but just in case
    float max_speed_decel = sqrtf(max_speed_decel2);
    // Bound the setpoint velocity vector
    float max_h_speed = Min(nav_max_speed, max_speed_decel);
    float_vect2_bound_in_2d(&speed_sp, max_h_speed);
  }

  VECT2_COPY(nav.speed, speed_sp);
  nav.horizontal_mode = NAV_HORIZONTAL_MODE_WAYPOINT;
  nav.setpoint_mode = NAV_SETPOINT_MODE_SPEED;
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
    desired_speed = dist_to_target * gih_params.pos_gain;
    Bound(desired_speed, 0.0f, nav_max_speed);
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
  v_to_line.x = dist_to_line * normalv.x / length_normalv * guidance_indi_line_gain;
  v_to_line.y = dist_to_line * normalv.y / length_normalv * guidance_indi_line_gain;
  // The distance that needs to be traveled along the line
  v_along_line.x = wp_diff.x / length_line * GUIDANCE_INDI_NAV_LINE_DIST;
  v_along_line.y = wp_diff.y / length_line * GUIDANCE_INDI_NAV_LINE_DIST;
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
  float sign_radius = radius > 0.f ? 1.f : -1.f;
  // absolute radius
  float abs_radius = fabsf(radius);

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
    float progress_angle = GUIDANCE_INDI_NAV_CIRCLE_DIST / abs_radius;
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
  if (force_forward) {
    desired_speed = nav_max_speed;
  } else {
    float radius_diff = fabsf(float_vect2_norm(&pos_diff) - abs_radius);
    if (radius_diff > GUIDANCE_INDI_NAV_CIRCLE_DIST) {
      // far from circle, speed proportional to diff
      desired_speed = radius_diff * gih_params.pos_gain;
    } else {
      // close to circle, speed function of radius for a feasible turn
      // MAX_BANK / 2 gives some margins for the turns
      desired_speed = sqrtf(PPRZ_ISA_GRAVITY * abs_radius * tanf(GUIDANCE_H_MAX_BANK / 2.f));
    }
    Bound(desired_speed, 0.0f, nav_max_speed);
  }
  // compute speed vector from target position
  struct FloatVect2 speed_unit;
  VECT2_DIFF(speed_unit, nav.target, *stateGetPositionEnu_f());
  float_vect2_normalize(&speed_unit);
  VECT2_SMUL(nav.speed, speed_unit, desired_speed);

  nav_rotorcraft_base.circle.center = *wp_center;
  nav_rotorcraft_base.circle.radius = radius;
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


