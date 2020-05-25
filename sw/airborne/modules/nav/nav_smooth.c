/*
 * Copyright (C) 2007-2009  ENAC, Pascal Brisset, Antoine Drouin
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
 * @file modules/nav/nav_smooth.c
 *
 * Smooth navigation to wp_a along an arc (around wp_cd),
 * a segment (from wp_rd to wp_ta) and a second arc (around wp_ca).
 */

#include <math.h>
#include "generated/airframe.h"
#include "modules/nav/nav_smooth.h"
#include "state.h"
#include "firmwares/fixedwing/nav.h"
#include "subsystems/gps.h"

#define Sign(_x) ((_x) > 0 ? 1 : (-1))
#define Norm2Pi(x) ({ uint8_t _i=1; float _x = x; while (_i && _x < 0.) { _i++;_x += 2*M_PI; } while (_i && _x > 2*M_PI) { _i++; _x -= 2*M_PI; } _x; })

static struct point wp_cd, wp_td, wp_ca, wp_ta;
static float d_radius, a_radius;
static float qdr_td;
static float qdr_a;
static uint8_t wp_a;
float snav_desired_tow; /* time of week, s */
static float u_a_ca_x, u_a_ca_y;
static uint8_t ground_speed_timer;

/* D is the current position */
bool snav_init(uint8_t a, float desired_course_rad, float radius)
{
  wp_a = a;
  radius = fabs(radius);

  float da_x = WaypointX(wp_a) - stateGetPositionEnu_f()->x;
  float da_y = WaypointY(wp_a) - stateGetPositionEnu_f()->y;

  /* D_CD orthogonal to current course, CD on the side of A */
  float u_x = cos(M_PI_2 - stateGetHorizontalSpeedDir_f());
  float u_y = sin(M_PI_2 - stateGetHorizontalSpeedDir_f());
  d_radius = - Sign(u_x * da_y - u_y * da_x) * radius;
  wp_cd.x = stateGetPositionEnu_f()->x + d_radius * u_y;
  wp_cd.y = stateGetPositionEnu_f()->y - d_radius * u_x;
  wp_cd.a = WaypointAlt(wp_a);

  /* A_CA orthogonal to desired course, CA on the side of D */
  float desired_u_x = cos(M_PI_2 - desired_course_rad);
  float desired_u_y = sin(M_PI_2 - desired_course_rad);
  a_radius = Sign(desired_u_x * da_y - desired_u_y * da_x) * radius;
  u_a_ca_x = desired_u_y;
  u_a_ca_y = - desired_u_x;
  wp_ca.x = WaypointX(wp_a) + a_radius * u_a_ca_x;
  wp_ca.y = WaypointY(wp_a) + a_radius * u_a_ca_y;
  wp_ca.a = WaypointAlt(wp_a);

  /* Unit vector along CD-CA */
  u_x = wp_ca.x - wp_cd.x;
  u_y = wp_ca.y - wp_cd.y;
  float cd_ca = sqrt(u_x * u_x + u_y * u_y);

  /* If it is too close in reverse direction, set CA on the other side */
  if (a_radius * d_radius < 0 && cd_ca < 2 * radius) {
    a_radius = -a_radius;
    wp_ca.x = WaypointX(wp_a) + a_radius * u_a_ca_x;
    wp_ca.y = WaypointY(wp_a) + a_radius * u_a_ca_y;
    u_x = wp_ca.x - wp_cd.x;
    u_y = wp_ca.y - wp_cd.y;
    cd_ca = sqrt(u_x * u_x + u_y * u_y);
  }

  u_x /= cd_ca;
  u_y /= cd_ca;

  if (a_radius * d_radius > 0) {
    /* Both arcs are in the same direction */
    /* CD_TD orthogonal to CD_CA */
    wp_td.x = wp_cd.x - d_radius * u_y;
    wp_td.y = wp_cd.y + d_radius * u_x;

    /* CA_TA also orthogonal to CD_CA */
    wp_ta.x = wp_ca.x - a_radius * u_y;
    wp_ta.y = wp_ca.y + a_radius * u_x;
  } else {
    /* Arcs are in reverse directions: trigonemetric puzzle :-) */
    float alpha = atan2(u_y, u_x) + acos(d_radius / (cd_ca / 2));
    wp_td.x = wp_cd.x + d_radius * cos(alpha);
    wp_td.y = wp_cd.y + d_radius * sin(alpha);

    wp_ta.x = wp_ca.x + a_radius * cos(alpha);
    wp_ta.y = wp_ca.y + a_radius * sin(alpha);
  }
  qdr_td = M_PI_2 - atan2(wp_td.y - wp_cd.y, wp_td.x - wp_cd.x);
  qdr_a = M_PI_2 - atan2(WaypointY(wp_a) - wp_ca.y, WaypointX(wp_a) - wp_ca.x);
  wp_td.a = wp_cd.a;
  wp_ta.a = wp_ca.a;
  ground_speed_timer = 0;

  return false;
}


bool snav_circle1(void)
{
  /* circle around CD until QDR_TD */
  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(wp_cd.a, 0.);
  nav_circle_XY(wp_cd.x, wp_cd.y, d_radius);
  return (! NavQdrCloseTo(DegOfRad(qdr_td)));
}

bool snav_route(void)
{
  /* Straight route from TD to TA */
  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(wp_cd.a, 0.);
  nav_route_xy(wp_td.x, wp_td.y, wp_ta.x, wp_ta.y);

  return (! nav_approaching_xy(wp_ta.x, wp_ta.y, wp_td.x, wp_td.y, CARROT));
}

bool snav_circle2(void)
{
  /* circle around CA until QDR_A */
  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(wp_cd.a, 0.);
  nav_circle_XY(wp_ca.x, wp_ca.y, a_radius);

  return (! NavQdrCloseTo(DegOfRad(qdr_a)));
}

#define NB_ANGLES 24
#define ANGLE_STEP (2.*M_PI/NB_ANGLES)
static float ground_speeds[NB_ANGLES]; /* Indexed by trigo angles */

static inline float ground_speed_of_course(float x)
{
  uint8_t i = Norm2Pi(M_PI_2 - x) / ANGLE_STEP;
  return ground_speeds[i];
}

/* Compute the ground speed for courses 0, 360/NB_ANGLES, ...
   (NB_ANGLES-1)360/NB_ANGLES. Return false if wind speed
   is greater than airspeed */
static bool compute_ground_speed(float airspeed,
                                 float wind_east,
                                 float wind_north)
{
  uint8_t i;
  float alpha = 0;
  float c = wind_north * wind_north + wind_east * wind_east - airspeed * airspeed;
  for (i = 0; i < NB_ANGLES; i++, alpha += ANGLE_STEP) {
    /* g^2 -2 scal g + c = 0 */
    float scal = wind_east * cos(alpha) + wind_north * sin(alpha);
    float delta = 4 * (scal * scal - c);
    if (delta < 0)
      return false;
    ground_speeds[i] = scal + sqrt(delta) / 2.;
    Bound(ground_speeds[i], NOMINAL_AIRSPEED / 4, 2 * NOMINAL_AIRSPEED);
  }
  return true;
}

/* Adjusting a circle around CA, tangent in A, to end at snav_desired_tow */
bool snav_on_time(float nominal_radius)
{
  nominal_radius = fabs(nominal_radius);

  float current_qdr = M_PI_2 - atan2(stateGetPositionEnu_f()->y - wp_ca.y, stateGetPositionEnu_f()->x - wp_ca.x);
  float remaining_angle = Norm2Pi(Sign(a_radius) * (qdr_a - current_qdr));
  float remaining_time = snav_desired_tow - gps.tow / 1000.;

  /* Use the nominal airspeed if the estimated one is not realistic */
  float airspeed = stateGetAirspeed_f();
  if (airspeed < NOMINAL_AIRSPEED / 2. ||
      airspeed > 2.* NOMINAL_AIRSPEED) {
    airspeed = NOMINAL_AIRSPEED;
  }

  /* Recompute ground speeds every 10 s */
  if (ground_speed_timer == 0) {
    ground_speed_timer = 40; /* every 10s, called at 40Hz */
    if (!compute_ground_speed(airspeed, stateGetHorizontalWindspeed_f()->y,
                              stateGetHorizontalWindspeed_f()->x)) { // Wind in NED frame
      return false; /* return false if the computation of ground speeds fails */
    }
  }
  ground_speed_timer--;

  /* Time to complete the circle at nominal_radius */
  float nominal_time = 0.;

  float a;
  float ground_speed = NOMINAL_AIRSPEED; /* Init to avoid a warning */
  /* Going one step too far */
  for (a = 0; a < remaining_angle + ANGLE_STEP; a += ANGLE_STEP) {
    float qdr = current_qdr + Sign(a_radius) * a;
    ground_speed = ground_speed_of_course(qdr + Sign(a_radius) * M_PI_2);
    nominal_time += ANGLE_STEP * nominal_radius / ground_speed;
  }
  /* Removing what exceeds remaining_angle */
  nominal_time -= (a - remaining_angle) * nominal_radius / ground_speed;

  /* Radius size to finish in one single circle */
  float radius = remaining_time / nominal_time * nominal_radius;
  if (radius > 2. * nominal_radius) {
    radius = nominal_radius;
  }

  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(wp_cd.a, 0.);

  radius *= Sign(a_radius);
  wp_ca.x = WaypointX(wp_a) + radius * u_a_ca_x;
  wp_ca.y = WaypointY(wp_a) + radius * u_a_ca_y;
  nav_circle_XY(wp_ca.x, wp_ca.y, radius);

  /* Stay in this mode until the end of time */
  return (remaining_time > 0);
}
