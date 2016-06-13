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
 * @file modules/nav/nav_drop.c
 *
 * Navigation module to drop a ball at a given point
 * taking into account the wind and ground speed
 */

#include "modules/nav/nav_drop.h"
#include "state.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "inter_mcu.h"


#if defined WP_RELEASE
/** Speed limit of the paint ball */
#ifndef TRIGGER_DELAY
#define TRIGGER_DELAY 1.
#endif


#ifndef ALPHA
#define ALPHA 6.26e-5
#endif
#ifndef MASS
#define MASS 3.31e-3
#endif
#define ALPHA_M (ALPHA / MASS)
#define DT 0.1
#define MAX_STEPS 100

/** climb time in seconds */
#ifndef CLIMB_TIME
#define CLIMB_TIME 3
#endif

/** safe climb distance in meters */
#ifndef SAFE_CLIMB
#define SAFE_CLIMB 20
#endif

/** airspeed at release point in m/s
 * only used if airspeed from state interface is not valid
 */
#ifndef AIRSPEED_AT_RELEASE
#define AIRSPEED_AT_RELEASE 14.
#endif

float nav_drop_trigger_delay = TRIGGER_DELAY;
float airspeed = AIRSPEED_AT_RELEASE;
float nav_drop_start_qdr;

static float nav_drop_x, nav_drop_y, nav_drop_z;
static float nav_drop_vx, nav_drop_vy, nav_drop_vz;


static void integrate(uint8_t wp_target)
{
  /* Inspired from Arnold Schroeter's code */
  int i = 0;
  while (nav_drop_z > 0. && i < MAX_STEPS) {
    /* relative wind experienced by the ball (wind in NED frame) */
    float airx = -nav_drop_vx + stateGetHorizontalWindspeed_f()->y;
    float airy = -nav_drop_vy + stateGetHorizontalWindspeed_f()->x;
    float airz = -nav_drop_vz;

    /* alpha / m * air */
    float beta = ALPHA_M * sqrt(airx * airx + airy * airy + airz * airz);

    /* Euler integration */
    nav_drop_vx += airx * beta * DT;
    nav_drop_vy += airy * beta * DT;
    nav_drop_vz += (airz * beta - NAV_GRAVITY) * DT;

    nav_drop_x += nav_drop_vx * DT;
    nav_drop_y += nav_drop_vy * DT;
    nav_drop_z += nav_drop_vz * DT;

    i++;
  }

  if (nav_drop_z > 0.) {
    /* Integration not finished -> approximation of the time with the current
       speed  */
    float t = - nav_drop_z / nav_drop_vz;
    nav_drop_x += nav_drop_vx * t;
    nav_drop_y += nav_drop_vy * t;
  }

  waypoints[WP_RELEASE].x = waypoints[wp_target].x - nav_drop_x;
  waypoints[WP_RELEASE].y = waypoints[wp_target].y - nav_drop_y;
}


/** Update the RELEASE location with the actual ground speed and altitude */
unit_t nav_drop_update_release(uint8_t wp_target)
{

  nav_drop_z = stateGetPositionUtm_f()->alt - waypoints[wp_target].a;
  nav_drop_x = 0.;
  nav_drop_y = 0.;

  nav_drop_vx = stateGetHorizontalSpeedNorm_f() * sin(stateGetHorizontalSpeedDir_f());
  nav_drop_vy = stateGetHorizontalSpeedNorm_f() * cos(stateGetHorizontalSpeedDir_f());
  nav_drop_vz = 0.;

  integrate(wp_target);

  return 0;
}


/** Compute a first approximation for the RELEASE waypoint from wind and
    expected airspeed and altitude */
unit_t nav_drop_compute_approach(uint8_t wp_target, uint8_t wp_start, uint8_t wp_baseturn, uint8_t wp_climbout,
                                 float nav_drop_radius)
{
  waypoints[WP_RELEASE].a = waypoints[wp_start].a;
  nav_drop_z = waypoints[WP_RELEASE].a - waypoints[wp_target].a;
  nav_drop_x = 0.;
  nav_drop_y = 0.;

  /* We approximate vx and vy, taking into account that RELEASE is next to
     TARGET */
  float x_0 = waypoints[wp_target].x - waypoints[wp_start].x;
  float y_0 = waypoints[wp_target].y - waypoints[wp_start].y;

  /* Unit vector from START to TARGET */
  float d = sqrt(x_0 * x_0 + y_0 * y_0);
  float x1 = x_0 / d;
  float y_1 = y_0 / d;

  waypoints[wp_baseturn].x = waypoints[wp_start].x + y_1 * nav_drop_radius;
  waypoints[wp_baseturn].y = waypoints[wp_start].y - x1 * nav_drop_radius;
  waypoints[wp_baseturn].a = waypoints[wp_start].a;
  nav_drop_start_qdr = M_PI - atan2(-y_1, -x1);
  if (nav_drop_radius < 0) {
    nav_drop_start_qdr += M_PI;
  }

  // wind in NED frame
  if (stateIsAirspeedValid()) {
    nav_drop_vx = x1 * stateGetAirspeed_f() + stateGetHorizontalWindspeed_f()->y;
    nav_drop_vy = y_1 * stateGetAirspeed_f() + stateGetHorizontalWindspeed_f()->x;
  } else {
    // use approximate airspeed, initially set to AIRSPEED_AT_RELEASE
    nav_drop_vx = x1 * airspeed + stateGetHorizontalWindspeed_f()->y;
    nav_drop_vy = y_1 * airspeed + stateGetHorizontalWindspeed_f()->x;
  }
  nav_drop_vz = 0.;

  float vx0 = nav_drop_vx;
  float vy0 = nav_drop_vy;

  integrate(wp_target);

  waypoints[wp_climbout].x = waypoints[WP_RELEASE].x + (CLIMB_TIME + CARROT) * vx0;
  waypoints[wp_climbout].y = waypoints[WP_RELEASE].y + (CLIMB_TIME + CARROT) * vy0;
  waypoints[wp_climbout].a = waypoints[WP_RELEASE].a + SAFE_CLIMB;

  return 0;
}



unit_t nav_drop_shoot(void)
{
  ap_state->commands[COMMAND_HATCH] = MAX_PPRZ;
  return 0;
}

/* Compute start and end waypoints to be aligned on w1-w2 */
bool compute_alignment(uint8_t w1, uint8_t w2, uint8_t wp_before, uint8_t wp_after, float d_before, float d_after)
{
  float x_0 = waypoints[w2].x - waypoints[w1].x;
  float y_0 = waypoints[w2].y - waypoints[w1].y;

  /* Unit vector from W1 to W2 */
  float d = sqrt(x_0 * x_0 + y_0 * y_0);
  x_0 /= d;
  y_0 /= d;

  waypoints[wp_before].x = waypoints[w1].x - d_before * x_0;
  waypoints[wp_before].y = waypoints[w1].y - d_before * y_0;
  waypoints[wp_after].x = waypoints[w2].x + d_after * x_0;
  waypoints[wp_after].y = waypoints[w2].y + d_after * y_0;

  return false;
}

#endif /* WP_RELEASE */
