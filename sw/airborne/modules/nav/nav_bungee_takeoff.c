/*
 * Copyright (C) 2008-2015 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/nav/nav_bungee_takeoff.c
 *
 * Takeoff functions for bungee takeoff.
 *
 * Run initialize function when the plane is on the bungee, the bungee is
 * fully extended and you are ready to launch the plane.
 * After initialized, the plane will follow a line drawn by the position
 * of the plane on initialization and the  position of the bungee (given in
 * the arguments).
 * Once the plane crosses the throttle line, which is perpendicular to the line
 * the plane is following, and intersects the position of the bungee (plus or
 * minus a fixed distance (BUNGEE_TAKEOFF_DISTANCE in airframe file) from
 * the bungee just in case the bungee doesn't release exactly above the bungee)
 * the prop will come on.
 * The plane will then continue to follow the line until it has reached a
 * specific height (defined in as BUNGEE_TAKEOFF_HEIGHT in airframe file) above
 * the bungee waypoint and airspeed (defined as BUNGEE_TAKEOFF_AIRSPEED in the
 * airframe file). The airspeed limit is only used if USE_AIRSPEED flag is
 * defined or set to true (and assuming the airspeed is then available).
 * It is also possible to specify the pitch angle (BUNGEE_TAKEOFF_PITCH) and
 * the throttle (BUNGEE_TAKEOFF_THROTTLE, between 0 and 1).
 *
 * @verbatim
 * <section name="BUNGEE" prefix="BUNGEE_TAKEOFF_">
 *   <define name="HEIGHT" value="30" unit="m"/>
 *   <define name="AIRSPEED" value="15" unit="m/s"/>
 *   <define name="DISTANCE" value="10" unit="m"/>
 *   <define name="MIN_SPEED" value="5" unit="m/s"/>
 *   <define name="PITCH" value="15." unit="deg"/>
 *   <define name="THROTTLE" value="1.0"/>
 * </section>
 * @endverbatim
 *
 *
 * initial code from OSAM advanced navigation routines
 *
 */

#include "modules/nav/nav_bungee_takeoff.h"

#include "state.h"
#include "paparazzi.h"
#include "firmwares/fixedwing/nav.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "math/pprz_algebra_float.h"


// Backward compatibility
#ifdef Takeoff_Distance
#warning "Takeoff_Distance depreciated, please use BUNGEE_TAKEOFF_DISTANCE instead"
#define BUNGEE_TAKEOFF_DISTANCE Takeoff_Distance
#endif
#ifdef Takeoff_Height
#warning "Takeoff_Height depreciated, please use BUNGEE_TAKEOFF_HEIGHT instead"
#define BUNGEE_TAKEOFF_HEIGHT Takeoff_Height
#endif
#ifdef Takeoff_Speed
#warning "Takeoff_Speed depreciated, please use BUNGEE_TAKEOFF_AIRSPEED instead (beware that USE_AIRSPEED flag is needed)"
#define BUNGEE_TAKEOFF_AIRSPEED Takeoff_Speed
#endif
#ifdef Takeoff_MinSpeed
#warning "Takeoff_MinSpeed depreciated, please use BUNGEE_TAKEOFF_MIN_SPEED instead"
#define BUNGEE_TAKEOFF_MIN_SPEED Takeoff_MinSpeed
#endif


#ifndef BUNGEE_TAKEOFF_DISTANCE
#define BUNGEE_TAKEOFF_DISTANCE 10.0
#endif
#ifndef BUNGEE_TAKEOFF_HEIGHT
#define BUNGEE_TAKEOFF_HEIGHT 30.0
#endif
#if USE_AIRSPEED
#ifndef BUNGEE_TAKEOFF_AIRSPEED
#define BUNGEE_TAKEOFF_AIRSPEED 15.0
#endif
#else
#ifdef BUNGEE_TAKEOFF_AIRSPEED
#warning "BUNGEE_TAKEOFF_AIRSPEED is defined but not USE_AIRSPEED. Airspeed limit will not be used"
#endif
#endif
#ifndef BUNGEE_TAKEOFF_MIN_SPEED
#define BUNGEE_TAKEOFF_MIN_SPEED 5.0
#endif
#ifndef BUNGEE_TAKEOFF_THROTTLE
#define BUNGEE_TAKEOFF_THROTTLE 1.0
#endif
#ifndef BUNGEE_TAKEOFF_PITCH
#ifdef AGR_CLIMB_PITCH
#define BUNGEE_TAKEOFF_PITCH AGR_CLIMB_PITCH
#else
#define BUNGEE_TAKEOFF_PITCH RadOfDeg(15.)
#endif
#endif

enum TakeoffStatus {
  Launch,
  Throttle,
  Finished
};

static enum TakeoffStatus CTakeoffStatus;

static struct FloatVect2 init_point;
static struct FloatVect2 throttle_point;
static struct FloatVect2 takeoff_dir;
static struct FloatVect3 bungee_point;

static void compute_points_from_bungee(void)
{
  // Store init point (current position, where the plane will be released)
  VECT2_ASSIGN(init_point, stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y);
  // Compute unitary 2D vector (bungee_point - init_point) = takeoff direction
  VECT2_DIFF(takeoff_dir, bungee_point, init_point);
  float_vect2_normalize(&takeoff_dir);
  // Find throttle point (the point where the throttle line and launch line intersect)
  // If TakeOff_Distance is positive, throttle point is after bungee point, before otherwise
  VECT2_SMUL(throttle_point, takeoff_dir, BUNGEE_TAKEOFF_DISTANCE);
  VECT2_SUM(throttle_point, bungee_point, throttle_point);
}

void nav_bungee_takeoff_setup(uint8_t bungee_wp)
{
  // Store bungee point (from WP id, altitude is current hmsl (e.g. ground alt))
  VECT3_ASSIGN(bungee_point, WaypointX(bungee_wp), WaypointY(bungee_wp), stateGetPositionUtm_f()->alt);

  // Compute other points
  compute_points_from_bungee();

  // Enable Launch Status and turn kill throttle on
  CTakeoffStatus = Launch;
  autopilot_set_kill_throttle(true);
}

bool nav_bungee_takeoff_run(void)
{
  float cross = 0.;

  // Get current position
  struct FloatVect2 pos;
  VECT2_ASSIGN(pos, stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y);

  switch (CTakeoffStatus) {
    case Launch:
      // Recalculate lines if below min speed
      if (stateGetHorizontalSpeedNorm_f() < BUNGEE_TAKEOFF_MIN_SPEED) {
        compute_points_from_bungee();
      }

      // Follow Launch Line with takeoff pitch and no throttle
      NavVerticalAutoThrottleMode(BUNGEE_TAKEOFF_PITCH);
      NavVerticalThrottleMode(0);
      // FIXME previously using altitude mode, maybe not wise without motors
      //NavVerticalAltitudeMode(bungee_point.z + BUNGEE_TAKEOFF_HEIGHT, 0.);
      nav_route_xy(init_point.x, init_point.y, throttle_point.x, throttle_point.y);

      autopilot_set_kill_throttle(true);

      // Find out if UAV has crossed the line
      VECT2_DIFF(pos, pos, throttle_point); // position local to throttle_point
      cross = VECT2_DOT_PRODUCT(pos, takeoff_dir);

      if (cross > 0. && stateGetHorizontalSpeedNorm_f() > BUNGEE_TAKEOFF_MIN_SPEED) {
        CTakeoffStatus = Throttle;
        autopilot_set_kill_throttle(false);
        nav_init_stage();
      } else {
        // If not crossed stay in this status
        break;
      }
    // Start throttle imidiatelly
    case Throttle:
      //Follow Launch Line
      NavVerticalAutoThrottleMode(BUNGEE_TAKEOFF_PITCH);
      NavVerticalThrottleMode(MAX_PPRZ * (BUNGEE_TAKEOFF_THROTTLE));
      autopilot.launch = true; // turn on motor
      nav_route_xy(init_point.x, init_point.y, throttle_point.x, throttle_point.y);
      autopilot_set_kill_throttle(false);

      if ((stateGetPositionUtm_f()->alt > bungee_point.z + BUNGEE_TAKEOFF_HEIGHT)
#if USE_AIRSPEED
          && (stateGetAirspeed_f() > BUNGEE_TAKEOFF_AIRSPEED)
#endif
          ) {
        CTakeoffStatus = Finished;
        return false;
      } else {
        return true;
      }
      break;
    default:
      // Invalid status or Finished, end function
      return false;
  }
  return true;
}

