#include "estimator.h"
#include "nav.h"
#include "bomb.h"
#include "flight_plan.h"
#include "airframe.h"
#include "inter_mcu.h"


#if defined WP_RELEASE
/** Speed limit of the paint ball */
#ifndef TRIGGER_DELAY
#define TRIGGER_DELAY 1.
#endif

#define ALPHA 6.26e-5
#define MASS 3.31e-3
#define ALPHA_M (ALPHA / MASS)
#define DT 0.1
#define MAX_STEPS 100

float bomb_trigger_delay = TRIGGER_DELAY;
float airspeed = 14.;
float bomb_start_qdr;

#define CLIMB_TIME 5  /* s */
#define SAFE_CLIMB 20 /* m */

static float bomb_x, bomb_y, bomb_z;
static float bomb_vx, bomb_vy, bomb_vz;


static void integrate( void ) {
  /* Inspired from Arnold Schroeter's code */
  int i = 0;
  while (bomb_z > 0. && i < MAX_STEPS) {
    /* relative wind experienced by the ball */
    float airx = -bomb_vx + wind_east;     
    float airy = -bomb_vy + wind_north;     
    float airz = -bomb_vz;

    /* alpha / m * air */
    float beta = ALPHA_M * sqrt(airx*airx+airy*airy+airz*airz);

    /* Euler integration */
    bomb_vx += airx * beta * DT;
    bomb_vy += airy * beta * DT;
    bomb_vz += (airz * beta - G) * DT;

    bomb_x += bomb_vx * DT;
    bomb_y += bomb_vy * DT;
    bomb_z += bomb_vz * DT;

    i++;
  }

  if (bomb_z > 0.) {
    /* Integration not finished -> approximation of the time with the current
       speed  */
    float t = - bomb_z / bomb_vz;
    bomb_x += bomb_vx * t;
    bomb_y += bomb_vy * t;
  }

  waypoints[WP_RELEASE].x = waypoints[WP_TARGET].x - bomb_x;
  waypoints[WP_RELEASE].y = waypoints[WP_TARGET].y - bomb_y;
}


/** Update the RELEASE location with the actual ground speed and altitude */
unit_t bomb_update_release( void ) {

  bomb_z = estimator_z - waypoints[WP_TARGET].a;
  bomb_x = 0.;
  bomb_y = 0.;

  bomb_vx = estimator_hspeed_mod * sin(estimator_hspeed_dir);
  bomb_vy = estimator_hspeed_mod * cos(estimator_hspeed_dir);
  bomb_vz = 0.;

  integrate();

  return 0;
}


/** Compute a first approximation for the RELEASE waypoint from wind and
    expected airspeed and altitude */
unit_t bomb_compute_approach( void ) {
  waypoints[WP_RELEASE].a = waypoints[WP_START].a;
  bomb_z = waypoints[WP_RELEASE].a - waypoints[WP_TARGET].a;
  bomb_x = 0.;
  bomb_y = 0.;

  /* We approximate vx and vy, taking into account that RELEASE is next to
     TARGET */
  float x_0 = waypoints[WP_TARGET].x - waypoints[WP_START].x;
  float y_0 = waypoints[WP_TARGET].y - waypoints[WP_START].y;

  /* Unit vector from START to TARGET */
  float d = sqrt(x_0*x_0+y_0*y_0);
  float x1 = x_0 / d;
  float y_1 = y_0 / d;

  waypoints[WP_BASELEG].x = waypoints[WP_START].x + y_1 * BOMB_RADIUS;
  waypoints[WP_BASELEG].y = waypoints[WP_START].y - x1 * BOMB_RADIUS;
  waypoints[WP_BASELEG].a = waypoints[WP_START].a;
  bomb_start_qdr = M_PI - atan2(-y_1, -x1);

  bomb_vx = x1 * airspeed + wind_east;
  bomb_vy = y_1 * airspeed + wind_north;
  bomb_vz = 0.;

  float vx0 = bomb_vx;
  float vy0 = bomb_vy;

  integrate();

  waypoints[WP_CLIMB].x = waypoints[WP_RELEASE].x + CLIMB_TIME * vx0;
  waypoints[WP_CLIMB].y = waypoints[WP_RELEASE].y + CLIMB_TIME * vy0;
  waypoints[WP_CLIMB].a = waypoints[WP_RELEASE].a + SAFE_CLIMB;

  return 0;
}



unit_t bomb_shoot( void ) {
  ap_state->commands[COMMAND_HATCH] = MAX_PPRZ;
  return 0;
}

#endif /* WP_RELEASE */


#if defined WP_TD

float baseleg_alt, downwind_altitude;
const float baseleg_alt_tolerance = 15.;


/** Computes BASELEG from AF and TD */
unit_t compute_baseleg( void ) {
  float x_0 = waypoints[WP_TD].x - waypoints[WP_AF].x;
  float y_0 = waypoints[WP_TD].y - waypoints[WP_AF].y;

  /* Unit vector from AF to TD */
  float d = sqrt(x_0*x_0+y_0*y_0);
  float x_1 = x_0 / d;
  float y_1 = y_0 / d;

  waypoints[WP_BASELEG].x = waypoints[WP_AF].x + y_1 * BOMB_RADIUS;
  waypoints[WP_BASELEG].y = waypoints[WP_AF].y - x_1 * BOMB_RADIUS;
  waypoints[WP_BASELEG].a = waypoints[WP_AF].a;
  bomb_start_qdr = M_PI - atan2(-y_1, -x_1);

  baseleg_alt = waypoints[WP_BASELEG].a;
  downwind_altitude =  estimator_z;

  return 0;
}


static float overrun = 100.;

/** Computes TOD from AF and TD */
bool_t compute_tod( void ) {
  float z_dot = -2.;

  float x_0 = waypoints[WP_TD].x - waypoints[WP_AF].x;
  float y_0 = waypoints[WP_TD].y - waypoints[WP_AF].y;

  /* Unit vector from AF to TD */
  float d = sqrt(x_0*x_0+y_0*y_0);
  float x_1 = x_0 / d;
  float y_1 = y_0 / d;

  waypoints[WP_TOD].a = waypoints[WP_AF].a;
  float d1 = estimator_hspeed_mod * (waypoints[WP_TD].a - waypoints[WP_AF].a) / z_dot;

  waypoints[WP_TOD].x = waypoints[WP_TD].x - d1 * x_1;
  waypoints[WP_TOD].y = waypoints[WP_TD].y - d1 * y_1;

  waypoints[WP_OVERRUN].a = waypoints[WP_TD].a;
  waypoints[WP_OVERRUN].x = waypoints[WP_TD].x + overrun * x_1;
  waypoints[WP_OVERRUN].y = waypoints[WP_TD].y + overrun * y_1;

  float distance_to_TD_2 = DistanceSquare(estimator_x, estimator_y, waypoints[WP_TD].x, waypoints[WP_TD].y);
  return (distance_to_TD_2 < d1*d1);
}

#endif /* WP_TD */
