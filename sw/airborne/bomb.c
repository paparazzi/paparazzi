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

#define CLIMB_TIME 3  /* s */
#define SAFE_CLIMB 20 /* m */

static float bomb_x, bomb_y, bomb_z;
static float bomb_vx, bomb_vy, bomb_vz;


static void integrate( uint8_t wp_target ) {
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

  waypoints[WP_RELEASE].x = waypoints[wp_target].x - bomb_x;
  waypoints[WP_RELEASE].y = waypoints[wp_target].y - bomb_y;
}


/** Update the RELEASE location with the actual ground speed and altitude */
unit_t bomb_update_release( uint8_t wp_target ) {

  bomb_z = estimator_z - waypoints[wp_target].a;
  bomb_x = 0.;
  bomb_y = 0.;

  bomb_vx = estimator_hspeed_mod * sin(estimator_hspeed_dir);
  bomb_vy = estimator_hspeed_mod * cos(estimator_hspeed_dir);
  bomb_vz = 0.;

  integrate(wp_target);

  return 0;
}


/** Compute a first approximation for the RELEASE waypoint from wind and
    expected airspeed and altitude */
unit_t bomb_compute_approach( uint8_t wp_target, uint8_t wp_start, float bomb_radius ) {
  waypoints[WP_RELEASE].a = waypoints[wp_start].a;
  bomb_z = waypoints[WP_RELEASE].a - waypoints[wp_target].a;
  bomb_x = 0.;
  bomb_y = 0.;

  /* We approximate vx and vy, taking into account that RELEASE is next to
     TARGET */
  float x_0 = waypoints[wp_target].x - waypoints[wp_start].x;
  float y_0 = waypoints[wp_target].y - waypoints[wp_start].y;

  /* Unit vector from START to TARGET */
  float d = sqrt(x_0*x_0+y_0*y_0);
  float x1 = x_0 / d;
  float y_1 = y_0 / d;

  waypoints[WP_BASELEG].x = waypoints[wp_start].x + y_1 * bomb_radius;
  waypoints[WP_BASELEG].y = waypoints[wp_start].y - x1 * bomb_radius;
  waypoints[WP_BASELEG].a = waypoints[wp_start].a;
  bomb_start_qdr = M_PI - atan2(-y_1, -x1);
  if (bomb_radius < 0) 
    bomb_start_qdr += M_PI;

  bomb_vx = x1 * airspeed + wind_east;
  bomb_vy = y_1 * airspeed + wind_north;
  bomb_vz = 0.;

  float vx0 = bomb_vx;
  float vy0 = bomb_vy;

  integrate(wp_target);

  waypoints[WP_CLIMB].x = waypoints[WP_RELEASE].x + (CLIMB_TIME + CARROT) * vx0;
  waypoints[WP_CLIMB].y = waypoints[WP_RELEASE].y + (CLIMB_TIME + CARROT) * vy0;
  waypoints[WP_CLIMB].a = waypoints[WP_RELEASE].a + SAFE_CLIMB;

  return 0;
}



unit_t bomb_shoot( void ) {
  ap_state->commands[COMMAND_HATCH] = MAX_PPRZ;
  return 0;
}

/* Compute start and end waypoints to be aligned on w1-w2 */
bool_t compute_alignment(uint8_t w1, uint8_t w2, uint8_t wp_before, uint8_t wp_after, float d_before, float d_after) {
  float x_0 = waypoints[w2].x - waypoints[w1].x;
  float y_0 = waypoints[w2].y - waypoints[w1].y;

  /* Unit vector from W1 to W2 */
  float d = sqrt(x_0*x_0+y_0*y_0);
  x_0 /= d;
  y_0 /= d;

  waypoints[wp_before].x = waypoints[w1].x - d_before * x_0;
  waypoints[wp_before].y = waypoints[w1].y - d_before * y_0;
  waypoints[wp_after].x = waypoints[w2].x + d_after * x_0;
  waypoints[wp_after].y = waypoints[w2].y + d_after * y_0;

  return FALSE;
}

#endif /* WP_RELEASE */
