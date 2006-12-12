#include "airframe.h"
#include "estimator.h"
#include "std.h"
#include "nav.h"
#include "flight_plan.h"
#include "ap_downlink.h"

uint8_t chemo_sensor;

#define MAX_RADIUS 500
#define ALPHA 0.5

static uint8_t last_plume_value;

static float radius;
static int8_t sign;

bool_t nav_chemotaxis_init( void ) {
  radius = MAX_RADIUS;
  last_plume_value = 0;
  sign = 1;
  return FALSE;
}

bool_t nav_chemotaxis( uint8_t c, uint8_t plume ) {

  if (chemo_sensor > last_plume_value) {
    /* Store this plume */
    waypoints[plume].x = estimator_x;
    waypoints[plume].y = estimator_y;
    DownlinkSendWp(plume);
    last_plume_value = chemo_sensor;

    /* Reduce the radius */
    sign = - sign;
    radius = sign * (MIN_CIRCLE_RADIUS+(255-chemo_sensor)/256.*(MAX_RADIUS-MIN_CIRCLE_RADIUS));

    /* Move the circle in this direction */
    float x = waypoints[c].x - waypoints[plume].x;
    float y = waypoints[c].y - waypoints[plume].y;
    waypoints[c].x = waypoints[plume].x + ALPHA * x;
    waypoints[c].y = waypoints[plume].y + ALPHA * y;
    DownlinkSendWp(c);
  }

  NavCircleWaypoint(c, radius);
  return TRUE;
}
