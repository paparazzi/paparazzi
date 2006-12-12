#include "airframe.h"
#include "estimator.h"
#include "std.h"
#include "nav.h"
#include "flight_plan.h"
#include "ap_downlink.h"

enum status { UTURN, CROSSWIND };
static enum status status;
static int8_t sign;
static int8_t upwind;
static struct point c;
static struct point c1;
static struct point c2;

bool_t disc_survey_init( void ) {
  status = UTURN;
  sign = 1;
  c.x = estimator_x;
  c.y = estimator_y;
  upwind = 1;
  return FALSE;
}

bool_t disc_survey( uint8_t center, float radius ) {
  float wind_dir = atan2(wind_north, wind_east) + M_PI;

  /** Not null even if wind_east=wind_north=0 */
  float upwind_x = cos(wind_dir);
  float upwind_y = sin(wind_dir);

  switch (status) {
  case UTURN:
    nav_circle_XY(c.x, c.y, MIN_CIRCLE_RADIUS*sign);
    if (NavQdrCloseTo(DegOfRad(M_PI_2-wind_dir))) {
      c1.x = estimator_x;
      c1.y = estimator_y;

      float d = ScalarProduct(upwind_x, upwind_y, estimator_x-waypoints[center].x, estimator_y-waypoints[center].y);
      float w = sqrt(radius*radius - d*d) - 1.5*MIN_CIRCLE_RADIUS;

      float crosswind_x = - upwind_y;
      float crosswind_y = upwind_x;

      c2.x = waypoints[center].x+d*upwind_x-w*sign*crosswind_x;
      c2.y = waypoints[center].y+d*upwind_y-w*sign*crosswind_y;

      status = CROSSWIND;
      nav_init_stage();
    }
    break;

  case CROSSWIND:
    nav_route_xy(c1.x, c1.y, c2.x, c2.y);
    if (nav_approaching_xy(c2.x, c2.y, CARROT)) {
      c.x = c2.x + MIN_CIRCLE_RADIUS*upwind_x;
      c.y = c2.y + MIN_CIRCLE_RADIUS*upwind_y;

      sign = -sign;
      status = UTURN;
      nav_init_stage();
    }
    break;
  }
  return TRUE;
}
