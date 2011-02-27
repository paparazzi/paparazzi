#include "subsystems/navigation/discsurvey.h"

#include "generated/airframe.h"
#include "estimator.h"
#include "std.h"
#include "subsystems/nav.h"
#include "generated/flight_plan.h"
#include "ap_downlink.h"

enum status { UTURN, SEGMENT, DOWNWIND };
static enum status status;
static int8_t sign;
static struct point c;
static struct point c1;
static struct point c2;

bool_t disc_survey_init( float grid ) {
  nav_survey_shift = grid;
  status = DOWNWIND;
  sign = 1;
  c1.x = estimator_x;
  c1.y = estimator_y;
  return FALSE;
}

bool_t disc_survey( uint8_t center, float radius) {
  float wind_dir = atan2(wind_north, wind_east) + M_PI;

  /** Not null even if wind_east=wind_north=0 */
  float upwind_x = cos(wind_dir);
  float upwind_y = sin(wind_dir);

  float grid = nav_survey_shift / 2;

  switch (status) {
  case UTURN:
    nav_circle_XY(c.x, c.y, grid*sign);
    if (NavQdrCloseTo(DegOfRad(M_PI_2-wind_dir))) {
      c1.x = estimator_x;
      c1.y = estimator_y;

      float d = ScalarProduct(upwind_x, upwind_y, estimator_x-WaypointX(center), estimator_y-WaypointY(center));
      if (d > radius) {
	status = DOWNWIND;
      } else {
	float w = sqrt(radius*radius - d*d) - 1.5*grid;

	float crosswind_x = - upwind_y;
	float crosswind_y = upwind_x;

	c2.x = WaypointX(center)+d*upwind_x-w*sign*crosswind_x;
	c2.y = WaypointY(center)+d*upwind_y-w*sign*crosswind_y;

	status = SEGMENT;
      }
      nav_init_stage();
    }
    break;

  case DOWNWIND:
    c2.x = WaypointX(center) - upwind_x * radius;
    c2.y = WaypointY(center) - upwind_y * radius;
    status = SEGMENT;
    /* No break; */

  case SEGMENT:
    nav_route_xy(c1.x, c1.y, c2.x, c2.y);
    if (nav_approaching_xy(c2.x, c2.y, c1.x, c1.y, CARROT)) {
      c.x = c2.x + grid*upwind_x;
      c.y = c2.y + grid*upwind_y;

      sign = -sign;
      status = UTURN;
      nav_init_stage();
    }
    break;
  }

  NavVerticalAutoThrottleMode(0.); /* No pitch */
  NavVerticalAltitudeMode(WaypointAlt(center), 0.); /* No preclimb */

  return TRUE;
}
