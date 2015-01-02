#include "modules/poles/nav_poles.h"
#include "subsystems/navigation/common_nav.h"

uint8_t nav_poles_count = 0;
float nav_poles_time = 0.;
int8_t nav_poles_land = 1;

#define SAFETY_MARGIN 0.7

/** computes position of wp1c and wp2c, reference points for an oval around
    waypoints wp1 and wp2 */
bool nav_poles_init(uint8_t wp1, uint8_t wp2,
                    uint8_t wp1c, uint8_t wp2c,
                    float radius)
{
  float x = WaypointX(wp2) - WaypointX(wp1);
  float y = WaypointY(wp2) - WaypointY(wp1);
  float d = sqrt(x * x + y * y);

  /* Unit vector from wp1 to wp2 */
  x /= d;
  y /= d;

  WaypointX(wp2c) = WaypointX(wp2) - (x * SAFETY_MARGIN + y) * radius;
  WaypointY(wp2c) = WaypointY(wp2) - (y * SAFETY_MARGIN - x) * radius;

  WaypointX(wp1c) = WaypointX(wp1) + (x * SAFETY_MARGIN - y) * radius;
  WaypointY(wp1c) = WaypointY(wp1) + (y * SAFETY_MARGIN + x) * radius;

  return false;
}
