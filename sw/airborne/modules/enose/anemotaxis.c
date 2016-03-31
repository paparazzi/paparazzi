#include "modules/enose/anemotaxis.h"
#include "generated/airframe.h"
#include "state.h"
#include "std.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/flight_plan.h"
#include "subsystems/datalink/downlink.h"
#include "modules/enose/chemo_detect.h"

enum status { UTURN, CROSSWIND };
static enum status status;
static int8_t sign;
static struct point last_plume;

static void last_plume_was_here(void)
{
  last_plume.x = stateGetPositionEnu_f()->x;
  last_plume.y = stateGetPositionEnu_f()->y;
}

bool nav_anemotaxis_downwind(uint8_t c, float radius)
{
  struct FloatVect2 *wind = stateGetHorizontalWindspeed_f();
  float wind_dir = atan2(wind->x, wind->y);
  waypoints[c].x = waypoints[WP_HOME].x + radius * cos(wind_dir);
  waypoints[c].y = waypoints[WP_HOME].y + radius * sin(wind_dir);
  return false;
}

bool nav_anemotaxis_init(uint8_t c)
{
  status = UTURN;
  sign = 1;
  struct FloatVect2 *wind = stateGetHorizontalWindspeed_f();
  float wind_dir = atan2(wind->x, wind->y);
  waypoints[c].x = stateGetPositionEnu_f()->x + DEFAULT_CIRCLE_RADIUS * cos(wind_dir + M_PI);
  waypoints[c].y = stateGetPositionEnu_f()->y + DEFAULT_CIRCLE_RADIUS * sin(wind_dir + M_PI);
  last_plume_was_here();
  return false;
}

bool nav_anemotaxis(uint8_t c, uint8_t c1, uint8_t c2, uint8_t plume)
{
  if (chemo_sensor) {
    last_plume_was_here();
    waypoints[plume].x = stateGetPositionEnu_f()->x;
    waypoints[plume].y = stateGetPositionEnu_f()->y;
    //    DownlinkSendWp(plume);
  }

  struct FloatVect2 *wind = stateGetHorizontalWindspeed_f();
  float wind_dir = atan2(wind->x, wind->y) + M_PI;

  /** Not null even if wind_east=wind_north=0 */
  float upwind_x = cos(wind_dir);
  float upwind_y = sin(wind_dir);

  switch (status) {
    case UTURN:
      NavCircleWaypoint(c, DEFAULT_CIRCLE_RADIUS * sign);
      if (NavQdrCloseTo(DegOfRad(M_PI_2 - wind_dir))) {
        float crosswind_x = - upwind_y;
        float crosswind_y = upwind_x;
        waypoints[c1].x = waypoints[c].x + DEFAULT_CIRCLE_RADIUS * upwind_x;
        waypoints[c1].y = waypoints[c].y + DEFAULT_CIRCLE_RADIUS * upwind_y;

        float width = Max(2 * ScalarProduct(upwind_x, upwind_y, stateGetPositionEnu_f()->x - last_plume.x,
                                            stateGetPositionEnu_f()->y - last_plume.y), DEFAULT_CIRCLE_RADIUS);

        waypoints[c2].x = waypoints[c1].x - width * crosswind_x * sign;
        waypoints[c2].y = waypoints[c1].y - width * crosswind_y * sign;

        //      DownlinkSendWp(c1);
        //      DownlinkSendWp(c2);

        status = CROSSWIND;
        nav_init_stage();
      }
      break;

    case CROSSWIND:
      NavSegment(c1, c2);
      if (NavApproaching(c2, CARROT)) {
        waypoints[c].x = waypoints[c2].x + DEFAULT_CIRCLE_RADIUS * upwind_x;
        waypoints[c].y = waypoints[c2].y + DEFAULT_CIRCLE_RADIUS * upwind_y;

        // DownlinkSendWp(c);

        sign = -sign;
        status = UTURN;
        nav_init_stage();
      }

      if (chemo_sensor) {
        waypoints[c].x = stateGetPositionEnu_f()->x + DEFAULT_CIRCLE_RADIUS * upwind_x;
        waypoints[c].y = stateGetPositionEnu_f()->y + DEFAULT_CIRCLE_RADIUS * upwind_y;

        // DownlinkSendWp(c);

        sign = -sign;
        status = UTURN;
        nav_init_stage();
      }
      break;
  }
  chemo_sensor = 0;
  return true;
}
