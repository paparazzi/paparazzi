#include <math.h>

#include "tl_nav.h"
#include "tl_estimator.h"
#include "tl_control.h"
#include "gps.h"

#define NavGotoWaypoint(_wp) {}
#define NavVerticalAutoThrottleMode(_foo) {}
#define NavVerticalAltitudeMode(_foo, _bar) {}

static void nav_home(void) {
}

#define NAV_C
#include "flight_plan.h"

void tl_nav_init(void) {
  nav_block = 0;
  nav_stage = 0;
}

void tl_nav_periodic_task(void) {
  compute_dist2_to_home();
  auto_nav();
}

void nav_init_stage( void ) {
  stage_time = 0;
}




