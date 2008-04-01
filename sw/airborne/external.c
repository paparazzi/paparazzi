#define EXTERNAL_C

#include "external.h"
#include "flight_plan.h"

float nav_roll = 0.;
float nav_throttle = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
float nav_cruise = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
float nav_alt = SECURITY_ALT;

int setNavCruise( void ) {
  v_ctl_auto_throttle_cruise_throttle = nav_cruise;
  return FALSE;
}

/*
int nav_throttle_loop( void ) {
  nav_throttle_setpoint = TRIM_UPPRZ(nav_throttle * MAX_PPRZ);
  return FALSE;
}
*/

