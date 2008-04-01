#ifndef EXTERNAL_H
#define EXTERNAL_H

#include "nav.h"

extern float nav_roll; /* degrees, >0 on the right */
/* int nav_ground_speed_loop(void); */

extern float nav_throttle;
/* int nav_throttle_loop(void); */

extern float nav_cruise;
int setNavCruise(void);

extern float nav_alt;

#define external_SetNavThrottle(_v) { \
  nav_throttle = (_v ? _v : nav_throttle); \
  Bound(nav_throttle, V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE, V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE); \
}

#endif
