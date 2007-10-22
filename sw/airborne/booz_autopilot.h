#ifndef BOOZ_AUTOPILOT_H
#define BOOZ_AUTOPILOT_H

#include "std.h"

#define BOOZ_AP_MODE_FAILSAFE 0
#define BOOZ_AP_MODE_RATE     1
#define BOOZ_AP_MODE_ATTITUDE 2
#define BOOZ_AP_MODE_NAV      3

extern uint8_t booz_autopilot_mode;

extern void booz_autopilot_init(void);
void booz_autopilot_periodic_task(void);





#endif /* BOOZ_AUTOPILOT_H */
