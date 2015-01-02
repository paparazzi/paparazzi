#ifndef NAV_POLS_H
#define NAV_POLS_H

#include <stdbool.h>
#include "std.h"

extern uint8_t nav_poles_count;
extern float nav_poles_time;
extern int8_t nav_poles_land;

bool nav_poles_init(uint8_t wp1, uint8_t wp2,
                    uint8_t wp1c, uint8_t wp2c,
                    float radius);

#define nav_poles_SetLandDir(_d) { if (_d < 0) _d = -1; else _d = 1; }

#endif
