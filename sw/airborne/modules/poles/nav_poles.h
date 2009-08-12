#ifndef NAV_POLS_H
#define NAV_POLS_H

#include <stdbool.h>
#include "std.h"

extern uint8_t nav_poles_count;

bool nav_poles_init(uint8_t wp1, uint8_t wp2,
                    uint8_t wp1c, uint8_t wp2c,
		    float radius );

#endif
