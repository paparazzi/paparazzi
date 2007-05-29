#ifndef CHEMO_DETECT_H
#define CHEMO_DETECT_H

#include "std.h"

extern uint8_t chemo_sensor;
#define MAX_CHEMO 255

void chemo_init( void );
void chemo_periodic( void );

#endif /*  CHEMO_DETECT_H */
