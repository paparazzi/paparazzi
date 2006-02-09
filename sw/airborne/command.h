#ifndef COMMAND_H
#define COMMAND_H

//#include "inter_mcu.h"

#include "paparazzi.h"

extern void command_init( void );
extern void command_set(const pprz_t values[]);

#if defined ACTUATORS
extern const pprz_t failsafe_values[];
extern void command_init( void );
#include ACTUATORS

#endif

#endif /*  COMMAND_H */
