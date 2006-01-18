#ifndef COMMAND_H
#define COMMAND_H

//#include "inter_mcu.h"

#include "paparazzi.h"

//extern void command_init( void );
extern void command_set(const pprz_t values[]);
extern const pprz_t failsafe_values[];

#include ACTUATORS


#endif /*  COMMAND_H */
