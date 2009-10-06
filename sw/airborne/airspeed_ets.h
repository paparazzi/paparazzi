/*
 * Driver for the EagleTree Systems Airspeed Sensor
 */

#ifndef AIRSPEED_ETS_H
#define AIRSPEED_ETS_H

#include "std.h"

extern void airspeed_ets_periodic( void );
extern void airspeed_ets_read( void );
extern void airspeed_ets_init( void );

extern uint16_t airspeed_ets_raw;
extern uint16_t airspeed_ets_offset;
extern bool_t airspeed_ets_valid;
extern float airspeed_ets;

#endif // AIRSPEED_ETS_H
