/*
 * Driver for the EagleTree Systems Airspeed Sensor
 * Modified by Mark Griffin on 8 September 2010 to work with new i2c transaction routines.
 */
#ifndef AIRSPEED_ETS_H
#define AIRSPEED_ETS_H

#include "std.h"
#include "i2c.h"

extern struct i2c_transaction airspeed_ets_i2c_trans;

extern void airspeed_ets_periodic( void );
extern void airspeed_ets_read( void );
extern void airspeed_ets_init( void );

extern uint16_t airspeed_ets_raw;
extern uint16_t airspeed_ets_offset;
extern bool_t airspeed_ets_valid;
extern float airspeed_ets;

#endif // AIRSPEED_ETS_H
