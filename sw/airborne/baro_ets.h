/*
 * Driver for the EagleTree Systems Altitude Sensor
 */

#ifndef BARO_ETS_H
#define BARO_ETS_H

#include "std.h"

#define BARO_ETS_DT 0.05

extern void baro_ets_periodic( void );
extern void baro_ets_read( void );
extern void baro_ets_init( void );

extern uint16_t baro_ets_adc;
extern uint16_t baro_ets_offset;
extern bool_t baro_ets_valid;
extern bool_t baro_ets_updated;
extern bool_t baro_ets_enabled;
extern float baro_ets_altitude;
extern float baro_ets_r;
extern float baro_ets_sigma2;

#endif // BARO_ETS_H
