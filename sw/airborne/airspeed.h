#ifndef AIRSPEED_H
#define AIRSPEED_H

#include <inttypes.h>

#if defined USE_AIRSPEED || defined MEASURE_AIRSPEED 
extern uint16_t adc_airspeed_val;
#endif

void airspeed_init( void );
void airspeed_update( void );

#endif /* ADC_GENERIC_H */
