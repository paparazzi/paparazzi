#ifndef WEATHER_METER_H
#define WEATHER_METER_H

#include <inttypes.h>

extern uint16_t adc_generic_val1;
int16_t get_wind_dir(uint16_t adc_wdir);
void weather_meter_init ( void );
void weather_meter_periodic( void );

#endif /* WEATHER_METER_H */
