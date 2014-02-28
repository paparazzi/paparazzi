#ifndef WEATHER_METER_HW_H
#define WEATHER_METER_HW_H

#include "std.h"

extern uint32_t trigger_t0;
extern uint32_t delta_t0;
extern volatile bool_t weather_meter_hw_valid;

void TRIG_ISR(void);
void weather_meter_hw_init( void );

#endif /* WEATHER_METER_HW_H */
