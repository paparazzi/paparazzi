#ifndef TEMP_ADC_H
#define TEMP_ADC_H

#include <inttypes.h>

extern uint16_t adc_raw;

float calc_ntc(int16_t raw_temp);
float calc_lm35(int16_t raw_temp);

void temp_adc_init( void );
void temp_adc_periodic( void );

#endif /* TEMP_ADC_H */
