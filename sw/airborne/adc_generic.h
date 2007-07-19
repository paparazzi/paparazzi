#ifndef ADC_GENERIC_H
#define ADC_GENERIC_H

#include <inttypes.h>

extern uint16_t adc_generic_val;
void adc_generic_init( void );
void adc_generic_periodic( void );

#endif /* ADC_GENERIC_H */
