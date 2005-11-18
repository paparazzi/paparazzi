#ifndef ADC_H
#define ADC_H

#include "types.h"


void adcInit ( void );
uint16_t adcPoll ( void );
void adcISR0 ( void ) __attribute__((naked));
void adcISR1 ( void ) __attribute__((naked));

#define ADC_NB_CHAN 6
extern volatile uint16_t adc0_val[ADC_NB_CHAN];
extern volatile uint16_t adc1_val[ADC_NB_CHAN];


#endif /* ADC_H */
