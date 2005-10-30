#ifndef ADC_H
#define ADC_H

#include "types.h"

void adcInit ( void );
uint16_t adcPoll ( void );
void adcISR ( void ) __attribute__((naked));

#endif /* ADC_H */
