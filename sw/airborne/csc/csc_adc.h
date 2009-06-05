#ifndef CSC_ADC_H
#define CSC_ADC_H

#include <inttypes.h>

extern uint8_t vsupply;

void csc_adc_init(void);
void csc_adc_periodic(void);

#endif /* CSC_ADC_H */

