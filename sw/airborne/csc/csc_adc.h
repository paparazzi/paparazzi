#ifndef CSC_ADC_H
#define CSC_ADC_H

#include <inttypes.h>

void csc_adc_init(void);
void csc_adc_periodic(void);

extern uint8_t vsupply;
extern uint16_t adc_slider;
extern uint16_t adc_values[];

#endif /* CSC_ADC_H */

