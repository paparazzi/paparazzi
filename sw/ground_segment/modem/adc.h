#ifndef ADC_H
#define ADC_H

void adc_init( void );
extern uint16_t adc_alim;
extern volatile uint8_t adc_got_val;

#endif
