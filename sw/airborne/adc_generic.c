#include "adc_generic.h"
#include "adc.h"
#include CONFIG

uint16_t adc_generic_val;

static struct adc_buf buf_generic;


void adc_generic_init( void ) {
  adc_buf_channel(ADC_CHANNEL_GENERIC, &buf_generic, ADC_CHANNEL_GENERIC_NB_SAMPLES);
}

void adc_generic_periodic( void ) {
  adc_generic_val = buf_generic.sum / buf_generic.av_nb_sample;
}
