#include "adc_generic.h"
#include "adc.h"
#include CONFIG

#ifdef ADC_CHANNEL_GENERIC1
uint16_t adc_generic_val1;
static struct adc_buf buf_generic1;
#endif

#ifdef ADC_CHANNEL_GENERIC2
uint16_t adc_generic_val2;
static struct adc_buf buf_generic2;
#endif


void adc_generic_init( void ) {
#ifdef ADC_CHANNEL_GENERIC1
  adc_buf_channel(ADC_CHANNEL_GENERIC1, &buf_generic1, ADC_CHANNEL_GENERIC_NB_SAMPLES);
#endif
#ifdef ADC_CHANNEL_GENERIC2
  adc_buf_channel(ADC_CHANNEL_GENERIC2, &buf_generic2, ADC_CHANNEL_GENERIC_NB_SAMPLES);
#endif
}

void adc_generic_periodic( void ) {
#ifdef ADC_CHANNEL_GENERIC1
  adc_generic_val1 = buf_generic1.sum / buf_generic1.av_nb_sample;
#endif
#ifdef ADC_CHANNEL_GENERIC2
  adc_generic_val2 = buf_generic2.sum / buf_generic2.av_nb_sample;
#endif
}
