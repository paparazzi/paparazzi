#include "adc.h"

static struct adc_buf* buffers[NB_ADC];

void adc_buf_channel(uint8_t adc_channel, struct adc_buf* s, uint8_t av_nb_sample) {
  buffers[adc_channel] = s;
  s->av_nb_sample = av_nb_sample;
}

void adc_init( void ) {

}
