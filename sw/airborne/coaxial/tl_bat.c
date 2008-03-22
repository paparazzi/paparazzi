#include "tl_bat.h"
#include CONFIG
#include "adc.h"
#include "airframe.h"

uint8_t tl_bat_decivolt;

struct adc_buf vsupply_adc_buf;

void tl_bat_init( void ) {
  adc_buf_channel(ADC_CHANNEL_VSUPPLY, &vsupply_adc_buf, DEFAULT_AV_NB_SAMPLE);
}


void tl_bat_periodic_task( void ) {
  tl_bat_decivolt = VoltageOfAdc((10*(vsupply_adc_buf.sum/vsupply_adc_buf.av_nb_sample)));
}
