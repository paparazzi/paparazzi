#include "booz_energy.h"
#include CONFIG
#include "adc.h"



uint8_t booz_energy_bat;
static struct adc_buf bat_adc_buf;

void booz_energy_init( void ) {
  adc_buf_channel(ADC_BAT, &bat_adc_buf, DEFAULT_AV_NB_SAMPLE);
}

void booz_energy_periodic( void ) {
  booz_energy_bat =  bat_adc_buf.sum * 0.004295;
}
