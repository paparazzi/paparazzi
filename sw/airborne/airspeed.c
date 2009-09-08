#include "airspeed.h"
#include "adc.h"
#include "airframe.h"
#include "estimator.h"
#include BOARD_CONFIG

#ifdef USE_AIRSPEED
uint16_t adc_airspeed_val;
#else
#error "You compiled the airspeed.c file but did not USE_AIRSPEED, which is needed in other *.c files"
#endif

#ifdef ADC_CHANNEL_AIRSPEED
#ifndef SITL
static struct adc_buf buf_airspeed;
#endif
#else
#error "You compiled the airspeed.c file but did not ADC_CHANNEL_AIRSPEED, which is needed in other *.c files"
#endif

void airspeed_init( void ) {
#ifdef ADC_CHANNEL_AIRSPEED
#  ifndef ADC_CHANNEL_AIRSPEED_NB_SAMPLES
#    error "You defined USE_AIRSPEED but did not assign a ADC_CHANNEL_AIRSPEED_NB_SAMPLES"
#  endif
#  ifndef SITL
     adc_buf_channel(ADC_CHANNEL_AIRSPEED, &buf_airspeed, ADC_CHANNEL_AIRSPEED_NB_SAMPLES);
#  endif
#else
#  error "You defined USE_AIRSPEED but did not assign a ADC_CHANNEL_AIRSPEED"
#endif
}

void airspeed_update( void ) {
#ifdef ADC_CHANNEL_AIRSPEED
#ifndef SITL
  adc_airspeed_val = (buf_airspeed.sum / buf_airspeed.av_nb_sample) - AIRSPEED_ZERO;
  float airspeed = AIRSPEED_SCALE * adc_airspeed_val;
  EstimatorSetAirspeed(airspeed);
#endif
#endif
}
