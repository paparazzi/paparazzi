#include "csc_airspeed.h"
#include "csc_ap_link.h"
#include "csc_adc.h"

float estimator_airspeed;

#define AIRSPEED_ADC 3

void csc_airspeed_periodic(void)
{
  float adc_airspeed;

  adc_airspeed = adc_values[AIRSPEED_ADC];
  csc_ap_link_send_airspeed(estimator_airspeed, adc_airspeed);
}
