#include "csc_adc.h"
#include "csc_ap_link.h"

#include "LPC21xx.h"
#include "led.h"
#include "adc.h"

struct adc_buf adc0;
struct adc_buf adc1;

#define ADC_VDIV 5.7
#define ADC_VOLT 3.28
#define ADC_FACTOR 1024.0 * ADC_VOLT * ADC_VDIV

#define ADC_AV_NB 8

void csc_adc_init(void)
{
  adc_init(); 
  adc_buf_channel(0, &adc0, ADC_AV_NB);
  adc_buf_channel(1, &adc1, ADC_AV_NB);
}

void csc_adc_periodic(void)
{
  float v1 = adc0.sum / adc0.av_nb_sample / ADC_FACTOR;
  float v2 = adc1.sum / adc1.av_nb_sample / ADC_FACTOR;
  csc_ap_link_send_adc(v1, v2);
}

