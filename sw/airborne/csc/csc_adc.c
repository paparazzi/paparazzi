#include "csc_adc.h"
#include "csc_ap_link.h"
#include <stdio.h>
#include "uart.h"
#include "print.h"

#include "LPC21xx.h"
#include "led.h"
#include "adc.h"
#include ACTUATORS
#include "csc_servos.h"
#include "sys_time.h"

struct adc_buf adc_bufs[ADC_NB];

uint8_t vsupply;
uint16_t adc_slider;

#define ADC_VDIV 5.7
#define ADC_VOLT 3.28
#define ADC_FACTOR 1024.0 * ADC_VOLT * ADC_VDIV

#ifndef ADC_AV_NB
#define ADC_AV_NB 8
#endif

#define ADC_VSUPPLY 0
#define ADC_SLIDER 3

#ifndef ADC_NB
#define ADC_NB 2
#endif

void csc_adc_init(void)
{
  adc_init(); 
  for (int i = 0; i < ADC_NB; i++) {
    adc_buf_channel(i, &adc_bufs[i], ADC_AV_NB);
  }
}

void csc_adc_periodic(void)
{
  vsupply = adc_bufs[ADC_VSUPPLY].sum / adc_bufs[ADC_VSUPPLY].av_nb_sample / ADC_FACTOR * 10;
  adc_slider = adc_bufs[ADC_SLIDER].sum / adc_bufs[ADC_SLIDER].av_nb_sample;
}

