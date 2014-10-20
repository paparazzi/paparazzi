#include "temp_adc.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include BOARD_CONFIG

uint16_t adc_raw;
float temp_c1, temp_c2, temp_c3;


#ifndef TEMP_ADC_CHANNEL1
#ifndef TEMP_ADC_CHANNEL2
#ifndef TEMP_ADC_CHANNEL3
#error "at least one TEMP_ADC_CHANNEL1/2/3 needs to be defined to use the temp_adc module"
#endif
#endif
#endif

#ifdef TEMP_ADC_CHANNEL1
static struct adc_buf temp_buf1;
#endif

#ifdef TEMP_ADC_CHANNEL2
static struct adc_buf temp_buf2;
#endif

#ifdef TEMP_ADC_CHANNEL3
static struct adc_buf temp_buf3;
#endif

#ifndef TEMP_ADC_NB_SAMPLES
#define TEMP_ADC_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

float calc_ntc(int16_t raw_temp){
  float temp_c;
  //calc for NTC
  temp_c = log(((10240000/raw_temp)-10000)*10);
  //temp_c = 1/(0.001129148+(0.000234125*temp_c)+(0.0000000876741*temp_c*temp_c*temp_c));
  temp_c = 1/(0.000603985662844+(0.000229995493730*temp_c)+(0.000000067653027*temp_c*temp_c*temp_c));
  temp_c = temp_c - 273.15; //convert do celcius
  return temp_c;   
}

float calc_lm35(int16_t raw_temp){
  return ((float)raw_temp * (3300.0f/1024.0f)/10.0f);
}

void temp_adc_init( void ) {
#ifdef TEMP_ADC_CHANNEL1
  adc_buf_channel(TEMP_ADC_CHANNEL1, &temp_buf1, TEMP_ADC_NB_SAMPLES);
#endif  
  
#ifdef TEMP_ADC_CHANNEL2
  adc_buf_channel(TEMP_ADC_CHANNEL2, &temp_buf2, TEMP_ADC_NB_SAMPLES);
#endif  
  
#ifdef TEMP_ADC_CHANNEL3
  adc_buf_channel(TEMP_ADC_CHANNEL3, &temp_buf3, TEMP_ADC_NB_SAMPLES);
#endif  
}


void temp_adc_periodic( void ) {
  
#ifdef TEMP_ADC_CHANNEL1
  adc_raw = temp_buf1.sum / temp_buf1.av_nb_sample;
  #ifdef TEMP_ADC_CHANNEL1_TYPE_LM35
    temp_c1 = calc_lm35(adc_raw);
  #endif
  #ifdef TEMP_ADC_CHANNEL1_TYPE_NTC
    temp_c1 = calc_ntc (&adc_raw);
  #endif
#endif    
  
#ifdef TEMP_ADC_CHANNEL2
  adc_raw = temp_buf2.sum / temp_buf2.av_nb_sample;
  #ifdef TEMP_ADC_CHANNEL2_TYPE_LM35
    temp_c2 = calc_lm35(adc_raw);
  #endif
  #ifdef TEMP_ADC_CHANNEL1_TYPE_NTC
    temp_c2 = calc_ntc (&adc_raw);
  #endif
#endif  

#ifdef TEMP_ADC_CHANNEL3
  adc_raw = temp_buf3.sum / temp_buf3.av_nb_sample;
  #ifdef TEMP_ADC_CHANNEL3_TYPE_LM35
  temp_c3 = calc_lm35(adc_raw);
  #endif
  #ifdef TEMP_ADC_CHANNEL3_TYPE_NTC
    temp_c3 = calc_ntc (&adc_raw);
  #endif  
#endif 

  DOWNLINK_SEND_ADC_TEMP(DefaultChannel, DefaultDevice, &temp_c1, &temp_c2, &temp_c3);
}
             
