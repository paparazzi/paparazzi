#include "temp_fine_wire.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include BOARD_CONFIG

uint16_t adc_fine_wire_val;

#define TEMP_ABS 273.15

#ifndef ADC_CHANNEL_TEMP_FINE_WIRE1
#warning "ADC_CHANNEL_TEMP_FINE_WIRE1 not defined, using AUX4 (ADC3) as default"
#define ADC_CHANNEL_TEMP_FINE_WIRE1 ADC_3
#define USE_ADC_3
#endif

#ifndef CAL_POINT_1_ADC
#error "CAL_POINT_1_ADC needs to be defined"
#endif
#ifndef CAL_POINT_1_TEMP
#error "CAL_POINT_1_TEMP needs to be defined"
#endif
#ifndef CAL_POINT_2_ADC
#error "CAL_POINT_2_ADC needs to be defined"
#endif
#ifndef CAL_POINT_2_TEMP
#error "CAL_POINT_2_TEMP needs to be defined"
#endif

#if (CAL_POINT_1_ADC == CAL_POINT_2_ADC)
#error "CAL_POINT_1_ADC and CAL_POINT_2_ADC need to be different"
#endif

#ifndef ADC_CHANNEL_TEMP_FINE_WIRE_NB_SAMPLES
#define ADC_CHANNEL_TEMP_FINE_WIRE_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

static struct adc_buf buf_temp_fine_wire1;
unsigned int pkt_count = 0;

void temp_fine_wire_init( void ) {
  adc_buf_channel(ADC_CHANNEL_TEMP_FINE_WIRE1, &buf_temp_fine_wire1, ADC_CHANNEL_TEMP_FINE_WIRE_NB_SAMPLES);
}

void temp_fine_wire_periodic( void ) {
  float ffast_temp;
  static float t1 = CAL_POINT_1_TEMP + TEMP_ABS, t2 = CAL_POINT_2_TEMP + TEMP_ABS;
  float m;

  pkt_count++;
  adc_fine_wire_val = buf_temp_fine_wire1.sum / buf_temp_fine_wire1.av_nb_sample;

  if ((CAL_POINT_2_ADC - CAL_POINT_1_ADC) != 0) {
    m = ((t2 - t1) / (CAL_POINT_2_ADC - CAL_POINT_1_ADC));
    ffast_temp = (t1 +  m * (adc_fine_wire_val - CAL_POINT_1_ADC)) - TEMP_ABS;
  }
  else {
    ffast_temp = -274;
  }

  DOWNLINK_SEND_TEMP_FINE_WIRE(DefaultChannel, DefaultDevice, &pkt_count, &adc_fine_wire_val, &ffast_temp);
}

