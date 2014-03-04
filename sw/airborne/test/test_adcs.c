/*
 * Basic program periodically sending the values of the 8 ADCs
 */


#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/adc.h"
#include "messages.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"


#define NB_ADC 8
#define ADC_NB_SAMPLES 16

static struct adc_buf buf_adc[NB_ADC];

int main (int argc, char** argv) {
  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  led_init();
  adc_init();

  adc_buf_channel(ADC_0, &buf_adc[0], ADC_NB_SAMPLES);
  adc_buf_channel(ADC_1, &buf_adc[1], ADC_NB_SAMPLES);
  adc_buf_channel(ADC_2, &buf_adc[2], ADC_NB_SAMPLES);
  adc_buf_channel(ADC_3, &buf_adc[3], ADC_NB_SAMPLES);
  adc_buf_channel(ADC_4, &buf_adc[4], ADC_NB_SAMPLES);
  adc_buf_channel(ADC_5, &buf_adc[5], ADC_NB_SAMPLES);
#ifdef ADC_6
  adc_buf_channel(ADC_6, &buf_adc[6], ADC_NB_SAMPLES);
#endif
#ifdef ADC_7
  adc_buf_channel(ADC_7, &buf_adc[7], ADC_NB_SAMPLES);
#endif

#if NB_ADC != 8
#error "8 ADCs expected !"
#endif

#if USE_UART0
  uart_periph_init(&uart0);
#endif
#if USE_UART1
  uart_periph_init(&uart1);
#endif

  mcu_int_enable();

  while(1) {
    if (sys_time_check_and_ack_timer(0)) {
      LED_TOGGLE(1);
      uint16_t values[NB_ADC];
      uint8_t i;
      for(i = 0; i < NB_ADC; i++)
  values[i] = buf_adc[i].sum / ADC_NB_SAMPLES;

      uint8_t id = 42;
      DOWNLINK_SEND_ADC(&id, NB_ADC, values);
    }
  }
  return 0;
}
