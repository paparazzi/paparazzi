
#include "std.h"
#include "mcu_periph/sys_time.h"
#include "LPC21xx.h"
#include "weather_meter_hw.h"
#include "led.h"
#include BOARD_CONFIG

uint32_t trigger_t0;
uint32_t delta_t0;
volatile bool_t weather_meter_hw_valid;


void TRIG_ISR() {
  static uint32_t last;
  uint32_t delta_t0_temp;
  trigger_t0 = PPM_CR;
  delta_t0_temp = trigger_t0 - last;
  if (MSEC_OF_CPU_TICKS(delta_t0_temp) > 10) {
    delta_t0 = delta_t0_temp;
    last = trigger_t0;
    weather_meter_hw_valid = TRUE;
    LED_TOGGLE(2);
  }
}

void weather_meter_hw_init(void) {
  /* select pin for capture */
  PPM_PINSEL |= PPM_PINSEL_VAL << PPM_PINSEL_BIT;
  /* enable capture 0.2 on falling edge + trigger interrupt */
  T0CCR = PPM_CCR_CRF | PPM_CCR_CRI;
  weather_meter_hw_valid = FALSE;
}
