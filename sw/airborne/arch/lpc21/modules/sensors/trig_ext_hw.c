
#include "std.h"
#include "mcu_periph/sys_time.h"
#include "LPC21xx.h"
#include "trig_ext_hw.h"
#include BOARD_CONFIG

uint32_t trigger_t0;
uint32_t delta_t0;
volatile bool_t trig_ext_valid;


void TRIG_ISR()
{
  static uint32_t last;
  uint32_t delta_t0_temp;
  trigger_t0 = PPM_CR;
  delta_t0_temp = trigger_t0 - last;
  if (msec_of_cpu_ticks(delta_t0_temp) > 10) {
    delta_t0 = delta_t0_temp;
    last = trigger_t0;
    trig_ext_valid = TRUE;
  }
}

void trig_ext_init(void)
{
  /* select pin for capture */
  PPM_PINSEL |= PPM_PINSEL_VAL << PPM_PINSEL_BIT;
  /* enable capture 0.2 on falling or rising edge + trigger interrupt */
#if defined TRIG_EXT_PULSE_TYPE && TRIG_EXT_PULSE_TYPE == TRIG_EXT_PULSE_TYPE_RISING
  T0CCR |= PPM_CCR_CRR | PPM_CCR_CRI;
#elif defined TRIG_EXT_PULSE_TYPE && TRIG_EXT_PULSE_TYPE == TRIG_EXT_PULSE_TYPE_FALLING
  T0CCR |= PPM_CCR_CRF | PPM_CCR_CRI;
#else
#error "trig_ext_hw.h: Unknown PULSE_TYPE"
#endif
  trig_ext_valid = FALSE;
}

