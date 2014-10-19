
#include "std.h"
#include "mcu_periph/sys_time.h"
#include "LPC21xx.h"
#include "flow_meter_hw.h"
#include BOARD_CONFIG

uint32_t flow_pulse;
volatile bool_t flow_valid;


void TRIG_ISR() {
  static uint32_t pulse;
  pulse = pulse + 1;
  flow_pulse = pulse;
  flow_valid = TRUE;
}

void flow_hw_init ( void ) {
  /* select pin for capture */
  TRIG_EXT_PINSEL |= TRIG_EXT_PINSEL_VAL << TRIG_EXT_PINSEL_BIT;
  /* enable capture 0.2 on falling or rising edge + trigger interrupt */
#if defined FLOW_PULSE_TYPE_RISING
PRINT_CONFIG_MSG( "flow_hw: PULSE_TYPE RISING")
  T0CCR |= TRIGGER_CRR | TRIGGER_CRI;
#elif defined FLOW_PULSE_TYPE_FALLING
PRINT_CONFIG_MSG( "flow_hw: PULSE_TYPE FALLING")
  T0CCR |= TRIGGER_CRF | TRIGGER_CRI;
#else
#error "flow_hw: Unknown PULSE_TYPE"
#endif
  flow_valid = FALSE;
}

