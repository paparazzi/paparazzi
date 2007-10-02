#include "mb_scale.h"

volatile uint32_t pulse_len;
float mb_scale_thrust;
float mb_scale_torque;

void mb_scale_init ( void ) {
  /* select pin for capture */
  ICP_PINSEL |= ICP_PINSEL_VAL << ICP_PINSEL_BIT;
  /* enable capture 0.3 on falling edge + trigger interrupt */
  T0CCR |= TCCR_CR3_F | TCCR_CR3_I;
}


void mb_scale_periodic(void) {
  mb_scale_thrust = 500. / 240500. * (pulse_len - 3073500);
}
