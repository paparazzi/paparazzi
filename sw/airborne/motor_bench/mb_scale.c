#include "mb_scale.h"

volatile uint32_t mb_scale_pulse_len;
float mb_scale_thrust;
float mb_scale_torque;

uint32_t mb_scale_neutral = 2892000; //2944640;
float    mb_scale_gain = 1.69e-3; //2.72e-3;

void mb_scale_init ( void ) {
  /* select pin for capture */
  ICP_PINSEL |= ICP_PINSEL_VAL << ICP_PINSEL_BIT;
  /* enable capture 0.3 on falling edge + trigger interrupt */
  T0CCR |= TCCR_CR3_F | TCCR_CR3_I;
}
