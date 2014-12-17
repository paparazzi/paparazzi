#include "mb_scale.h"

volatile uint32_t mb_scale_pulse_len;
volatile float mb_scale_thrust;
volatile float mb_scale_torque;

volatile uint32_t mb_scale_neutral = 2892000; //2944640;
float    mb_scale_gain =  0.0018584; //1;
volatile uint8_t  mb_scale_calib;

void mb_scale_init(void)
{
  /* select pin for capture */
  ICP_PINSEL |= ICP_PINSEL_VAL << ICP_PINSEL_BIT;
  /* enable capture 0.3 on falling edge + trigger interrupt */
  T0CCR |= TCCR_CR3_F | TCCR_CR3_I;
  mb_scale_thrust = 0.;
  mb_scale_torque = 0.;
  mb_scale_calib = 0;
}
