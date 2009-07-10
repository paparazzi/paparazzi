#include "pwm_input.h"

#include "LPC21xx.h"

#include "interrupt_hw.h"

volatile uint32_t pwm_input_duration;
volatile uint8_t pwm_input_valid;

/* INPUT CAPTURE CAP0.3 on P0.29 */
#define PWM_INPUT_PINSEL     PINSEL1
#define PWM_INPUT_PINSEL_BIT 26
#define PWM_INPUT_PINSEL_VAL (0x2 << PWM_INPUT_PINSEL_BIT)
#define PWM_INPUT_PINSEL_MASK (0x3 <<PWM_INPUT_PINSEL_BIT)

void pwm_input_init ( void )
{
  /* select pin for capture */
  PWM_INPUT_PINSEL = (PWM_INPUT_PINSEL & ~PWM_INPUT_PINSEL_MASK) | PWM_INPUT_PINSEL_VAL;
  /* enable capture 0.3 on falling edge + trigger interrupt */
  T0CCR |= TCCR_CR3_R | TCCR_CR3_I;
}
