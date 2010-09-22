#include "pwm_input.h"

#include "LPC21xx.h"

#include "interrupt_hw.h"

volatile uint32_t pwm_input_duration[PWM_INPUT_NB];
volatile uint8_t pwm_input_valid[PWM_INPUT_NB];

#ifdef USE_PWM_INPUT1
/* INPUT CAPTURE CAP0.3 on P0.29 */
#define PWM_INPUT1_PINSEL     PINSEL1
#define PWM_INPUT1_PINSEL_BIT 26
#define PWM_INPUT1_PINSEL_VAL (0x2 << PWM_INPUT1_PINSEL_BIT)
#define PWM_INPUT1_PINSEL_MASK (0x3 <<PWM_INPUT1_PINSEL_BIT)
#endif
#ifdef USE_PWM_INPUT2
/* INPUT CAPTURE CAP0.0 on P0.30 */
#define PWM_INPUT2_PINSEL     PINSEL1
#define PWM_INPUT2_PINSEL_BIT 28
#define PWM_INPUT2_PINSEL_VAL (0x3 << PWM_INPUT2_PINSEL_BIT)
#define PWM_INPUT2_PINSEL_MASK (0x3 <<PWM_INPUT2_PINSEL_BIT)
#endif
#ifdef USE_PWM_INPUT3
/* INPUT CAPTURE CAP0.1 on P0.27 */
#define PWM_INPUT3_PINSEL     PINSEL1
#define PWM_INPUT3_PINSEL_BIT 22
#define PWM_INPUT3_PINSEL_VAL (0x2 << PWM_INPUT3_PINSEL_BIT)
#define PWM_INPUT3_PINSEL_MASK (0x3 <<PWM_INPUT3_PINSEL_BIT)
#endif
#ifdef USE_PWM_INPUT4
/* INPUT CAPTURE CAP0.2 on P0.28 */
#define PWM_INPUT4_PINSEL     PINSEL1
#define PWM_INPUT4_PINSEL_BIT 24
#define PWM_INPUT4_PINSEL_VAL (0x2 << PWM_INPUT4_PINSEL_BIT)
#define PWM_INPUT4_PINSEL_MASK (0x3 <<PWM_INPUT4_PINSEL_BIT)
#endif

void pwm_input_init ( void )
{
  /* select pin for capture */
#ifdef USE_PWM_INPUT1
  PWM_INPUT1_PINSEL = (PWM_INPUT1_PINSEL & ~PWM_INPUT1_PINSEL_MASK) | PWM_INPUT1_PINSEL_VAL;
  /* enable capture 0.3 on falling edge + trigger interrupt */
  T0CCR |= TCCR_CR3_R | TCCR_CR3_I;
#endif
#ifdef USE_PWM_INPUT2
  PWM_INPUT2_PINSEL = (PWM_INPUT2_PINSEL & ~PWM_INPUT2_PINSEL_MASK) | PWM_INPUT2_PINSEL_VAL;
  /* enable capture 0.0 on falling edge + trigger interrupt */
  T0CCR |= TCCR_CR0_R | TCCR_CR0_I;
#endif
#ifdef USE_PWM_INPUT3
  PWM_INPUT3_PINSEL = (PWM_INPUT3_PINSEL & ~PWM_INPUT3_PINSEL_MASK) | PWM_INPUT3_PINSEL_VAL;
  /* enable capture 0.1 on falling edge + trigger interrupt */
  T0CCR |= TCCR_CR1_R | TCCR_CR1_I;
#endif
#ifdef USE_PWM_INPUT4
  PWM_INPUT4_PINSEL = (PWM_INPUT4_PINSEL & ~PWM_INPUT4_PINSEL_MASK) | PWM_INPUT4_PINSEL_VAL;
  /* enable capture 0.2 on falling edge + trigger interrupt */
  T0CCR |= TCCR_CR2_R | TCCR_CR2_I;
#endif
}
