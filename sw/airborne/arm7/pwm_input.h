#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h"

#define PWM_INPUT_NB 2

void pwm_input_init ( void );

/* tracks of length of positive pulse duration */
extern volatile uint32_t pwm_input_duration[PWM_INPUT_NB];
extern volatile uint8_t pwm_input_valid[PWM_INPUT_NB];

static inline void pwm_input_isr1()
{
  static uint32_t t_rise;
  static uint32_t t_fall;

  if (T0CCR & TCCR_CR3_F) {
    t_fall = T0CR3;
    T0CCR |= TCCR_CR3_R;
    T0CCR &= ~TCCR_CR3_F;
    pwm_input_duration[0] = t_fall - t_rise;
    pwm_input_valid[0] = TRUE;
  } else if (T0CCR & TCCR_CR3_R) {
    t_rise = T0CR3;
    T0CCR |= TCCR_CR3_F;
    T0CCR &= ~TCCR_CR3_R;
  }
}

static inline void pwm_input_isr2()
{
  static uint32_t t_rise;
  static uint32_t t_fall;

  if (T0CCR & TCCR_CR0_F) {
    t_fall = T0CR0;
    T0CCR |= TCCR_CR0_R;
    T0CCR &= ~TCCR_CR0_F;
    pwm_input_duration[1] = t_fall - t_rise;
    pwm_input_valid[1] = TRUE;
  } else if (T0CCR & TCCR_CR0_R) {
    t_rise = T0CR0;
    T0CCR |= TCCR_CR0_F;
    T0CCR &= ~TCCR_CR0_R;
  }
}

#define PWM_INPUT_IT1 TIR_CR3I
#define PWM_INPUT_IT2 TIR_CR0I
#define PWM_INPUT_ISR_1()	pwm_input_isr1()
#define PWM_INPUT_ISR_2()	pwm_input_isr2()

#endif /* PWM_INPUT_H */
