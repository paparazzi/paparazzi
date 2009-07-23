#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h"

void pwm_input_init ( void );

/* tracks of length of positive pulse duration */
extern volatile uint32_t pwm_input_duration;
extern volatile uint8_t pwm_input_valid;

static inline void pwm_input_isr()
{
  static uint32_t t_rise;
  static uint32_t t_fall;

  if (T0CCR & TCCR_CR3_F) {
    t_fall = T0CR3;
    T0CCR |= TCCR_CR3_R;
    T0CCR &= ~TCCR_CR3_F;
    pwm_input_duration = t_fall - t_rise;
    pwm_input_valid = TRUE;
  } else {
    t_rise = T0CR3;
    T0CCR |= TCCR_CR3_F;
    T0CCR &= ~TCCR_CR3_R;
  }
}

#define PWM_INPUT_IT TIR_CR3I
#define PWM_INPUT_ISR()	pwm_input_isr()

#endif /* PWM_INPUT_H */
