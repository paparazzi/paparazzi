#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h"

#define PWM_INPUT_NB 4



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
#if USE_PWM_INPUT2 == PWM_PULSE_TYPE_ACTIVE_LOW
#else
    pwm_input_duration[0] = t_fall - t_rise;
    pwm_input_valid[0] = TRUE;
#endif //ACTIVE_HIGH
  } else if (T0CCR & TCCR_CR3_R) {
    t_rise = T0CR3;
    T0CCR |= TCCR_CR3_F;
    T0CCR &= ~TCCR_CR3_R;
#if USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_LOW
    pwm_input_duration[0] = t_rise - t_fall;
    pwm_input_valid[0] = TRUE;
#endif //ACTIVE_LOW
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
#if USE_PWM_INPUT2 == PWM_PULSE_TYPE_ACTIVE_LOW
#else
    pwm_input_duration[1] = t_fall - t_rise;
    pwm_input_valid[1] = TRUE;
#endif //ACTIVE_HIGH
  } else if (T0CCR & TCCR_CR0_R) {
    t_rise = T0CR0;
    T0CCR |= TCCR_CR0_F;
    T0CCR &= ~TCCR_CR0_R;
#if USE_PWM_INPUT2 == PWM_PULSE_TYPE_ACTIVE_LOW
    pwm_input_duration[1] = t_rise - t_fall;
    pwm_input_valid[1] = TRUE;
#endif //ACTIVE_LOW
  }
}

static inline void pwm_input_isr3()
{
  static uint32_t t_rise;
  static uint32_t t_fall;

  if (T0CCR & TCCR_CR1_F) {
    t_fall = T0CR1;
    T0CCR |= TCCR_CR1_R;
    T0CCR &= ~TCCR_CR1_F;
#if USE_PWM_INPUT3 == PWM_PULSE_TYPE_ACTIVE_LOW
#else
    pwm_input_duration[2] = t_fall - t_rise;
    pwm_input_valid[2] = TRUE;
#endif //ACTIVE_HIGH
  } else if (T0CCR & TCCR_CR1_R) {
    t_rise = T0CR1;
    T0CCR |= TCCR_CR1_F;
    T0CCR &= ~TCCR_CR1_R;
#if USE_PWM_INPUT3 == PWM_PULSE_TYPE_ACTIVE_LOW
    pwm_input_duration[2] = t_rise - t_fall;
    pwm_input_valid[2] = TRUE;
#endif //ACTIVE_LOW
  }
}

static inline void pwm_input_isr4()
{
  static uint32_t t_rise;
  static uint32_t t_fall;

  if (T0CCR & TCCR_CR2_F) {
    t_fall = T0CR2;
    T0CCR |= TCCR_CR2_R;
    T0CCR &= ~TCCR_CR2_F;
#if USE_PWM_INPUT4 == PWM_PULSE_TYPE_ACTIVE_LOW
#else
    pwm_input_duration[3] = t_fall - t_rise;
    pwm_input_valid[3] = TRUE;
#endif //ACTIVE_HIGH
  } else if (T0CCR & TCCR_CR2_R) {
    t_rise = T0CR2;
    T0CCR |= TCCR_CR2_F;
    T0CCR &= ~TCCR_CR2_R;
#if USE_PWM_INPUT4 == PWM_PULSE_TYPE_ACTIVE_LOW
    pwm_input_duration[3] = t_rise - t_fall;
    pwm_input_valid[3] = TRUE;
#endif //ACTIVE_LOW
  }
}

#define PWM_INPUT_IT1 TIR_CR3I
#define PWM_INPUT_IT2 TIR_CR0I
#define PWM_INPUT_IT3 TIR_CR1I
#define PWM_INPUT_IT4 TIR_CR2I
#define PWM_INPUT_ISR_1()	pwm_input_isr1()
#define PWM_INPUT_ISR_2()	pwm_input_isr2()
#define PWM_INPUT_ISR_3()	pwm_input_isr3()
#define PWM_INPUT_ISR_4()	pwm_input_isr4()

#endif /* PWM_INPUT_H */
