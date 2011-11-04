/*  $Id$
 *
 * Copyright (C) 2011 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \brief handling of arm7 PWM input using a timer with capture
 *  
 */

#ifndef PWM_INPUT_ARCH_H
#define PWM_INPUT_ARCH_H

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h"

#define PWM_INPUT_NB 4 //this is architecture dependent

//FIXME what about clock time overflow???
#ifdef USE_PWM_INPUT1
static inline void pwm_input_isr1()
{
  static uint32_t t_rise;
  static uint32_t t_fall;
  static uint32_t t_oldrise = 0;
  static uint32_t t_oldfall = 0;

  if (T0CCR & TCCR_CR3_F) {
    t_fall = T0CR3;
    T0CCR |= TCCR_CR3_R;
    T0CCR &= ~TCCR_CR3_F;
#if USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_HIGH
    pwm_input_duty_tics[0] = t_fall - t_rise;
    pwm_input_duty_valid[0] = TRUE;
#elif USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_LOW
  pwm_input_period_tics[0] = t_fall - t_oldfall;
  pwm_input_period_valid[0] = TRUE;
  t_oldfall = t_fall;
#endif //ACTIVE_HIGH
  } else if (T0CCR & TCCR_CR3_R) {
    t_rise = T0CR3;
    T0CCR |= TCCR_CR3_F;
    T0CCR &= ~TCCR_CR3_R;
#if USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_LOW
    pwm_input_duty_tics[0] = t_rise - t_fall;
    pwm_input_duty_valid[0] = TRUE;
#elif USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_HIGH
  pwm_input_period_tics[0] = t_rise - t_oldrise;
  pwm_input_period_valid[0] = TRUE;
  t_oldrise = t_rise;
#endif //ACTIVE_LOW
  }
}
#define PWM_INPUT_IT1 TIR_CR3I
#define PWM_INPUT_ISR_1() pwm_input_isr1()
#endif //USE_PWM_INPUT1

#ifdef USE_PWM_INPUT2
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
#define PWM_INPUT_IT2 TIR_CR0I
#define PWM_INPUT_ISR_2() pwm_input_isr2()
#endif //USE_PWM_INPUT2

/*
#ifdef USE_PWM_INPUT3
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
#define PWM_INPUT_IT3 TIR_CR1I
#define PWM_INPUT_IT4 TIR_CR2I
#endif //USE_PWM_INPUT3

#ifdef USE_PWM_INPUT4
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
#define PWM_INPUT_ISR_3() pwm_input_isr3()
#define PWM_INPUT_ISR_4() pwm_input_isr4()
#endif //USE_PWM_INPUT4
*/

#endif /* PWM_INPUT_ARCH_H */
