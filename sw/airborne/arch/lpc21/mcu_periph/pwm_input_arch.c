/*
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

/**
 * @file arch/lpc21/mcu_periph/pwm_input_arch.c
 * @ingroup lpc21_arch
 *
 * handling of arm7 PWM input using a timer with capture.
 */

#include "mcu_periph/pwm_input_arch.h"

#include "LPC21xx.h"
#include "armVIC.h"

//UPDATE THESE TO BE MORE ACCESSIBLE AND BE WARY OF EXISTING USAGE
//POSSIBLY MAKE MORE INPUTS ACCESSIBLE
#ifdef USE_PWM_INPUT1
//INPUT CAPTURE CAP0.3 on P0.29
#define PWM_INPUT1_PINSEL     PINSEL1
#define PWM_INPUT1_PINSEL_BIT 26
#define PWM_INPUT1_PINSEL_VAL (0x2 << PWM_INPUT1_PINSEL_BIT)
#define PWM_INPUT1_PINSEL_MASK (0x3 <<PWM_INPUT1_PINSEL_BIT)
#endif
#ifdef USE_PWM_INPUT2
//INPUT CAPTURE CAP0.0 on P0.30
#define PWM_INPUT2_PINSEL     PINSEL1
#define PWM_INPUT2_PINSEL_BIT 28
#define PWM_INPUT2_PINSEL_VAL (0x3 << PWM_INPUT2_PINSEL_BIT)
#define PWM_INPUT2_PINSEL_MASK (0x3 <<PWM_INPUT2_PINSEL_BIT)
#endif

void pwm_input_init(void)
{
  // initialize the arrays to 0 to avoid junk
  for (int i = 0; i < PWM_INPUT_NB; i++) {
    pwm_input_duty_tics[i] = 0;
    pwm_input_duty_valid[i] = 0;
    pwm_input_period_tics[i] = 0;
    pwm_input_period_valid[i] = 0;
  }
  /* select pin for capture */
#ifdef USE_PWM_INPUT1
  PWM_INPUT1_PINSEL = (PWM_INPUT1_PINSEL & ~PWM_INPUT1_PINSEL_MASK) | PWM_INPUT1_PINSEL_VAL;
  //enable capture 0.3 on rising edge + trigger interrupt
  T0CCR |= TCCR_CR3_R | TCCR_CR3_I;
#endif
#ifdef USE_PWM_INPUT2
  PWM_INPUT2_PINSEL = (PWM_INPUT2_PINSEL & ~PWM_INPUT2_PINSEL_MASK) | PWM_INPUT2_PINSEL_VAL;
  //enable capture 0.0 on rising edge + trigger interrupt
  T0CCR |= TCCR_CR0_R | TCCR_CR0_I;
#endif
}

//FIXME what about clock time overflow???
#ifdef USE_PWM_INPUT1
void pwm_input_isr1(void)
{
  static uint32_t t_rise;
  static uint32_t t_fall;
#if USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_HIGH
  static uint32_t t_oldrise = 0;
#elif USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_LOW
  static uint32_t t_oldfall = 0;
#endif

  if (T0CCR & TCCR_CR3_F) {
    t_fall = T0CR3;
    T0CCR |= TCCR_CR3_R;
    T0CCR &= ~TCCR_CR3_F;
#if USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_HIGH
    pwm_input_duty_tics[0] = t_fall - t_rise;
    pwm_input_duty_valid[0] = true;
#elif USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_LOW
    pwm_input_period_tics[0] = t_fall - t_oldfall;
    pwm_input_period_valid[0] = true;
    t_oldfall = t_fall;
#endif //ACTIVE_HIGH
  } else if (T0CCR & TCCR_CR3_R) {
    t_rise = T0CR3;
    T0CCR |= TCCR_CR3_F;
    T0CCR &= ~TCCR_CR3_R;
#if USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_LOW
    pwm_input_duty_tics[0] = t_rise - t_fall;
    pwm_input_duty_valid[0] = true;
#elif USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_HIGH
    pwm_input_period_tics[0] = t_rise - t_oldrise;
    pwm_input_period_valid[0] = true;
    t_oldrise = t_rise;
#endif //ACTIVE_LOW
  }
}
#endif //USE_PWM_INPUT1

#ifdef USE_PWM_INPUT2
void pwm_input_isr2(void)
{
  static uint32_t t_rise;
  static uint32_t t_fall;
#if USE_PWM_INPUT2 == PWM_PULSE_TYPE_ACTIVE_HIGH
  static uint32_t t_oldrise = 0;
#elif USE_PWM_INPUT2 == PWM_PULSE_TYPE_ACTIVE_LOW
  static uint32_t t_oldfall = 0;
#endif

  if (T0CCR & TCCR_CR0_F) {
    t_fall = T0CR0;
    T0CCR |= TCCR_CR0_R;
    T0CCR &= ~TCCR_CR0_F;
#if USE_PWM_INPUT2 == PWM_PULSE_TYPE_ACTIVE_HIGH
    pwm_input_duty_tics[1] = t_fall - t_rise;
    pwm_input_duty_valid[1] = true;
#elif USE_PWM_INPUT2 == PWM_PULSE_TYPE_ACTIVE_LOW
    pwm_input_period_tics[1] = t_fall - t_oldfall;
    pwm_input_period_valid[1] = true;
    t_oldfall = t_fall;
#endif //ACTIVE_HIGH
  } else if (T0CCR & TCCR_CR0_R) {
    t_rise = T0CR0;
    T0CCR |= TCCR_CR0_F;
    T0CCR &= ~TCCR_CR0_R;
#if USE_PWM_INPUT2 == PWM_PULSE_TYPE_ACTIVE_LOW
    pwm_input_duty_tics[1] = t_rise - t_fall;
    pwm_input_duty_valid[1] = true;
#elif USE_PWM_INPUT2 == PWM_PULSE_TYPE_ACTIVE_HIGH
    pwm_input_period_tics[1] = t_rise - t_oldrise;
    pwm_input_period_valid[1] = true;
    t_oldrise = t_rise;
#endif //ACTIVE_LOW
  }
}
#endif //USE_PWM_INPUT2
