/*
 * Copyright (C) 2014 Gautier Hattenberger
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
 */

/**
 * @file arch/stm32/mcu_periph/pwm_input_arch.c
 * @ingroup stm32_arch
 *
 * handling of smt32 PWM input using a timer with capture.
 */
#include "mcu_periph/pwm_input.h"

#include BOARD_CONFIG
#include "generated/airframe.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "mcu_periph/sys_time.h"
#include "mcu_periph/gpio.h"

// for timer_get_frequency
#include "mcu_arch.h"

#define ONE_MHZ_CLK 1000000
#ifdef NVIC_TIM_IRQ_PRIO
#define PWM_INPUT_IRQ_PRIO  NVIC_TIM_IRQ_PRIO
#else
#define PWM_INPUT_IRQ_PRIO 2
#endif

static inline void pwm_input_set_timer(uint32_t tim, uint32_t ticks_per_usec)
{
  timer_reset(tim);
  timer_set_mode(tim, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_period(tim, 0xFFFF);
  uint32_t timer_clk = timer_get_frequency(tim);
  timer_set_prescaler(tim, (timer_clk / (ticks_per_usec * ONE_MHZ_CLK)) - 1);
  timer_enable_counter(tim);
}

void pwm_input_init(void)
{
  int i;
  // initialize the arrays to 0
  for (i = 0; i < PWM_INPUT_NB; i++) {
    pwm_input_duty_tics[i] = 0;
    pwm_input_duty_valid[i] = 0;
    pwm_input_period_tics[i] = 0;
    pwm_input_period_valid[i] = 0;
  }

  /** Configure timers
   *  - timer clock enable
   *  - base configuration
   *  - enable counter
   */
#if USE_PWM_INPUT_TIM1
  rcc_periph_clock_enable(RCC_TIM1);
  pwm_input_set_timer(TIM1, TIM1_TICKS_PER_USEC);
#endif
#if USE_PWM_INPUT_TIM2
  rcc_periph_clock_enable(RCC_TIM2);
  pwm_input_set_timer(TIM2, TIM2_TICKS_PER_USEC);
#endif
#if USE_PWM_INPUT_TIM3
  rcc_periph_clock_enable(RCC_TIM3);
  pwm_input_set_timer(TIM3, TIM3_TICKS_PER_USEC);
#endif
#if USE_PWM_INPUT_TIM4
  rcc_periph_clock_enable(RCC_TIM4);
  pwm_input_set_timer(TIM4, TIM4_TICKS_PER_USEC);
#endif
#if USE_PWM_INPUT_TIM5
  rcc_periph_clock_enable(RCC_TIM5);
  pwm_input_set_timer(TIM5, TIM5_TICKS_PER_USEC);
#endif
#if USE_PWM_INPUT_TIM8
  rcc_periph_clock_enable(RCC_TIM8);
  pwm_input_set_timer(TIM8, TIM8_TICKS_PER_USEC);
#endif
#if USE_PWM_INPUT_TIM9
  rcc_periph_clock_enable(RCC_TIM9);
  pwm_input_set_timer(TIM9, TIM9_TICKS_PER_USEC);
#endif

#ifdef USE_PWM_INPUT1
  /* GPIO configuration as input capture for timer */
  gpio_setup_pin_af(PWM_INPUT1_GPIO_PORT, PWM_INPUT1_GPIO_PIN, PWM_INPUT1_GPIO_AF, FALSE);

  /** TIM configuration: Input Capture mode
   *  Two IC signals are mapped to the same TI input
   */
  timer_ic_set_input(PWM_INPUT1_TIMER, PWM_INPUT1_CHANNEL_PERIOD, PWM_INPUT1_TIMER_INPUT);
  timer_ic_set_input(PWM_INPUT1_TIMER, PWM_INPUT1_CHANNEL_DUTY, PWM_INPUT1_TIMER_INPUT);
#if USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_LOW
  timer_ic_set_polarity(PWM_INPUT1_TIMER, PWM_INPUT1_CHANNEL_PERIOD, TIM_IC_RISING);
  timer_ic_set_polarity(PWM_INPUT1_TIMER, PWM_INPUT1_CHANNEL_DUTY, TIM_IC_FALLING);
#elif USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_HIGH
  timer_ic_set_polarity(PWM_INPUT1_TIMER, PWM_INPUT1_CHANNEL_PERIOD, TIM_IC_FALLING);
  timer_ic_set_polarity(PWM_INPUT1_TIMER, PWM_INPUT1_CHANNEL_DUTY, TIM_IC_RISING);
#endif

  /* Select the valid trigger input */
  timer_slave_set_trigger(PWM_INPUT1_TIMER, PWM_INPUT1_SLAVE_TRIG);
  /* Configure the slave mode controller in reset mode */
  timer_slave_set_mode(PWM_INPUT1_TIMER, TIM_SMCR_SMS_RM);

  /* Enable timer Interrupt(s). */
  nvic_set_priority(PWM_INPUT1_IRQ, PWM_INPUT_IRQ_PRIO);
  nvic_enable_irq(PWM_INPUT1_IRQ);
#ifdef PWM_INPUT1_IRQ2
  nvic_set_priority(PWM_INPUT1_IRQ2, PWM_INPUT_IRQ_PRIO);
  nvic_enable_irq(PWM_INPUT1_IRQ2);
#endif

  /* Enable the Capture/Compare and Update interrupt requests. */
  timer_enable_irq(PWM_INPUT1_TIMER, (PWM_INPUT1_CC_IE | TIM_DIER_UIE));

  /* Enable capture channel. */
  timer_ic_enable(PWM_INPUT1_TIMER, PWM_INPUT1_CHANNEL_PERIOD);
  timer_ic_enable(PWM_INPUT1_TIMER, PWM_INPUT1_CHANNEL_DUTY);
#endif

#ifdef USE_PWM_INPUT2
  /* GPIO configuration as input capture for timer */
  gpio_setup_pin_af(PWM_INPUT2_GPIO_PORT, PWM_INPUT2_GPIO_PIN, PWM_INPUT2_GPIO_AF, FALSE);

  /** TIM configuration: Input Capture mode
   *  Two IC signals are mapped to the same TI input
   */
  timer_ic_set_input(PWM_INPUT2_TIMER, PWM_INPUT2_CHANNEL_PERIOD, PWM_INPUT2_TIMER_INPUT);
  timer_ic_set_input(PWM_INPUT2_TIMER, PWM_INPUT2_CHANNEL_DUTY, PWM_INPUT2_TIMER_INPUT);
#if USE_PWM_INPUT2 == PWM_PULSE_TYPE_ACTIVE_LOW
  timer_ic_set_polarity(PWM_INPUT2_TIMER, PWM_INPUT2_CHANNEL_PERIOD, TIM_IC_RISING);
  timer_ic_set_polarity(PWM_INPUT2_TIMER, PWM_INPUT2_CHANNEL_DUTY, TIM_IC_FALLING);
#elif USE_PWM_INPUT2 == PWM_PULSE_TYPE_ACTIVE_HIGH
  timer_ic_set_polarity(PWM_INPUT2_TIMER, PWM_INPUT2_CHANNEL_PERIOD, TIM_IC_FALLING);
  timer_ic_set_polarity(PWM_INPUT2_TIMER, PWM_INPUT2_CHANNEL_DUTY, TIM_IC_RISING);
#endif

  /* Select the valid trigger input */
  timer_slave_set_trigger(PWM_INPUT2_TIMER, PWM_INPUT2_SLAVE_TRIG);
  /* Configure the slave mode controller in reset mode */
  timer_slave_set_mode(PWM_INPUT2_TIMER, TIM_SMCR_SMS_RM);

  /* Enable timer Interrupt(s). */
  nvic_set_priority(PWM_INPUT2_IRQ, PWM_INPUT_IRQ_PRIO);
  nvic_enable_irq(PWM_INPUT2_IRQ);
#ifdef PWM_INPUT2_IRQ2
  nvic_set_priority(PWM_INPUT2_IRQ2, PWM_INPUT_IRQ_PRIO);
  nvic_enable_irq(PWM_INPUT2_IRQ2);
#endif

  /* Enable the Capture/Compare and Update interrupt requests. */
  timer_enable_irq(PWM_INPUT2_TIMER, (PWM_INPUT2_CC_IE | TIM_DIER_UIE));

  /* Enable capture channel. */
  timer_ic_enable(PWM_INPUT2_TIMER, PWM_INPUT2_CHANNEL_PERIOD);
  timer_ic_enable(PWM_INPUT2_TIMER, PWM_INPUT2_CHANNEL_DUTY);
#endif

}


#if USE_PWM_INPUT_TIM1

#if defined(STM32F1)
void tim1_up_isr(void)
{
#elif defined(STM32F4)
void tim1_up_tim10_isr(void) {
#endif
  if ((TIM1_SR & TIM_SR_UIF) != 0) {
    timer_clear_flag(TIM1, TIM_SR_UIF);
    // FIXME clear overflow interrupt but what else ?
  }
}

void tim1_cc_isr(void) {
  if ((TIM1_SR & TIM1_CC_IF_PERIOD) != 0) {
    timer_clear_flag(TIM1, TIM1_CC_IF_PERIOD);
    pwm_input_period_tics[TIM1_PWM_INPUT_IDX] = TIM1_CCR_PERIOD;
    pwm_input_period_valid[TIM1_PWM_INPUT_IDX] = true;
  }
  if ((TIM1_SR & TIM1_CC_IF_DUTY) != 0) {
    timer_clear_flag(TIM1, TIM1_CC_IF_DUTY);
    pwm_input_duty_tics[TIM1_PWM_INPUT_IDX] = TIM1_CCR_DUTY;
    pwm_input_duty_valid[TIM1_PWM_INPUT_IDX] = true;
  }
}

#endif

#if USE_PWM_INPUT_TIM2

void tim2_isr(void) {
  if ((TIM2_SR & TIM2_CC_IF_PERIOD) != 0) {
    timer_clear_flag(TIM2, TIM2_CC_IF_PERIOD);
    pwm_input_period_tics[TIM2_PWM_INPUT_IDX] = TIM2_CCR_PERIOD;
    pwm_input_period_valid[TIM2_PWM_INPUT_IDX] = true;
  }
  if ((TIM2_SR & TIM2_CC_IF_DUTY) != 0) {
    timer_clear_flag(TIM2, TIM2_CC_IF_DUTY);
    pwm_input_duty_tics[TIM2_PWM_INPUT_IDX] = TIM2_CCR_DUTY;
    pwm_input_duty_valid[TIM2_PWM_INPUT_IDX] = true;
  }
  if ((TIM2_SR & TIM_SR_UIF) != 0) {
    timer_clear_flag(TIM2, TIM_SR_UIF);
    // FIXME clear overflow interrupt but what else ?
  }
}

#endif

#if USE_PWM_INPUT_TIM3

void tim3_isr(void) {
  if ((TIM3_SR & TIM3_CC_IF_PERIOD) != 0) {
    timer_clear_flag(TIM3, TIM3_CC_IF_PERIOD);
    pwm_input_period_tics[TIM3_PWM_INPUT_IDX] = TIM3_CCR_PERIOD;
    pwm_input_period_valid[TIM3_PWM_INPUT_IDX] = true;
  }
  if ((TIM3_SR & TIM3_CC_IF_DUTY) != 0) {
    timer_clear_flag(TIM3, TIM3_CC_IF_DUTY);
    pwm_input_duty_tics[TIM3_PWM_INPUT_IDX] = TIM3_CCR_DUTY;
    pwm_input_duty_valid[TIM3_PWM_INPUT_IDX] = true;
  }
  if ((TIM3_SR & TIM_SR_UIF) != 0) {
    timer_clear_flag(TIM3, TIM_SR_UIF);
    // FIXME clear overflow interrupt but what else ?
  }
}

#endif

#if USE_PWM_INPUT_TIM4

void tim4_isr(void) {
  if ((TIM4_SR & TIM4_CC_IF_PERIOD) != 0) {
    timer_clear_flag(TIM4, TIM4_CC_IF_PERIOD);
    pwm_input_period_tics[TIM4_PWM_INPUT_IDX] = TIM4_CCR_PERIOD;
    pwm_input_period_valid[TIM4_PWM_INPUT_IDX] = true;
  }
  if ((TIM4_SR & TIM4_CC_IF_DUTY) != 0) {
    timer_clear_flag(TIM4, TIM4_CC_IF_DUTY);
    pwm_input_duty_tics[TIM4_PWM_INPUT_IDX] = TIM4_CCR_DUTY;
    pwm_input_duty_valid[TIM4_PWM_INPUT_IDX] = true;
  }
  if ((TIM4_SR & TIM_SR_UIF) != 0) {
    timer_clear_flag(TIM4, TIM_SR_UIF);
    // FIXME clear overflow interrupt but what else ?
  }
}

#endif

#if USE_PWM_INPUT_TIM5

void tim5_isr(void) {
  if ((TIM5_SR & TIM5_CC_IF_PERIOD) != 0) {
    timer_clear_flag(TIM5, TIM5_CC_IF_PERIOD);
    pwm_input_period_tics[TIM5_PWM_INPUT_IDX] = TIM5_CCR_PERIOD;
    pwm_input_period_valid[TIM5_PWM_INPUT_IDX] = true;
  }
  if ((TIM5_SR & TIM5_CC_IF_DUTY) != 0) {
    timer_clear_flag(TIM5, TIM5_CC_IF_DUTY);
    pwm_input_duty_tics[TIM5_PWM_INPUT_IDX] = TIM5_CCR_DUTY;
    pwm_input_duty_valid[TIM5_PWM_INPUT_IDX] = true;
  }
  if ((TIM5_SR & TIM_SR_UIF) != 0) {
    timer_clear_flag(TIM5, TIM_SR_UIF);
    // FIXME clear overflow interrupt but what else ?
  }
}

#endif

#if USE_PWM_INPUT_TIM8

#if defined(STM32F1)
void tim8_up_isr(void)
{
#elif defined(STM32F4)
void tim8_up_tim13_isr(void) {
#endif
  if ((TIM8_SR & TIM_SR_UIF) != 0) {
    timer_clear_flag(TIM8, TIM_SR_UIF);
    // FIXME clear overflow interrupt but what else ?
  }
}

void tim8_cc_isr(void) {
  if ((TIM8_SR & TIM8_CC_IF_PERIOD) != 0) {
    timer_clear_flag(TIM8, TIM8_CC_IF_PERIOD);
    pwm_input_period_tics[TIM8_PWM_INPUT_IDX] = TIM8_CCR_PERIOD;
    pwm_input_period_valid[TIM8_PWM_INPUT_IDX] = true;
  }
  if ((TIM8_SR & TIM8_CC_IF_DUTY) != 0) {
    timer_clear_flag(TIM8, TIM8_CC_IF_DUTY);
    pwm_input_duty_tics[TIM8_PWM_INPUT_IDX] = TIM8_CCR_DUTY;
    pwm_input_duty_valid[TIM8_PWM_INPUT_IDX] = true;
  }
}

#endif

#if USE_PWM_INPUT_TIM9

// TIM1 break interrupt (which we don't care here) and TIM9 global interrupt
void tim1_brk_tim9_isr(void) {
  if ((TIM9_SR & TIM9_CC_IF_PERIOD) != 0) {
    timer_clear_flag(TIM9, TIM9_CC_IF_PERIOD);
    pwm_input_period_tics[TIM9_PWM_INPUT_IDX] = TIM9_CCR_PERIOD;
    pwm_input_period_valid[TIM9_PWM_INPUT_IDX] = true;
  }
  if ((TIM9_SR & TIM9_CC_IF_DUTY) != 0) {
    timer_clear_flag(TIM9, TIM9_CC_IF_DUTY);
    pwm_input_duty_tics[TIM9_PWM_INPUT_IDX] = TIM9_CCR_DUTY;
    pwm_input_duty_valid[TIM9_PWM_INPUT_IDX] = true;
  }
  if ((TIM9_SR & TIM_SR_UIF) != 0) {
    timer_clear_flag(TIM9, TIM_SR_UIF);
    // FIXME clear overflow interrupt but what else ?
  }
}

#endif

