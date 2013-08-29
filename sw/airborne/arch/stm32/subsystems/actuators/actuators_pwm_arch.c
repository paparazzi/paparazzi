/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file arch/stm32/subsystems/actuators/actuators_pwm_arch.c
 *  STM32 PWM servos handling.
 */

#include "subsystems/actuators/actuators_pwm_arch.h"
#include "subsystems/actuators/actuators_pwm.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

#if defined(STM32F1)
//#define PCLK 72000000
#define PCLK AHB_CLK
#elif defined(STM32F4)
//#define PCLK 84000000
#define PCLK AHB_CLK/2
#endif

#define ONE_MHZ_CLK 1000000
/** Default servo update rate in Hz */
#ifndef SERVO_HZ
#define SERVO_HZ 40
#endif

// Update rate can be adapted for each timer
#ifndef TIM1_SERVO_HZ
#define TIM1_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM2_SERVO_HZ
#define TIM2_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM3_SERVO_HZ
#define TIM3_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM4_SERVO_HZ
#define TIM4_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM5_SERVO_HZ
#define TIM5_SERVO_HZ SERVO_HZ
#endif

/** Set PWM channel configuration
 */
static inline void actuators_pwm_arch_channel_init(uint32_t timer_peripheral,
						enum tim_oc_id oc_id) {

  timer_disable_oc_output(timer_peripheral, oc_id);
  timer_disable_oc_clear(timer_peripheral, oc_id);
  timer_enable_oc_preload(timer_peripheral, oc_id);
  timer_set_oc_slow_mode(timer_peripheral, oc_id);
  timer_set_oc_mode(timer_peripheral, oc_id, TIM_OCM_PWM1);
  timer_set_oc_polarity_high(timer_peripheral, oc_id);
  timer_enable_oc_output(timer_peripheral, oc_id);
}

/** Set GPIO configuration
 */
#if defined(STM32F4)
static inline void set_servo_gpio(uint32_t gpioport, uint16_t pin, uint8_t af_num, uint32_t en) {
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, en);
  gpio_mode_setup(gpioport, GPIO_MODE_AF, GPIO_PUPD_NONE, pin);
  gpio_set_af(gpioport, af_num, pin);
}
#elif defined(STM32F1)
static inline void set_servo_gpio(uint32_t gpioport, uint16_t pin, uint8_t none, uint32_t en) {
  rcc_peripheral_enable_clock(&RCC_APB2ENR, en | RCC_APB2ENR_AFIOEN);
  gpio_set_mode(gpioport, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, pin);
}
#endif

/** Set Timer configuration
 */
static inline void set_servo_timer(uint32_t timer, uint32_t period, uint8_t channels_mask) {
  timer_reset(timer);

  /* Timer global mode:
   * - No divider.
   * - Alignement edge.
   * - Direction up.
   */
  timer_set_mode(timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  timer_set_prescaler(timer, (PCLK / ONE_MHZ_CLK) - 1); // 1uS

  timer_disable_preload(timer);

  timer_continuous_mode(timer);

  timer_set_period(timer, (ONE_MHZ_CLK / period) - 1);

  /* Disable outputs and configure channel if needed. */
  if (bit_is_set(channels_mask, 0)) {
    actuators_pwm_arch_channel_init(timer, TIM_OC1);
  }
  if (bit_is_set(channels_mask, 1)) {
    actuators_pwm_arch_channel_init(timer, TIM_OC2);
  }
  if (bit_is_set(channels_mask, 2)) {
    actuators_pwm_arch_channel_init(timer, TIM_OC3);
  }
  if (bit_is_set(channels_mask, 3)) {
    actuators_pwm_arch_channel_init(timer, TIM_OC4);
  }

  /*
   * Set initial output compare values.
   * Note: Maybe we should preload the compare registers with some sensible
   * values before we enable the timer?
   */
  //timer_set_oc_value(timer, TIM_OC1, 1000);
  //timer_set_oc_value(timer, TIM_OC2, 1000);
  //timer_set_oc_value(timer, TIM_OC3, 1000);
  //timer_set_oc_value(timer, TIM_OC4, 1000);

  /* -- Enable timer -- */
  /*
   * ARR reload enable.
   * Note: In our case it does not matter much if we do preload or not. As it
   * is unlikely we will want to change the frequency of the timer during
   * runtime anyways.
   */
  timer_enable_preload(timer);

  /* Counter enable. */
  timer_enable_counter(timer);
}

/** PWM arch init called by generic pwm driver
 */
void actuators_pwm_arch_init(void) {

  /*-----------------------------------
   * Configure timer peripheral clocks
   *-----------------------------------*/
#if PWM_USE_TIM1
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);
#endif
#if PWM_USE_TIM2
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
#endif
#if PWM_USE_TIM3
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);
#endif
#if PWM_USE_TIM4
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);
#endif
#if PWM_USE_TIM5
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM5EN);
#endif

  /*----------------
   * Configure GPIO
   *----------------*/
#if defined(STM32F1)
  /* TIM3 GPIO for PWM1..4 */
  AFIO_MAPR |= AFIO_MAPR_TIM3_REMAP_FULL_REMAP;
#endif

#ifdef PWM_SERVO_0
  set_servo_gpio(PWM_SERVO_0_GPIO, PWM_SERVO_0_PIN, PWM_SERVO_0_AF, PWM_SERVO_0_RCC_IOP);
#endif
#ifdef PWM_SERVO_1
  set_servo_gpio(PWM_SERVO_1_GPIO, PWM_SERVO_1_PIN, PWM_SERVO_1_AF, PWM_SERVO_1_RCC_IOP);
#endif
#ifdef PWM_SERVO_2
  set_servo_gpio(PWM_SERVO_2_GPIO, PWM_SERVO_2_PIN, PWM_SERVO_2_AF, PWM_SERVO_2_RCC_IOP);
#endif
#ifdef PWM_SERVO_3
  set_servo_gpio(PWM_SERVO_3_GPIO, PWM_SERVO_3_PIN, PWM_SERVO_3_AF, PWM_SERVO_3_RCC_IOP);
#endif
#ifdef PWM_SERVO_4
  set_servo_gpio(PWM_SERVO_4_GPIO, PWM_SERVO_4_PIN, PWM_SERVO_4_AF, PWM_SERVO_4_RCC_IOP);
#endif
#ifdef PWM_SERVO_5
  set_servo_gpio(PWM_SERVO_5_GPIO, PWM_SERVO_5_PIN, PWM_SERVO_5_AF, PWM_SERVO_5_RCC_IOP);
#endif
#ifdef PWM_SERVO_6
  set_servo_gpio(PWM_SERVO_6_GPIO, PWM_SERVO_6_PIN, PWM_SERVO_6_AF, PWM_SERVO_6_RCC_IOP);
#endif
#ifdef PWM_SERVO_7
  set_servo_gpio(PWM_SERVO_7_GPIO, PWM_SERVO_7_PIN, PWM_SERVO_7_AF, PWM_SERVO_7_RCC_IOP);
#endif
#ifdef PWM_SERVO_8
  set_servo_gpio(PWM_SERVO_8_GPIO, PWM_SERVO_8_PIN, PWM_SERVO_8_AF, PWM_SERVO_8_RCC_IOP);
#endif
#ifdef PWM_SERVO_9
  set_servo_gpio(PWM_SERVO_9_GPIO, PWM_SERVO_9_PIN, PWM_SERVO_9_AF, PWM_SERVO_9_RCC_IOP);
#endif


#if PWM_USE_TIM1
  set_servo_timer(TIM1, TIM1_SERVO_HZ, PWM_TIM1_CHAN_MASK);
#endif

#if PWM_USE_TIM2
  set_servo_timer(TIM2, TIM2_SERVO_HZ, PWM_TIM2_CHAN_MASK);
#endif

#if PWM_USE_TIM3
  set_servo_timer(TIM3, TIM3_SERVO_HZ, PWM_TIM3_CHAN_MASK);
#endif

#if PWM_USE_TIM4
  set_servo_timer(TIM4, TIM4_SERVO_HZ, PWM_TIM4_CHAN_MASK);
#endif

#if PWM_USE_TIM5
  set_servo_timer(TIM5, TIM5_SERVO_HZ, PWM_TIM5_CHAN_MASK);
#endif

}

/** Set pulse widths from actuator values, assumed to be in us
 */
void actuators_pwm_commit(void) {
#ifdef PWM_SERVO_0
  timer_set_oc_value(PWM_SERVO_0_TIMER, PWM_SERVO_0_OC, actuators_pwm_values[PWM_SERVO_0]);
#endif
#ifdef PWM_SERVO_1
  timer_set_oc_value(PWM_SERVO_1_TIMER, PWM_SERVO_1_OC, actuators_pwm_values[PWM_SERVO_1]);
#endif
#ifdef PWM_SERVO_2
  timer_set_oc_value(PWM_SERVO_2_TIMER, PWM_SERVO_2_OC, actuators_pwm_values[PWM_SERVO_2]);
#endif
#ifdef PWM_SERVO_3
  timer_set_oc_value(PWM_SERVO_3_TIMER, PWM_SERVO_3_OC, actuators_pwm_values[PWM_SERVO_3]);
#endif
#ifdef PWM_SERVO_4
  timer_set_oc_value(PWM_SERVO_4_TIMER, PWM_SERVO_4_OC, actuators_pwm_values[PWM_SERVO_4]);
#endif
#ifdef PWM_SERVO_5
  timer_set_oc_value(PWM_SERVO_5_TIMER, PWM_SERVO_5_OC, actuators_pwm_values[PWM_SERVO_5]);
#endif
#ifdef PWM_SERVO_6
  timer_set_oc_value(PWM_SERVO_6_TIMER, PWM_SERVO_6_OC, actuators_pwm_values[PWM_SERVO_6]);
#endif
#ifdef PWM_SERVO_7
  timer_set_oc_value(PWM_SERVO_7_TIMER, PWM_SERVO_7_OC, actuators_pwm_values[PWM_SERVO_7]);
#endif
#ifdef PWM_SERVO_8
  timer_set_oc_value(PWM_SERVO_8_TIMER, PWM_SERVO_8_OC, actuators_pwm_values[PWM_SERVO_8]);
#endif
#ifdef PWM_SERVO_9
  timer_set_oc_value(PWM_SERVO_9_TIMER, PWM_SERVO_9_OC, actuators_pwm_values[PWM_SERVO_9]);
#endif

}
