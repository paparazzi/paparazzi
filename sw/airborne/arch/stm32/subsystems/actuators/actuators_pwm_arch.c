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

//VALID TIMERS ARE TIM1,2,3,4,5,8,9,12

#include "subsystems/actuators/actuators_shared_arch.h"
#include "subsystems/actuators/actuators_pwm_arch.h"
#include "subsystems/actuators/actuators_pwm.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>


int32_t actuators_pwm_values[ACTUATORS_PWM_NB];


/** PWM arch init called by generic pwm driver
 */
void actuators_pwm_arch_init(void) {

  /*-----------------------------------
   * Configure timer peripheral clocks
   *-----------------------------------*/
#if PWM_USE_TIM1
  rcc_periph_clock_enable(RCC_TIM1);
#endif
#if PWM_USE_TIM2
  rcc_periph_clock_enable(RCC_TIM2);
#endif
#if PWM_USE_TIM3
  rcc_periph_clock_enable(RCC_TIM3);
#endif
#if PWM_USE_TIM4
  rcc_periph_clock_enable(RCC_TIM4);
#endif
#if PWM_USE_TIM5
  rcc_periph_clock_enable(RCC_TIM5);
#endif
#if PWM_USE_TIM8
  rcc_periph_clock_enable(RCC_TIM8);
#endif
#if PWM_USE_TIM9
  rcc_periph_clock_enable(RCC_TIM9);
#endif
#if PWM_USE_TIM12
  rcc_periph_clock_enable(RCC_TIM12);
#endif

  /*----------------
   * Configure GPIO
   *----------------*/
#if defined(STM32F1)
  /* TIM3 GPIO for PWM1..4 */
  AFIO_MAPR |= AFIO_MAPR_TIM3_REMAP_FULL_REMAP;
#endif

#ifdef PWM_SERVO_0
  set_servo_gpio(PWM_SERVO_0_GPIO, PWM_SERVO_0_PIN, PWM_SERVO_0_AF, PWM_SERVO_0_RCC);
#endif
#ifdef PWM_SERVO_1
  set_servo_gpio(PWM_SERVO_1_GPIO, PWM_SERVO_1_PIN, PWM_SERVO_1_AF, PWM_SERVO_1_RCC);
#endif
#ifdef PWM_SERVO_2
  set_servo_gpio(PWM_SERVO_2_GPIO, PWM_SERVO_2_PIN, PWM_SERVO_2_AF, PWM_SERVO_2_RCC);
#endif
#ifdef PWM_SERVO_3
  set_servo_gpio(PWM_SERVO_3_GPIO, PWM_SERVO_3_PIN, PWM_SERVO_3_AF, PWM_SERVO_3_RCC);
#endif
#ifdef PWM_SERVO_4
  set_servo_gpio(PWM_SERVO_4_GPIO, PWM_SERVO_4_PIN, PWM_SERVO_4_AF, PWM_SERVO_4_RCC);
#endif
#ifdef PWM_SERVO_5
  set_servo_gpio(PWM_SERVO_5_GPIO, PWM_SERVO_5_PIN, PWM_SERVO_5_AF, PWM_SERVO_5_RCC);
#endif
#ifdef PWM_SERVO_6
  set_servo_gpio(PWM_SERVO_6_GPIO, PWM_SERVO_6_PIN, PWM_SERVO_6_AF, PWM_SERVO_6_RCC);
#endif
#ifdef PWM_SERVO_7
  set_servo_gpio(PWM_SERVO_7_GPIO, PWM_SERVO_7_PIN, PWM_SERVO_7_AF, PWM_SERVO_7_RCC);
#endif
#ifdef PWM_SERVO_8
  set_servo_gpio(PWM_SERVO_8_GPIO, PWM_SERVO_8_PIN, PWM_SERVO_8_AF, PWM_SERVO_8_RCC);
#endif
#ifdef PWM_SERVO_9
  set_servo_gpio(PWM_SERVO_9_GPIO, PWM_SERVO_9_PIN, PWM_SERVO_9_AF, PWM_SERVO_9_RCC);
#endif
#ifdef PWM_SERVO_10
  set_servo_gpio(PWM_SERVO_10_GPIO, PWM_SERVO_10_PIN, PWM_SERVO_10_AF, PWM_SERVO_10_RCC);
#endif
#ifdef PWM_SERVO_11
  set_servo_gpio(PWM_SERVO_11_GPIO, PWM_SERVO_11_PIN, PWM_SERVO_11_AF, PWM_SERVO_11_RCC);
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

#if PWM_USE_TIM8
  set_servo_timer(TIM8, TIM8_SERVO_HZ, PWM_TIM8_CHAN_MASK);
#endif

#if PWM_USE_TIM9
  set_servo_timer(TIM9, TIM9_SERVO_HZ, PWM_TIM9_CHAN_MASK);
#endif

#if PWM_USE_TIM12
  set_servo_timer(TIM12, TIM12_SERVO_HZ, PWM_TIM12_CHAN_MASK);
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
#ifdef PWM_SERVO_10
  timer_set_oc_value(PWM_SERVO_10_TIMER, PWM_SERVO_10_OC, actuators_pwm_values[PWM_SERVO_10]);
#endif
#ifdef PWM_SERVO_11
  timer_set_oc_value(PWM_SERVO_11_TIMER, PWM_SERVO_11_OC, actuators_pwm_values[PWM_SERVO_11]);
#endif

}
