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

/** @file arch/stm32/subsystems/actuators/actuators_shared_arch.c
 *  STM32 PWM and dualPWM servos shared functions.
 */

#include "subsystems/actuators/actuators_shared_arch.h"


/** Set GPIO configuration
 */
#if defined(STM32F4)
void set_servo_gpio(uint32_t gpioport, uint16_t pin, uint8_t af_num, enum rcc_periph_clken clken) {
  rcc_periph_clock_enable(clken);
  gpio_mode_setup(gpioport, GPIO_MODE_AF, GPIO_PUPD_NONE, pin);
  gpio_set_af(gpioport, af_num, pin);
}
#elif defined(STM32F1)
void set_servo_gpio(uint32_t gpioport, uint16_t pin, uint8_t none __attribute__((unused)), enum rcc_periph_clken clken) {
  rcc_periph_clock_enable(clken);
  rcc_periph_clock_enable(RCC_AFIO);
  gpio_set_mode(gpioport, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, pin);
}
#endif



/** Set PWM channel configuration
 */
void actuators_pwm_arch_channel_init(uint32_t timer_peripheral,
                                                   enum tim_oc_id oc_id) {

  timer_disable_oc_output(timer_peripheral, oc_id);
  //There is no such register in TIM9 and 12.
  if (timer_peripheral != TIM9 && timer_peripheral != TIM12)
    timer_disable_oc_clear(timer_peripheral, oc_id);
  timer_enable_oc_preload(timer_peripheral, oc_id);
  timer_set_oc_slow_mode(timer_peripheral, oc_id);
  timer_set_oc_mode(timer_peripheral, oc_id, TIM_OCM_PWM1);
  timer_set_oc_polarity_high(timer_peripheral, oc_id);
  timer_enable_oc_output(timer_peripheral, oc_id);
  // Used for TIM1 and TIM8, the function does nothing if other timer is specified.
  timer_enable_break_main_output(timer_peripheral);
}


/** Set Timer configuration
 */
void set_servo_timer(uint32_t timer, uint32_t period, uint8_t channels_mask) {
  timer_reset(timer);

  /* Timer global mode:
   * - No divider.
   * - Alignement edge.
   * - Direction up.
   */
  if ((timer == TIM9) || (timer == TIM12))
    //There are no EDGE and DIR settings in TIM9 and TIM12
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT, 0, 0);
  else
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);


  // TIM1, 8 and 9 use APB2 clock, all others APB1
  if (timer != TIM1 && timer != TIM8 && timer != TIM9) {
    timer_set_prescaler(timer, (TIMER_APB1_CLK / ONE_MHZ_CLK) - 1); // 1uS
  } else {
    // TIM9, 1 and 8 use APB2 clock
    timer_set_prescaler(timer, (TIMER_APB2_CLK / ONE_MHZ_CLK) - 1);
  }

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

