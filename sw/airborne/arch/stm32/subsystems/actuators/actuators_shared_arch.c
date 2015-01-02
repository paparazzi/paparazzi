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

#include "arch/stm32/subsystems/actuators/actuators_shared_arch.h"

#include <libopencm3/stm32/timer.h>
// for timer_get_frequency
#include "arch/stm32/mcu_arch.h"


/** Set PWM channel configuration
 */
void actuators_pwm_arch_channel_init(uint32_t timer_peripheral,
                                     enum tim_oc_id oc_id)
{

  timer_disable_oc_output(timer_peripheral, oc_id);
  //There is no such register in TIM9 and 12.
  if (timer_peripheral != TIM9 && timer_peripheral != TIM12) {
    timer_disable_oc_clear(timer_peripheral, oc_id);
  }
  timer_enable_oc_preload(timer_peripheral, oc_id);
  timer_set_oc_slow_mode(timer_peripheral, oc_id);
  timer_set_oc_mode(timer_peripheral, oc_id, TIM_OCM_PWM1);
  timer_set_oc_polarity_high(timer_peripheral, oc_id);
  timer_enable_oc_output(timer_peripheral, oc_id);
  // Used for TIM1 and TIM8, the function does nothing if other timer is specified.
  timer_enable_break_main_output(timer_peripheral);
}


/** Set Timer configuration
 * @param[in] timer Timer register address base
 * @param[in] period period in us
 * @param[in] channels_mask output compare channels to enable
 */
void set_servo_timer(uint32_t timer, uint32_t period, uint8_t channels_mask)
{
  // WARNING, this reset is only implemented for TIM1-8 in libopencm3!!
  timer_reset(timer);

  /* Timer global mode:
   * - No divider.
   * - Alignement edge.
   * - Direction up.
   */
  if ((timer == TIM9) || (timer == TIM12))
    //There are no EDGE and DIR settings in TIM9 and TIM12
  {
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT, 0, 0);
  } else {
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  }


  // By default the PWM_BASE_FREQ is set to 1MHz thus the timer tick period is 1uS
  uint32_t timer_clk = timer_get_frequency(timer);
  timer_set_prescaler(timer, (timer_clk / PWM_BASE_FREQ) - 1);

  timer_disable_preload(timer);

  timer_continuous_mode(timer);

  timer_set_period(timer, (PWM_BASE_FREQ / period) - 1);

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

