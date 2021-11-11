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

/** @file arch/stm32/modules/actuators/actuators_dualpwm_arch.c
 *  STM32 dual PWM servos handling.
 */

//VALID TIMERS IS TIM5 ON THE LISA/M

#include "modules/actuators/actuators_shared_arch.h"
#include "modules/actuators/actuators_dualpwm_arch.h"
#include "modules/actuators/actuators_dualpwm.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "mcu_periph/gpio_arch.h"

uint32_t ratio_4ms, ratio_16ms;


uint32_t actuators_dualpwm_values[ACTUATORS_DUALPWM_NB];

/** PWM arch init called by generic pwm driver
 */
void actuators_dualpwm_arch_init(void)
{

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
#ifdef DUAL_PWM_SERVO_5
  gpio_setup_pin_af(DUAL_PWM_SERVO_5_GPIO, DUAL_PWM_SERVO_5_PIN, DUAL_PWM_SERVO_5_AF, TRUE);
#endif
#ifdef DUAL_PWM_SERVO_6
  gpio_setup_pin_af(DUAL_PWM_SERVO_6_GPIO, DUAL_PWM_SERVO_6_PIN, DUAL_PWM_SERVO_6_AF, TRUE);
#endif

#if DUAL_PWM_USE_TIM5
  set_servo_timer(TIM5, TIM5_SERVO_HZ, PWM_TIM5_CHAN_MASK);

  nvic_set_priority(NVIC_TIM5_IRQ, 2);
  nvic_enable_irq(NVIC_TIM5_IRQ);
  timer_enable_irq(TIM5, TIM_DIER_CC1IE);
#endif

  //calculation the values to put into the timer registers to generate pulses every 4ms and 16ms.
  ratio_4ms = (ONE_MHZ_CLK / 250) - 1;
  ratio_16ms = (ONE_MHZ_CLK / 62.5) - 1;

}


/** Interuption called at the end of the timer. In our case alternatively very 4ms and 16ms (twice every 20ms)
 */
#if DUAL_PWM_USE_TIM5
void tim5_isr(void)
{

  dual_pwm_isr();
}
#endif




/** Fonction that clears the flag of interuption in order to reactivate the interuption
 */
void clear_timer_flag(void)
{

#if DUAL_PWM_USE_TIM5
  timer_clear_flag(TIM5, TIM_SR_CC1IF);
#endif
}


void set_dual_pwm_timer_s_period(uint32_t period)
{

#if DUAL_PWM_USE_TIM5
  timer_set_period(TIM5, period);
#endif
}


void set_dual_pwm_timer_s_oc(uint32_t oc_value, uint32_t oc_value2)
{

#if DUAL_PWM_USE_TIM5
  timer_set_oc_value(DUAL_PWM_SERVO_5_TIMER, DUAL_PWM_SERVO_5_OC, oc_value);
  timer_set_oc_value(DUAL_PWM_SERVO_6_TIMER, DUAL_PWM_SERVO_6_OC, oc_value2);
#endif
}




void dual_pwm_isr(void)
{

  static int num_pulse = 0;  //status of the timer. Are we controling the first or the second servo

  clear_timer_flag();

  if (num_pulse == 1) {

    set_dual_pwm_timer_s_period(ratio_16ms);
    set_dual_pwm_timer_s_oc(actuators_dualpwm_values[DUAL_PWM_SERVO_5_P1],actuators_dualpwm_values[DUAL_PWM_SERVO_5_P2]);

    num_pulse = 0;
  } else {

    set_dual_pwm_timer_s_period(ratio_4ms);
    set_dual_pwm_timer_s_oc(actuators_dualpwm_values[DUAL_PWM_SERVO_6_P1],actuators_dualpwm_values[DUAL_PWM_SERVO_6_P2]);

    num_pulse = 1;
  }
}


/** Set pulse widths from actuator values, assumed to be in us
 */
void actuators_dualpwm_commit(void)
{

  //we don't need to commit the values into this function as far as it's done in the interuption
  //(wich is called every 4ms and 16ms alternatively (twice every 20ms))

}
