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

/** @file arch/stm32/subsystems/actuators/actuators_dualpwm_arch.c
 *  STM32 dual PWM servos handling.
 */

//VALID TIMERS IS TIM5 ON THE LISA/M

#include "subsystems/actuators/actuators_shared_arch.h"
#include "subsystems/actuators/actuators_dualpwm_arch.h"
#include "subsystems/actuators/actuators_dualpwm.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>


uint32_t ratio_4ms, ratio_16ms;


uint32_t actuators_dualpwm_values[ACTUATORS_PWM_NB];

/** PWM arch init called by generic pwm driver
 */
void actuators_dualpwm_arch_init(void) {

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

#ifdef DUAL_PWM_SERVO_0
  set_servo_gpio(DUAL_PWM_SERVO_0_GPIO, DUAL_PWM_SERVO_0_PIN, DUAL_PWM_SERVO_0_AF, DUAL_PWM_SERVO_0_RCC);
#endif
#ifdef DUAL_PWM_SERVO_1
  set_servo_gpio(DUAL_PWM_SERVO_1_GPIO, DUAL_PWM_SERVO_1_PIN, DUAL_PWM_SERVO_1_AF, DUAL_PWM_SERVO_1_RCC);
#endif
#ifdef DUAL_PWM_SERVO_2
  set_servo_gpio(DUAL_PWM_SERVO_2_GPIO, DUAL_PWM_SERVO_2_PIN, DUAL_PWM_SERVO_2_AF, DUAL_PWM_SERVO_2_RCC);
#endif
#ifdef DUAL_PWM_SERVO_3
  set_servo_gpio(DUAL_PWM_SERVO_3_GPIO, DUAL_PWM_SERVO_3_PIN, DUAL_PWM_SERVO_3_AF, DUAL_PWM_SERVO_3_RCC);
#endif
#ifdef DUAL_PWM_SERVO_4
  set_servo_gpio(DUAL_PWM_SERVO_4_GPIO, DUAL_PWM_SERVO_4_PIN, DUAL_PWM_SERVO_4_AF, DUAL_PWM_SERVO_4_RCC);
#endif
#ifdef DUAL_PWM_SERVO_5
  set_servo_gpio(DUAL_PWM_SERVO_5_GPIO, DUAL_PWM_SERVO_5_PIN, DUAL_PWM_SERVO_5_AF, DUAL_PWM_SERVO_5_RCC);
#endif
#ifdef DUAL_PWM_SERVO_6
  set_servo_gpio(DUAL_PWM_SERVO_6_GPIO, DUAL_PWM_SERVO_6_PIN, DUAL_PWM_SERVO_6_AF, DUAL_PWM_SERVO_6_RCC);
#endif
#ifdef DUAL_PWM_SERVO_7
  set_servo_gpio(DUAL_PWM_SERVO_7_GPIO, DUAL_PWM_SERVO_7_PIN, DUAL_PWM_SERVO_7_AF, DUAL_PWM_SERVO_7_RCC);
#endif
#ifdef DUAL_PWM_SERVO_8
  set_servo_gpio(DUAL_PWM_SERVO_8_GPIO, DUAL_PWM_SERVO_8_PIN, DUAL_PWM_SERVO_8_AF, DUAL_PWM_SERVO_8_RCC);
#endif
#ifdef DUAL_PWM_SERVO_9
  set_servo_gpio(DUAL_PWM_SERVO_9_GPIO, DUAL_PWM_SERVO_9_PIN, DUAL_PWM_SERVO_9_AF, DUAL_PWM_SERVO_9_RCC);
#endif
#ifdef DUAL_PWM_SERVO_10
  set_servo_gpio(DUAL_PWM_SERVO_10_GPIO, DUAL_PWM_SERVO_10_PIN, DUAL_PWM_SERVO_10_AF, DUAL_PWM_SERVO_10_RCC);
#endif
#ifdef DUAL_PWM_SERVO_11
  set_servo_gpio(DUAL_PWM_SERVO_11_GPIO, DUAL_PWM_SERVO_11_PIN, DUAL_PWM_SERVO_11_AF, DUAL_PWM_SERVO_11_RCC);
#endif





#if DUAL_PWM_USE_TIM1
  set_servo_timer(TIM1, TIM1_SERVO_HZ, PWM_TIM1_CHAN_MASK);

  nvic_set_priority(NVIC_TIM1_IRQ, 2);
  nvic_enable_irq(NVIC_TIM1_IRQ);
  timer_enable_irq(TIM1, TIM_DIER_CC1IE);
#endif

#if DUAL_PWM_USE_TIM2
  set_servo_timer(TIM2, TIM2_SERVO_HZ, PWM_TIM2_CHAN_MASK);

  nvic_set_priority(NVIC_TIM2_IRQ, 2);
  nvic_enable_irq(NVIC_TIM2_IRQ);
  timer_enable_irq(TIM2, TIM_DIER_CC1IE);
#endif

#if DUAL_PWM_USE_TIM3
  set_servo_timer(TIM3, TIM3_SERVO_HZ, PWM_TIM3_CHAN_MASK);

  nvic_set_priority(NVIC_TIM3_IRQ, 2);
  nvic_enable_irq(NVIC_TIM3_IRQ);
  timer_enable_irq(TIM3, TIM_DIER_CC1IE);
#endif

#if DUAL_PWM_USE_TIM4
  set_servo_timer(TIM4, TIM4_SERVO_HZ, PWM_TIM4_CHAN_MASK);

  nvic_set_priority(NVIC_TIM4_IRQ, 2);
  nvic_enable_irq(NVIC_TIM4_IRQ);
  timer_enable_irq(TIM4, TIM_DIER_CC1IE);
#endif

#if DUAL_PWM_USE_TIM5
  set_servo_timer(TIM5, TIM5_SERVO_HZ, PWM_TIM5_CHAN_MASK);

  nvic_set_priority(NVIC_TIM5_IRQ, 2);
  nvic_enable_irq(NVIC_TIM5_IRQ);
  timer_enable_irq(TIM5, TIM_DIER_CC1IE);
#endif

#if DUAL_PWM_USE_TIM8
  set_servo_timer(TIM8, TIM8_SERVO_HZ, PWM_TIM8_CHAN_MASK);

  nvic_set_priority(NVIC_TIM8_IRQ, 2);
  nvic_enable_irq(NVIC_TIM8_IRQ);
  timer_enable_irq(TIM8, TIM_DIER_CC1IE);
#endif

#if DUAL_PWM_USE_TIM9
  set_servo_timer(TIM9, TIM9_SERVO_HZ, PWM_TIM9_CHAN_MASK);

  nvic_set_priority(NVIC_TIM9_IRQ, 2);
  nvic_enable_irq(NVIC_TIM9_IRQ);
  timer_enable_irq(TIM9, TIM_DIER_CC1IE);
#endif

#if DUAL_PWM_USE_TIM12
  set_servo_timer(TIM12, TIM12_SERVO_HZ, PWM_TIM12_CHAN_MASK);

  nvic_set_priority(NVIC_TIM12_IRQ, 2);
  nvic_enable_irq(NVIC_TIM12_IRQ);
  timer_enable_irq(TIM12, TIM_DIER_CC1IE);
#endif

  //calculation the values to put into the timer registers to generate pulses every 4ms and 16ms.
  ratio_4ms = (ONE_MHZ_CLK / 250)-1;
  ratio_16ms = (ONE_MHZ_CLK / 62.5)-1;

}


/** Interuption called at the end of the timer. In our case alternatively very 4ms and 16ms (twice every 20ms)
 */
#if DUAL_PWM_USE_TIM1
void tim1_isr(void){

  dual_pwm_isr();
}
#endif

#if DUAL_PWM_USE_TIM2
void tim2_isr(void){

  dual_pwm_isr();
}
#endif

#if DUAL_PWM_USE_TIM3
void tim3_isr(void){

  dual_pwm_isr();
}
#endif

#if DUAL_PWM_USE_TIM4
void tim4_isr(void){

  dual_pwm_isr();
}
#endif

#if DUAL_PWM_USE_TIM5
void tim5_isr(void){

  dual_pwm_isr();
}
#endif


#if DUAL_PWM_USE_TIM8
void tim8_isr(void){

  dual_pwm_isr();
}
#endif

#if DUAL_PWM_USE_TIM9
void tim9_isr(void){

  dual_pwm_isr();
}
#endif

#if DUAL_PWM_USE_TIM12
void tim12_isr(void){

  dual_pwm_isr();
}
#endif





/** Fonction that clears the flag of interuption in order to reactivate the interuption
 */
void clear_timer_flag(void){

#if DUAL_PWM_USE_TIM1
  timer_clear_flag(TIM1, TIM_SR_CC1IF);
#endif

#if DUAL_PWM_USE_TIM2
  timer_clear_flag(TIM2, TIM_SR_CC1IF);
#endif

#if DUAL_PWM_USE_TIM3
  timer_clear_flag(TIM3, TIM_SR_CC1IF);
#endif

#if DUAL_PWM_USE_TIM4
  timer_clear_flag(TIM4, TIM_SR_CC1IF);
#endif

#if DUAL_PWM_USE_TIM5
  timer_clear_flag(TIM5, TIM_SR_CC1IF);
#endif

#if DUAL_PWM_USE_TIM8
  timer_clear_flag(TIM8, TIM_SR_CC1IF);
#endif

#if DUAL_PWM_USE_TIM9
  timer_clear_flag(TIM9, TIM_SR_CC1IF);
#endif

#if DUAL_PWM_USE_TIM12
  timer_clear_flag(TIM12, TIM_SR_CC1IF);
#endif
}


void set_dual_pwm_timer_s_period(uint32_t period){

#if DUAL_PWM_USE_TIM1
  timer_set_period(TIM1, period);
#endif

#if DUAL_PWM_USE_TIM2
  timer_set_period(TIM2, period);
#endif

#if DUAL_PWM_USE_TIM3
  timer_set_period(TIM3, period);
#endif

#if DUAL_PWM_USE_TIM4
  timer_set_period(TIM4, period);
#endif

#if DUAL_PWM_USE_TIM5
  timer_set_period(TIM5, period);
#endif

#if DUAL_PWM_USE_TIM8
  timer_set_period(TIM8, period);
#endif

#if DUAL_PWM_USE_TIM9
  timer_set_period(TIM9, period);
#endif

#if DUAL_PWM_USE_TIM12
  timer_set_period(TIM12, period);
#endif
}


void set_dual_pwm_timer_s_oc(uint32_t oc_value){

#if DUAL_PWM_USE_TIM1
  timer_set_oc_value(DUAL_PWM_SERVO_1_TIMER, DUAL_PWM_SERVO_1_OC, oc_value);
#endif

#if DUAL_PWM_USE_TIM2
  timer_set_oc_value(DUAL_PWM_SERVO_2_TIMER, DUAL_PWM_SERVO_2_OC, oc_value);
#endif

#if DUAL_PWM_USE_TIM3
  timer_set_oc_value(DUAL_PWM_SERVO_3_TIMER, DUAL_PWM_SERVO_3_OC, oc_value);
#endif

#if DUAL_PWM_USE_TIM4
  timer_set_oc_value(DUAL_PWM_SERVO_4_TIMER, DUAL_PWM_SERVO_4_OC, oc_value);
#endif

#if DUAL_PWM_USE_TIM5
  timer_set_oc_value(DUAL_PWM_SERVO_5_TIMER, DUAL_PWM_SERVO_5_OC, oc_value);
#endif

#if DUAL_PWM_USE_TIM8
  timer_set_oc_value(DUAL_PWM_SERVO_8_TIMER, DUAL_PWM_SERVO_8_OC, oc_value);
#endif

#if DUAL_PWM_USE_TIM9
  timer_set_oc_value(DUAL_PWM_SERVO_9_TIMER, DUAL_PWM_SERVO_9_OC, oc_value);
#endif

#if DUAL_PWM_USE_TIM12
  timer_set_oc_value(DUAL_PWM_SERVO_12_TIMER, DUAL_PWM_SERVO_12_OC, oc_value);
#endif
}




void dual_pwm_isr(void){

  static int num_pulse = 0;  //status of the timer. Are we controling the first or the second servo

  clear_timer_flag();

  if(num_pulse == 1){

    set_dual_pwm_timer_s_period(ratio_16ms);
    set_dual_pwm_timer_s_oc(actuators_dualpwm_values[FIRST_DUAL_PWM_SERVO]);

    num_pulse = 0;
  }else{

    set_dual_pwm_timer_s_period(ratio_4ms);
    set_dual_pwm_timer_s_oc(actuators_dualpwm_values[SECOND_DUAL_PWM_SERVO]);

    num_pulse = 1;
  }
}


/** Set pulse widths from actuator values, assumed to be in us
 */
void actuators_dualpwm_commit(void) {

  //we don't need to commit the values into this function as far as it's done in the interuption
  //(wich is called every 4ms and 16ms alternatively (twice every 20ms))

}
