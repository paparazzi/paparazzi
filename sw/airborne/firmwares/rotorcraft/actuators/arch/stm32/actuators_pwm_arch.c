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

/** @file arch/stm32/actuators_pwm_arch.c
 *  STM32 PWM servos handling
 */

#include "firmwares/rotorcraft/actuators/actuators_pwm.h"

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/timer.h>

#define PCLK 72000000
#define ONE_MHZ_CLK 1000000
#ifndef SERVO_HZ
#define SERVO_HZ 40
#endif

#define _TIM_OC_INIT(n) TIM_OC##n##Init
#define TIM_OC_INIT(n) _TIM_OC_INIT(n)

#define _TIM_OC_PRELOADCONFIG(n) TIM_OC##n##PreloadConfig
#define TIM_OC_PRELOADCONFIG(n) _TIM_OC_PRELOADCONFIG(n)

#define _TIM_SETCOMPARE(n) TIM_SetCompare##n
#define TIM_SETCOMPARE(n) _TIM_SETCOMPARE(n)

static inline void actuators_pwm_arch_channel_init(u32 timer_peripheral,
						enum tim_oc_id oc_id) {

  timer_disable_oc_clear(timer_peripheral, oc_id);
  timer_enable_oc_preload(timer_peripheral, oc_id);
  timer_set_oc_slow_mode(timer_peripheral, oc_id);
  timer_set_oc_mode(timer_peripheral, oc_id, TIM_OCM_PWM1);
  timer_set_oc_polarity_high(timer_peripheral, oc_id);
  timer_enable_oc_output(timer_peripheral, oc_id);
}

void actuators_pwm_arch_init(void) {

  /*-----------------------------------
   * Configure timer peripheral clocks
   *-----------------------------------*/
  /* TIM3, TIM4 and TIM5 clock enable */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);
#if REMAP_SERVOS_5AND6
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM5EN);
#else
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);
#endif
#if USE_SERVOS_7AND8
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);
#endif

  /*----------------
   * Configure GPIO
   *----------------*/
  /* GPIO A,B and C clock enable */
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN |
			  RCC_APB2ENR_IOPBEN |
			  RCC_APB2ENR_IOPCEN |
			  RCC_APB2ENR_AFIOEN);

  /* TIM3 GPIO for PWM1..4 */
  AFIO_MAPR |= AFIO_MAPR_TIM3_REMAP_FULL_REMAP;
  gpio_set_mode(GPIO_BANK_TIM3_FR,
	  GPIO_MODE_OUTPUT_50_MHZ,
	  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
	  GPIO_TIM3_FR_CH1 |
	  GPIO_TIM3_FR_CH2 |
	  GPIO_TIM3_FR_CH3 |
	  GPIO_TIM3_FR_CH4);

  /* TIM4 GPIO for PWM7..8 */
#if USE_SERVOS_7AND8
  gpio_set_mode(GPIO_BANK_TIM4,
	  GPIO_MODE_OUTPUT_50_MHZ,
	  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
	  GPIO_TIM4_CH1 |
	  GPIO_TIM4_CH2);
#endif

  /* TIM4/5 GPIO for PWM6..7 */
#if REMAP_SERVOS_5AND6
  gpio_set_mode(GPIO_BANK_TIM5,
	  GPIO_MODE_OUTPUT_50_MHZ,
	  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
	  GPIO_TIM5_CH1 |
	  GPIO_TIM5_CH2);
#else
  gpio_set_mode(GPIO_BANK_TIM4,
	  GPIO_MODE_OUTPUT_50_MHZ,
	  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
	  GPIO_TIM4_CH3 |
	  GPIO_TIM4_CH4);
#endif

  /*---------------
   * Timer 3 setup
   *---------------*/
  timer_reset(TIM3);

  /* Timer global mode:
   * - No divider.
   * - Alignement edge.
   * - Direction up.
   */
  timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT,
	  TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  timer_set_prescaler(TIM3, (PCLK / ONE_MHZ_CLK) - 1); // 1uS

  timer_disable_preload(TIM3);

  timer_continuous_mode(TIM3);

  timer_set_period(TIM3, (ONE_MHZ_CLK / SERVO_HZ) - 1);

  /* Disable outputs. */
  timer_disable_oc_output(TIM3, TIM_OC1);
  timer_disable_oc_output(TIM3, TIM_OC2);
  timer_disable_oc_output(TIM3, TIM_OC3);
  timer_disable_oc_output(TIM3, TIM_OC4);

  /* -- Channel configuration -- */
  actuators_pwm_arch_channel_init(TIM3, TIM_OC1);
  actuators_pwm_arch_channel_init(TIM3, TIM_OC2);
  actuators_pwm_arch_channel_init(TIM3, TIM_OC3);
  actuators_pwm_arch_channel_init(TIM3, TIM_OC4);

  /*
   * Set initial output compare values.
   * Note: Maybe we should preload the compare registers with some sensible
   * values before we enable the timer?
   */
  //timer_set_oc_value(TIM3, TIM_OC1, 1000);
  //timer_set_oc_value(TIM3, TIM_OC2, 1000);
  //timer_set_oc_value(TIM3, TIM_OC3, 1000);
  //timer_set_oc_value(TIM3, TIM_OC4, 1000);

  /* -- Enable timer -- */
  /*
   * ARR reload enable.
   * Note: In our case it does not matter much if we do preload or not. As it
   * is unlikely we will want to change the frequency of the timer during
   * runtime anyways.
   */
  timer_enable_preload(TIM3);

  /* Counter enable. */
  timer_enable_counter(TIM3);

#if (!REMAP_SERVOS_5AND6 || USE_SERVOS_7AND8)
#if !REMAP_SERVOS_5AND6
#pragma message "Not remapping servos 6 and 7 using PB8 and PB9 -> TIM4"
#endif
#if USE_SERVOS_7AND8
#pragma message "Enabeling sevros 7 and 8 on PB6, PB7 -> TIM4"
#endif
  /*---------------
   * Timer 4 setup
   *---------------*/
  timer_reset(TIM4);

  /* Timer global mode:
   * - No divider.
   * - Alignement edge.
   * - Direction up.
   */
  timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT,
	  TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  timer_set_prescaler(TIM4, (PCLK / ONE_MHZ_CLK) - 1); // 1uS

  timer_disable_preload(TIM4);

  timer_continuous_mode(TIM4);

#ifdef SERVO_HZ_SECONDARY
  timer_set_period(TIM4, (ONE_MHZ_CLK / SERVO_HZ_SECONDARY) - 1);
#else
  timer_set_period(TIM4, (ONE_MHZ_CLK / SERVO_HZ) - 1);
#endif

  /* Disable outputs. */
#if USE_SERVOS_7AND8
  timer_disable_oc_output(TIM4, TIM_OC1);
  timer_disable_oc_output(TIM4, TIM_OC2);
#endif
#if !REMAP_SERVOS_5AND6
  timer_disable_oc_output(TIM4, TIM_OC3);
  timer_disable_oc_output(TIM4, TIM_OC4);
#endif

  /* -- Channel configuration -- */
#if USE_SERVOS_7AND8
  actuators_pwm_arch_channel_init(TIM4, TIM_OC1);
  actuators_pwm_arch_channel_init(TIM4, TIM_OC2);
#endif
#if !REMAP_SERVOS_5AND6
  actuators_pwm_arch_channel_init(TIM4, TIM_OC3);
  actuators_pwm_arch_channel_init(TIM4, TIM_OC4);
#endif

  /*
   * Set initial output compare values.
   * Note: Maybe we should preload the compare registers with some sensible
   * values before we enable the timer?
   */
#if USE_SERVOS_7AND8
  //timer_set_oc_value(TIM4, TIM_OC1, 1000);
  //timer_set_oc_value(TIM4, TIM_OC2, 1000);
#endif
#if ! REMAP_SERVOS_5AND6
  //timer_set_oc_value(TIM4, TIM_OC3, 1000);
  //timer_set_oc_value(TIM4, TIM_OC4, 1000);
#endif

  /* -- Enable timer -- */
  /*
   * ARR reload enable.
   * Note: In our case it does not matter much if we do preload or not. As it
   * is unlikely we will want to change the frequency of the timer during
   * runtime anyways.
   */
  timer_enable_preload(TIM4);

  /* Counter enable. */
  timer_enable_counter(TIM4);

#endif

#if REMAP_SERVOS_5AND6
#pragma message "Remapping servo outputs 6 and 7 to PA0,PA1 -> TIM5"
  /*---------------
   * Timer 5 setup
   *---------------*/
  timer_reset(TIM5);

  /* Timer global mode:
   * - No divider.
   * - Alignement edge.
   * - Direction up.
   */
  timer_set_mode(TIM5, TIM_CR1_CKD_CK_INT,
	  TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  timer_set_prescaler(TIM5, (PCLK / ONE_MHZ_CLK) - 1); // 1uS

  timer_disable_preload(TIM5);

  timer_continuous_mode(TIM5);

#ifdef SERVO_HZ_SECONDARY
  timer_set_period(TIM5, (ONE_MHZ_CLK / SERVO_HZ_SECONDARY) - 1);
#else
  timer_set_period(TIM5, (ONE_MHZ_CLK / SERVO_HZ) - 1);
#endif

  /* Disable outputs. */
  timer_disable_oc_output(TIM5, TIM_OC1);
  timer_disable_oc_output(TIM5, TIM_OC2);

  /* -- Channel configuration -- */
  actuators_pwm_arch_channel_init(TIM5, TIM_OC1);
  actuators_pwm_arch_channel_init(TIM5, TIM_OC2);

  /*
   * Set the capture compare value for OC1.
   * Note: Maybe we should preload the compare registers with some sensible
   * values before we enable the timer?
   */
  //timer_set_oc_value(TIM5, TIM_OC1, 1000);
  //timer_set_oc_value(TIM5, TIM_OC2, 1000);

  /* -- Enable timer -- */
  /*
   * ARR reload enable.
   * Note: In our case it does not matter much if we do preload or not. As it
   * is unlikely we will want to change the frequency of the timer during
   * runtime anyways.
   */
  timer_enable_preload(TIM5);

  /* Counter enable. */
  timer_enable_counter(TIM5);

#endif

}

/* set pulse widths from actuator values, assumed to be in us */
void actuators_pwm_commit(void) {
  timer_set_oc_value(TIM3, TIM_OC1, actuators_pwm_values[0]);
  timer_set_oc_value(TIM3, TIM_OC2, actuators_pwm_values[1]);
  timer_set_oc_value(TIM3, TIM_OC3, actuators_pwm_values[2]);
  timer_set_oc_value(TIM3, TIM_OC4, actuators_pwm_values[3]);

#if USE_SERVOS_7AND8
  timer_set_oc_value(TIM4, TIM_OC1, actuators_pwm_values[6]);
  timer_set_oc_value(TIM4, TIM_OC2, actuators_pwm_values[7]);
#endif
#if REMAP_SERVOS_5AND6
  timer_set_oc_value(TIM5, TIM_OC1, actuators_pwm_values[4]);
  timer_set_oc_value(TIM5, TIM_OC2, actuators_pwm_values[5]);
#else
  timer_set_oc_value(TIM4, TIM_OC3, actuators_pwm_values[4]);
  timer_set_oc_value(TIM4, TIM_OC4, actuators_pwm_values[5]);
#endif

}
