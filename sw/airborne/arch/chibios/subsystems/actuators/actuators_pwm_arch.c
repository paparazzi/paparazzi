/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
 * @file arch/chibios/subsystems/actuators/actuators_pwm_arch.c
 * Interface from actuators to ChibiOS PWM driver
 *
 * PWM configuration files are defined in the board file,
 * so maximal architecture independence is ensured.
 */
#include "subsystems/actuators/actuators_pwm_arch.h"
#include "subsystems/actuators/actuators_pwm.h"
#include "mcu_periph/gpio.h"


/**
 * CMD_TO_US() is depending on architecture
 * and on the hardware settings (clock speed etc.). Hence it has to be
 * defined separately for each board.
 *
 * It converts the actuator command from paparazzi. which is in pulse width
 * in milliseconds to microseconds (required by pwmEnableChannel())
 */
#ifndef PWM_CMD_TO_US
#define PWM_CMD_TO_US(_t) (PWM_FREQUENCY * _t / 1000000)
#endif

int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

/**
 * PWM callback function
 *
 * Called after each period. All PWM configurations (from board.h)
 * should reference to this callback. Empty for now, can be used
 * later for fail safe monitoring (i.e. reset counter or something).
 *
 * @param[in] pwmp pointer to a @p PWMDriver object
 */
 __attribute__((unused)) static void pwmpcb(PWMDriver *pwmp __attribute__((unused))) {}

#if PWM_CONF_TIM1
static PWMConfig pwmcfg1 = PWM_CONF1_DEF;
#endif
#if PWM_CONF_TIM2
static PWMConfig pwmcfg2 = PWM_CONF2_DEF;
#endif
#if PWM_CONF_TIM3
static PWMConfig pwmcfg3 = PWM_CONF3_DEF;
#endif
#if PWM_CONF_TIM4
static PWMConfig pwmcfg4 = PWM_CONF4_DEF;
#endif
#if PWM_CONF_TIM5
static PWMConfig pwmcfg5 = PWM_CONF5_DEF;
#endif
#if PWM_CONF_TIM8
static PWMConfig pwmcfg8 = PWM_CONF8_DEF;
#endif
#if PWM_CONF_TIM9
static PWMConfig pwmcfg9 = PWM_CONF9_DEF;
#endif
#if PWM_CONF_TIM10
static PWMConfig pwmcfg10 = PWM_CONF10_DEF;
#endif
#if PWM_CONF_TIM11
static PWMConfig pwmcfg11 = PWM_CONF11_DEF;
#endif
#if PWM_CONF_TIM12
static PWMConfig pwmcfg12 = PWM_CONF12_DEF;
#endif


void actuators_pwm_arch_init(void)
{
  /*----------------
   * Configure GPIO
   *----------------*/
#ifdef PWM_SERVO_0
  gpio_setup_pin_af(PWM_SERVO_0_GPIO, PWM_SERVO_0_PIN, PWM_SERVO_0_AF, true);
#endif
#ifdef PWM_SERVO_1
  gpio_setup_pin_af(PWM_SERVO_1_GPIO, PWM_SERVO_1_PIN, PWM_SERVO_1_AF, true);
#endif
#ifdef PWM_SERVO_2
  gpio_setup_pin_af(PWM_SERVO_2_GPIO, PWM_SERVO_2_PIN, PWM_SERVO_2_AF, true);
#endif
#ifdef PWM_SERVO_3
  gpio_setup_pin_af(PWM_SERVO_3_GPIO, PWM_SERVO_3_PIN, PWM_SERVO_3_AF, true);
#endif
#ifdef PWM_SERVO_4
  gpio_setup_pin_af(PWM_SERVO_4_GPIO, PWM_SERVO_4_PIN, PWM_SERVO_4_AF, true);
#endif
#ifdef PWM_SERVO_5
  gpio_setup_pin_af(PWM_SERVO_5_GPIO, PWM_SERVO_5_PIN, PWM_SERVO_5_AF, true);
#endif
#ifdef PWM_SERVO_6
  gpio_setup_pin_af(PWM_SERVO_6_GPIO, PWM_SERVO_6_PIN, PWM_SERVO_6_AF, true);
#endif
#ifdef PWM_SERVO_7
  gpio_setup_pin_af(PWM_SERVO_7_GPIO, PWM_SERVO_7_PIN, PWM_SERVO_7_AF, true);
#endif
#ifdef PWM_SERVO_8
  gpio_setup_pin_af(PWM_SERVO_8_GPIO, PWM_SERVO_8_PIN, PWM_SERVO_8_AF, true);
#endif
#ifdef PWM_SERVO_9
  gpio_setup_pin_af(PWM_SERVO_9_GPIO, PWM_SERVO_9_PIN, PWM_SERVO_9_AF, true);
#endif
#ifdef PWM_SERVO_10
  gpio_setup_pin_af(PWM_SERVO_10_GPIO, PWM_SERVO_10_PIN, PWM_SERVO_10_AF, true);
#endif
#ifdef PWM_SERVO_11
  gpio_setup_pin_af(PWM_SERVO_11_GPIO, PWM_SERVO_11_PIN, PWM_SERVO_11_AF, true);
#endif
#ifdef PWM_SERVO_12
  gpio_setup_pin_af(PWM_SERVO_12_GPIO, PWM_SERVO_12_PIN, PWM_SERVO_12_AF, true);
#endif
#ifdef PWM_SERVO_13
  gpio_setup_pin_af(PWM_SERVO_13_GPIO, PWM_SERVO_13_PIN, PWM_SERVO_13_AF, true);
#endif
#ifdef PWM_SERVO_14
  gpio_setup_pin_af(PWM_SERVO_14_GPIO, PWM_SERVO_14_PIN, PWM_SERVO_14_AF, true);
#endif
#ifdef PWM_SERVO_15
  gpio_setup_pin_af(PWM_SERVO_15_GPIO, PWM_SERVO_15_PIN, PWM_SERVO_15_AF, true);
#endif
#ifdef PWM_SERVO_16
  gpio_setup_pin_af(PWM_SERVO_16_GPIO, PWM_SERVO_16_PIN, PWM_SERVO_16_AF, true);
#endif

  /*---------------
   * Configure PWM
   *---------------*/
#if PWM_CONF_TIM1
  pwmStart(&PWMD1, &pwmcfg1);
#endif
#if PWM_CONF_TIM2
  pwmStart(&PWMD2, &pwmcfg2);
#endif
#if PWM_CONF_TIM3
  pwmStart(&PWMD3, &pwmcfg3);
#endif
#if PWM_CONF_TIM4
  pwmStart(&PWMD4, &pwmcfg4);
#endif
#if PWM_CONF_TIM5
  pwmStart(&PWMD5, &pwmcfg5);
#endif
#if PWM_CONF_TIM8
  pwmStart(&PWMD8, &pwmcfg8);
#endif
#if PWM_CONF_TIM9
  pwmStart(&PWMD9, &pwmcfg9);
#endif
#if PWM_CONF_TIM10
  pwmStart(&PWMD10, &pwmcfg10);
#endif
#if PWM_CONF_TIM11
  pwmStart(&PWMD11, &pwmcfg11);
#endif
#if PWM_CONF_TIM12
  pwmStart(&PWMD12, &pwmcfg12);
#endif
}


void actuators_pwm_commit(void)
{
#ifdef PWM_SERVO_0
  pwmEnableChannel(&PWM_SERVO_0_DRIVER, PWM_SERVO_0_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_0]));
#endif
#ifdef PWM_SERVO_1
  pwmEnableChannel(&PWM_SERVO_1_DRIVER, PWM_SERVO_1_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_1]));
#endif
#ifdef PWM_SERVO_2
  pwmEnableChannel(&PWM_SERVO_2_DRIVER, PWM_SERVO_2_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_2]));
#endif
#ifdef PWM_SERVO_3
  pwmEnableChannel(&PWM_SERVO_3_DRIVER, PWM_SERVO_3_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_3]));
#endif
#ifdef PWM_SERVO_4
  pwmEnableChannel(&PWM_SERVO_4_DRIVER, PWM_SERVO_4_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_4]));
#endif
#ifdef PWM_SERVO_5
  pwmEnableChannel(&PWM_SERVO_5_DRIVER, PWM_SERVO_5_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_5]));
#endif
#ifdef PWM_SERVO_6
  pwmEnableChannel(&PWM_SERVO_6_DRIVER, PWM_SERVO_6_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_6]));
#endif
#ifdef PWM_SERVO_7
  pwmEnableChannel(&PWM_SERVO_7_DRIVER, PWM_SERVO_7_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_7]));
#endif
#ifdef PWM_SERVO_8
  pwmEnableChannel(&PWM_SERVO_8_DRIVER, PWM_SERVO_8_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_8]));
#endif
#ifdef PWM_SERVO_9
  pwmEnableChannel(&PWM_SERVO_9_DRIVER, PWM_SERVO_9_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_9]));
#endif
#ifdef PWM_SERVO_10
  pwmEnableChannel(&PWM_SERVO_10_DRIVER, PWM_SERVO_10_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_10]));
#endif
#ifdef PWM_SERVO_11
  pwmEnableChannel(&PWM_SERVO_11_DRIVER, PWM_SERVO_11_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_11]));
#endif
#ifdef PWM_SERVO_12
  pwmEnableChannel(&PWM_SERVO_12_DRIVER, PWM_SERVO_12_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_12]));
#endif
#ifdef PWM_SERVO_13
  pwmEnableChannel(&PWM_SERVO_13_DRIVER, PWM_SERVO_13_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_13]));
#endif
#ifdef PWM_SERVO_14
  pwmEnableChannel(&PWM_SERVO_14_DRIVER, PWM_SERVO_14_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_14]));
#endif
#ifdef PWM_SERVO_15
  pwmEnableChannel(&PWM_SERVO_15_DRIVER, PWM_SERVO_15_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_15]));
#endif
#ifdef PWM_SERVO_16
  pwmEnableChannel(&PWM_SERVO_16_DRIVER, PWM_SERVO_16_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_16]));
#endif
}
