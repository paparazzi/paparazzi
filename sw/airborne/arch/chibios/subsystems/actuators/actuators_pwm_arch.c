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
 * @brief   Interface from actuators to ChibiOS PWM driver
 * @details PWM configuration files are defined in the board file,
 *          so maximal architecture independence is ensured.
 */
#include "subsystems/actuators/actuators_pwm_arch.h"
#include "subsystems/actuators/actuators_pwm.h"

/*
 * Check that the Timer frequency is the same as system clock (for STM32 only)
 */
#ifdef STM32_TIMCLK1
#if STM32_TIMCLK1 != STM32_SYSCLK
#error STM32_TIMCLK1 is not equal to STM32_SYSCLK, please redefine PWM_TO_ST macro
#else
#define PWM_TO_ST(_t) US2ST(_t)
#endif
#endif


/**
 * @brief   PWM callback
 * @details Called after each period. All PWM configurations
 *          should reference to this callback (from board.h).
 *          Empty for now, can be used for fail safe thread.
 *          (i.e. reset counter or something)
 */
static void pwmpcb(PWMDriver *pwmp) {
  (void)pwmp;
}

#if PWM_CONF_TIM1
  static PWMConfig pwmcfg1  = PWM_CONF1_DEF;
#endif
#if PWM_CONF_TIM2
  static PWMConfig pwmcfg2  = PWM_CONF2_DEF;
#endif
#if PWM_CONF_TIM3
  static PWMConfig pwmcfg3  = PWM_CONF3_DEF;
#endif
#if PWM_CONF_TIM4
  static PWMConfig pwmcfg4  = PWM_CONF4_DEF;
#endif
#if PWM_CONF_TIM5
  static PWMConfig pwmcfg5  = PWM_CONF5_DEF;
#endif
#if PWM_CONF_TIM8
  static PWMConfig pwmcfg8  = PWM_CONF8_DEF;
#endif
#if PWM_CONF_TIM9
  static PWMConfig pwmcfg9  = PWM_CONF9_DEF;
#endif


int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

/** @brief PWM arch init called by generic pwm driver
 */
void actuators_pwm_arch_init(void) {
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
}

/** @brief Set pulse widths from actuator values, assumed to be in us
 */
void actuators_pwm_commit(void) {
#ifdef PWM_SERVO_0
  pwmEnableChannel(&PWM_SERVO_0_DRIVER, PWM_SERVO_0_CHANNEL, PWM_TO_ST(actuators_pwm_values[PWM_SERVO_0]));
#endif
#ifdef PWM_SERVO_1
  pwmEnableChannel(&PWM_SERVO_1_DRIVER, PWM_SERVO_1_CHANNEL, PWM_TO_ST(actuators_pwm_values[PWM_SERVO_1]));
#endif
#ifdef PWM_SERVO_2
  pwmEnableChannel(&PWM_SERVO_2_DRIVER, PWM_SERVO_2_CHANNEL, PWM_TO_ST(actuators_pwm_values[PWM_SERVO_2]));
#endif
#ifdef PWM_SERVO_3
  pwmEnableChannel(&PWM_SERVO_3_DRIVER, PWM_SERVO_3_CHANNEL, PWM_TO_ST(actuators_pwm_values[PWM_SERVO_3]));
#endif
#ifdef PWM_SERVO_4
  pwmEnableChannel(&PWM_SERVO_4_DRIVER, PWM_SERVO_4_CHANNEL, PWM_TO_ST(actuators_pwm_values[PWM_SERVO_4]));
#endif
#ifdef PWM_SERVO_5
  pwmEnableChannel(&PWM_SERVO_5_DRIVER, PWM_SERVO_5_CHANNEL, PWM_TO_ST(actuators_pwm_values[PWM_SERVO_5]));
#endif
#ifdef PWM_SERVO_6
  pwmEnableChannel(&PWM_SERVO_6_DRIVER, PWM_SERVO_6_CHANNEL, PWM_TO_ST(actuators_pwm_values[PWM_SERVO_6]));
#endif
#ifdef PWM_SERVO_7
  pwmEnableChannel(&PWM_SERVO_7_DRIVER, PWM_SERVO_7_CHANNEL, PWM_TO_ST(actuators_pwm_values[PWM_SERVO_7]));
#endif
#ifdef PWM_SERVO_8
  pwmEnableChannel(&PWM_SERVO_8_DRIVER, PWM_SERVO_8_CHANNEL, PWM_TO_ST(actuators_pwm_values[PWM_SERVO_8]));
#endif
#ifdef PWM_SERVO_9
  pwmEnableChannel(&PWM_SERVO_9_DRIVER, PWM_SERVO_9_CHANNEL, PWM_TO_ST(actuators_pwm_values[PWM_SERVO_9]));
#endif
#ifdef PWM_SERVO_10
  pwmEnableChannel(&PWM_SERVO_10_DRIVER, PWM_SERVO_10_CHANNEL, PWM_TO_ST(actuators_pwm_values[PWM_SERVO_10]));
#endif
#ifdef PWM_SERVO_11
  pwmEnableChannel(&PWM_SERVO_11_DRIVER, PWM_SERVO_11_CHANNEL, PWM_TO_ST(actuators_pwm_values[PWM_SERVO_11]));
#endif
}
