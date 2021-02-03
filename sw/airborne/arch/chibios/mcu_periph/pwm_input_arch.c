/*
 * Copyright (C) 2015 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file arch/chibios/mcu_periph/pwm_input_arch.c
 * @ingroup chibios_arch
 *
 * handling of stm32 PWM input using a timer with capture.
 */
#include "mcu_periph/pwm_input_arch.h"
#include "mcu_periph/pwm_input.h"
#include "mcu_periph/gpio.h"
#include <hal.h>
#include BOARD_CONFIG

#define ONE_MHZ_CLK 1000000

#ifdef USE_PWM_INPUT1
static void input1_period_cb(ICUDriver *icup) {
  pwm_input_period_tics[PWM_INPUT1] = icuGetPeriodX(icup);
  pwm_input_period_valid[PWM_INPUT1] = true;
}

static void input1_width_cb(ICUDriver *icup) {
  pwm_input_duty_tics[PWM_INPUT1] = icuGetWidthX(icup);
  pwm_input_duty_valid[PWM_INPUT1] = true;
}

static ICUConfig pwm_input1_cfg = {
#if USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_LOW
  ICU_INPUT_ACTIVE_LOW,
#elif USE_PWM_INPUT1 == PWM_PULSE_TYPE_ACTIVE_HIGH
  ICU_INPUT_ACTIVE_HIGH,
#else
#error "Unknown PWM_INPUT1_PULSE_TYPE"
#endif
  PWM_INPUT1_TICKS_PER_USEC * ONE_MHZ_CLK,
  input1_width_cb,
  input1_period_cb,
  NULL,
  PWM_INPUT1_CHANNEL,
  0,
  0xFFFFFFFFU
};
#endif

#ifdef USE_PWM_INPUT2
static void input2_period_cb(ICUDriver *icup) {
  pwm_input_period_tics[PWM_INPUT2] = icuGetPeriodX(icup);
  pwm_input_period_valid[PWM_INPUT2] = true;
}

static void input2_width_cb(ICUDriver *icup) {
  pwm_input_duty_tics[PWM_INPUT2] = icuGetWidthX(icup);
  pwm_input_duty_valid[PWM_INPUT2] = true;
}

static ICUConfig pwm_input2_cfg = {
#if USE_PWM_INPUT2 == PWM_PULSE_TYPE_ACTIVE_LOW
  ICU_INPUT_ACTIVE_LOW,
#elif USE_PWM_INPUT2 == PWM_PULSE_TYPE_ACTIVE_HIGH
  ICU_INPUT_ACTIVE_HIGH,
#else
#error "Unknown PWM_INPUT2_PULSE_TYPE"
#endif
  PWM_INPUT2_TICKS_PER_USEC * ONE_MHZ_CLK,
  input2_width_cb,
  input2_period_cb,
  NULL,
  PWM_INPUT2_CHANNEL,
  0,
  0xFFFFFFFFU
};
#endif

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

#ifdef USE_PWM_INPUT1
  icuStart(&PWM_INPUT1_ICU, &pwm_input1_cfg);
  gpio_setup_pin_af(PWM_INPUT1_GPIO_PORT, PWM_INPUT1_GPIO_PIN, PWM_INPUT1_GPIO_AF, false);
  icuStartCapture(&PWM_INPUT1_ICU);
  icuEnableNotifications(&PWM_INPUT1_ICU);
#endif

#ifdef USE_PWM_INPUT2
  icuStart(&PWM_INPUT2_ICU, &pwm_input2_cfg);
  gpio_setup_pin_af(PWM_INPUT2_GPIO_PORT, PWM_INPUT2_GPIO_PIN, PWM_INPUT2_GPIO_AF, false);
  icuStartCapture(&PWM_INPUT2_ICU);
  icuEnableNotifications(&PWM_INPUT2_ICU);
#endif

}

