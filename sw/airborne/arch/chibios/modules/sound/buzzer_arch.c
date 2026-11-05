/*
 * Copyright (C) 2026 Tomaso De Ponti <tomaso@aerogriduav.com>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file    arch/chibios/modules/sound/buzzer_arch.c
 * @brief   Piezo buzzer tone output with a ChibiOS PWM driver
 */

#include <hal.h>
#include "modules/sound/buzzer_arch.h"
#include "mcu_periph/gpio.h"

#include BOARD_CONFIG

/** PWM timer counting frequency, 1 us resolution */
#define BUZZER_PWM_FREQUENCY 1000000

static PWMConfig buzzer_pwm_conf = {
  .frequency = BUZZER_PWM_FREQUENCY,
  .period    = BUZZER_PWM_FREQUENCY / 1000,
  .callback  = NULL,
  .channels  = {
    { .mode = PWM_OUTPUT_DISABLED, .callback = NULL },
    { .mode = PWM_OUTPUT_DISABLED, .callback = NULL },
    { .mode = PWM_OUTPUT_DISABLED, .callback = NULL },
    { .mode = PWM_OUTPUT_DISABLED, .callback = NULL },
  },
  .cr2  = 0,
  .dier = 0
};

void buzzer_arch_init(void)
{
  gpio_setup_pin_af(BUZZER_GPIO, BUZZER_PIN, BUZZER_AF, true);
  buzzer_pwm_conf.channels[BUZZER_PWM_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
  pwmStart(&BUZZER_PWMD, &buzzer_pwm_conf);
  pwmDisableChannel(&BUZZER_PWMD, BUZZER_PWM_CHANNEL);
}

void buzzer_arch_tone(uint16_t freq)
{
  if (freq == 0) {
    pwmDisableChannel(&BUZZER_PWMD, BUZZER_PWM_CHANNEL);
    return;
  }
  pwmcnt_t period = BUZZER_PWM_FREQUENCY / freq;
  pwmChangePeriod(&BUZZER_PWMD, period);
  pwmEnableChannel(&BUZZER_PWMD, BUZZER_PWM_CHANNEL, period / 2);
}
