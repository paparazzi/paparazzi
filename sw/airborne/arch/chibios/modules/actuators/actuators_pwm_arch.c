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
 * @file arch/chibios/modules/actuators/actuators_pwm_arch.c
 * Interface from actuators to ChibiOS PWM driver
 *
 * PWM configuration files are defined in the board file,
 * so maximal architecture independence is ensured.
 */
#include "modules/actuators/actuators_pwm_arch.h"
#include "modules/actuators/actuators_pwm.h"
#include "mcu_periph/gpio.h"

/* Default timer base frequency is 1MHz */
#ifndef PWM_FREQUENCY
#define PWM_FREQUENCY 1000000
#endif

/* Default servo update rate in Hz */
#ifndef SERVO_HZ
#define SERVO_HZ 40
#endif

/* For each timer the period van be configured differently */
#ifndef TIM1_SERVO_HZ
#define TIM1_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM2_SERVO_HZ
#define TIM2_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM3_SERVO_HZ
#define TIM3_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM4_SERVO_HZ
#define TIM4_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM5_SERVO_HZ
#define TIM5_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM8_SERVO_HZ
#define TIM8_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM9_SERVO_HZ
#define TIM9_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM12_SERVO_HZ
#define TIM12_SERVO_HZ SERVO_HZ
#endif

/**
 * Print the configuration variables from the header
 */
PRINT_CONFIG_VAR(ACTUATORS_PWM_NB)
PRINT_CONFIG_VAR(PWM_FREQUENCY)
PRINT_CONFIG_VAR(SERVO_HZ)

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

#if USE_PWM_TIM1
static PWMConfig pwmcfg1 = {
  .frequency = PWM_FREQUENCY,
  .period = PWM_FREQUENCY/TIM1_SERVO_HZ,
  .callback = NULL,
  .channels = {
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
  },
  .cr2 = 0,
  .bdtr = 0,
  .dier = 0
};
#endif
#if USE_PWM_TIM2
static PWMConfig pwmcfg2 = {
  .frequency = PWM_FREQUENCY,
  .period = PWM_FREQUENCY/TIM2_SERVO_HZ,
  .callback = NULL,
  .channels = {
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
  },
  .cr2 = 0,
  .bdtr = 0,
  .dier = 0
};
#endif
#if USE_PWM_TIM3
static PWMConfig pwmcfg3 = {
  .frequency = PWM_FREQUENCY,
  .period = PWM_FREQUENCY/TIM3_SERVO_HZ,
  .callback = NULL,
  .channels = {
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
  },
  .cr2 = 0,
  .bdtr = 0,
  .dier = 0
};
#endif
#if USE_PWM_TIM4
static PWMConfig pwmcfg4 = {
  .frequency = PWM_FREQUENCY,
  .period = PWM_FREQUENCY/TIM4_SERVO_HZ,
  .callback = NULL,
  .channels = {
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
  },
  .cr2 = 0,
  .bdtr = 0,
  .dier = 0
};
#endif
#if USE_PWM_TIM5
static PWMConfig pwmcfg5 = {
  .frequency = PWM_FREQUENCY,
  .period = PWM_FREQUENCY/TIM5_SERVO_HZ,
  .callback = NULL,
  .channels = {
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
  },
  .cr2 = 0,
  .bdtr = 0,
  .dier = 0
};
#endif
#if USE_PWM_TIM8
static PWMConfig pwmcfg8 = {
  .frequency = PWM_FREQUENCY,
  .period = PWM_FREQUENCY/TIM8_SERVO_HZ,
  .callback = NULL,
  .channels = {
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
  },
  .cr2 = 0,
  .bdtr = 0,
  .dier = 0
};
#endif
#if USE_PWM_TIM9
static PWMConfig pwmcfg9 = {
  .frequency = PWM_FREQUENCY,
  .period = PWM_FREQUENCY/TIM9_SERVO_HZ,
  .callback = NULL,
  .channels = {
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
  },
  .cr2 = 0,
  .bdtr = 0,
  .dier = 0
};
#endif
#if USE_PWM_TIM10
static PWMConfig pwmcfg10 = {
  .frequency = PWM_FREQUENCY,
  .period = PWM_FREQUENCY/TIM10_SERVO_HZ,
  .callback = NULL,
  .channels = {
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
  },
  .cr2 = 0,
  .bdtr = 0,
  .dier = 0
};
#endif
#if USE_PWM_TIM11
static PWMConfig pwmcfg11 = {
  .frequency = PWM_FREQUENCY,
  .period = PWM_FREQUENCY/TIM11_SERVO_HZ,
  .callback = NULL,
  .channels = {
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
  },
  .cr2 = 0,
  .bdtr = 0,
  .dier = 0
};
#endif
#if USE_PWM_TIM12
static PWMConfig pwmcfg12 = {
  .frequency = PWM_FREQUENCY,
  .period = PWM_FREQUENCY/TIM12_SERVO_HZ,
  .callback = NULL,
  .channels = {
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
    { PWM_OUTPUT_DISABLED, NULL },
  },
  .cr2 = 0,
  .bdtr = 0,
  .dier = 0
};
#endif


void actuators_pwm_arch_init(void)
{
  /*----------------
   * Configure GPIO
   *----------------*/
#if USE_PWM_SERVO0
  gpio_setup_pin_af(SERVO0_GPIO, SERVO0_PIN, SERVO0_AF, true);
  SERVO0_PWM_CONF.channels[SERVO0_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO1
  gpio_setup_pin_af(SERVO1_GPIO, SERVO1_PIN, SERVO1_AF, true);
  SERVO1_PWM_CONF.channels[SERVO1_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO2
  gpio_setup_pin_af(SERVO2_GPIO, SERVO2_PIN, SERVO2_AF, true);
  SERVO2_PWM_CONF.channels[SERVO2_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO3
  gpio_setup_pin_af(SERVO3_GPIO, SERVO3_PIN, SERVO3_AF, true);
  SERVO3_PWM_CONF.channels[SERVO3_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO4
  gpio_setup_pin_af(SERVO4_GPIO, SERVO4_PIN, SERVO4_AF, true);
  SERVO4_PWM_CONF.channels[SERVO4_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO5
  gpio_setup_pin_af(SERVO5_GPIO, SERVO5_PIN, SERVO5_AF, true);
  SERVO5_PWM_CONF.channels[SERVO5_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO6
  gpio_setup_pin_af(SERVO6_GPIO, SERVO6_PIN, SERVO6_AF, true);
  SERVO6_PWM_CONF.channels[SERVO6_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO7
  gpio_setup_pin_af(SERVO7_GPIO, SERVO7_PIN, SERVO7_AF, true);
  SERVO7_PWM_CONF.channels[SERVO7_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO8
  gpio_setup_pin_af(SERVO8_GPIO, SERVO8_PIN, SERVO8_AF, true);
  SERVO8_PWM_CONF.channels[SERVO8_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO9
  gpio_setup_pin_af(SERVO9_GPIO, SERVO9_PIN, SERVO9_AF, true);
  SERVO9_PWM_CONF.channels[SERVO9_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO10
  gpio_setup_pin_af(SERVO10_GPIO, SERVO10_PIN, SERVO10_AF, true);
  SERVO10_PWM_CONF.channels[SERVO10_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO11
  gpio_setup_pin_af(SERVO11_GPIO, SERVO11_PIN, SERVO11_AF, true);
  SERVO11_PWM_CONF.channels[SERVO11_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO12
  gpio_setup_pin_af(SERVO12_GPIO, SERVO12_PIN, SERVO12_AF, true);
  SERVO12_PWM_CONF.channels[SERVO12_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO13
  gpio_setup_pin_af(SERVO13_GPIO, SERVO13_PIN, SERVO13_AF, true);
  SERVO13_PWM_CONF.channels[SERVO13_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO14
  gpio_setup_pin_af(SERVO14_GPIO, SERVO14_PIN, SERVO14_AF, true);
  SERVO14_PWM_CONF.channels[SERVO14_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO15
  gpio_setup_pin_af(SERVO15_GPIO, SERVO15_PIN, SERVO15_AF, true);
  SERVO15_PWM_CONF.channels[SERVO15_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif
#if USE_PWM_SERVO16
  gpio_setup_pin_af(SERVO16_GPIO, SERVO16_PIN, SERVO16_AF, true);
  SERVO16_PWM_CONF.channels[SERVO16_CHANNEL].mode = PWM_OUTPUT_ACTIVE_HIGH;
#endif

  /*---------------
   * Configure PWM
   *---------------*/
#if USE_PWM_TIM1
  pwmStart(&PWMD1, &pwmcfg1);
#endif
#if USE_PWM_TIM2
  pwmStart(&PWMD2, &pwmcfg2);
#endif
#if USE_PWM_TIM3
  pwmStart(&PWMD3, &pwmcfg3);
#endif
#if USE_PWM_TIM4
  pwmStart(&PWMD4, &pwmcfg4);
#endif
#if USE_PWM_TIM5
  pwmStart(&PWMD5, &pwmcfg5);
#endif
#if USE_PWM_TIM8
  pwmStart(&PWMD8, &pwmcfg8);
#endif
#if USE_PWM_TIM9
  pwmStart(&PWMD9, &pwmcfg9);
#endif
#if USE_PWM_TIM10
  pwmStart(&PWMD10, &pwmcfg10);
#endif
#if USE_PWM_TIM11
  pwmStart(&PWMD11, &pwmcfg11);
#endif
#if USE_PWM_TIM12
  pwmStart(&PWMD12, &pwmcfg12);
#endif
}


void actuators_pwm_commit(void)
{
#if USE_PWM_SERVO0
  pwmEnableChannel(&SERVO0_PWM_DRIVER, SERVO0_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO0]));
#endif
#if USE_PWM_SERVO1
  pwmEnableChannel(&SERVO1_PWM_DRIVER, SERVO1_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO1]));
#endif
#if USE_PWM_SERVO2
  pwmEnableChannel(&SERVO2_PWM_DRIVER, SERVO2_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO2]));
#endif
#if USE_PWM_SERVO3
  pwmEnableChannel(&SERVO3_PWM_DRIVER, SERVO3_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO3]));
#endif
#if USE_PWM_SERVO4
  pwmEnableChannel(&SERVO4_PWM_DRIVER, SERVO4_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO4]));
#endif
#if USE_PWM_SERVO5
  pwmEnableChannel(&SERVO5_PWM_DRIVER, SERVO5_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO5]));
#endif
#if USE_PWM_SERVO6
  pwmEnableChannel(&SERVO6_PWM_DRIVER, SERVO6_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO6]));
#endif
#if USE_PWM_SERVO7
  pwmEnableChannel(&SERVO7_PWM_DRIVER, SERVO7_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO7]));
#endif
#if USE_PWM_SERVO8
  pwmEnableChannel(&SERVO8_PWM_DRIVER, SERVO8_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO8]));
#endif
#if USE_PWM_SERVO9
  pwmEnableChannel(&SERVO9_PWM_DRIVER, SERVO9_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO9]));
#endif
#if USE_PWM_SERVO10
  pwmEnableChannel(&SERVO10_PWM_DRIVER, SERVO10_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO10]));
#endif
#if USE_PWM_SERVO11
  pwmEnableChannel(&SERVO11_PWM_DRIVER, SERVO11_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO11]));
#endif
#if USE_PWM_SERVO12
  pwmEnableChannel(&SERVO12_PWM_DRIVER, SERVO12_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO12]));
#endif
#if USE_PWM_SERVO13
  pwmEnableChannel(&SERVO13_PWM_DRIVER, SERVO13_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO13]));
#endif
#if USE_PWM_SERVO14
  pwmEnableChannel(&SERVO14_PWM_DRIVER, SERVO14_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO14]));
#endif
#if USE_PWM_SERVO15
  pwmEnableChannel(&SERVO15_PWM_DRIVER, SERVO15_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO15]));
#endif
#if USE_PWM_SERVO16
  pwmEnableChannel(&SERVO16_PWM_DRIVER, SERVO16_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[SERVO16]));
#endif
}
