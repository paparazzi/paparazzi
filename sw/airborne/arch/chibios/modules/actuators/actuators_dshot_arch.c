/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/actuators/actuators_dshot_arch.c"
 * @author Gautier Hattenberger
 * Driver for DSHOT speed controller protocol
 * Arch dependent part
 */

#include "modules/actuators/actuators_dshot.h"
#include "modules/actuators/esc_dshot.h"
#include "mcu_periph/ram_arch.h"
#include "mcu_periph/gpio.h"
#include BOARD_CONFIG

uint16_t actuators_dshot_values[ACTUATORS_DSHOT_NB];

#if DSHOT_CONF_TIM1
static IN_DMA_SECTION_NOINIT(DSHOTDriver DSHOTD1);
static DSHOTConfig dshotcfg1 = DSHOT_CONF1_DEF;
#endif
#if DSHOT_CONF_TIM2
static IN_DMA_SECTION_NOINIT(DSHOTDriver  DSHOTD2);
static DSHOTConfig dshotcfg2 = DSHOT_CONF2_DEF;
#endif
#if DSHOT_CONF_TIM3
static IN_DMA_SECTION_NOINIT(DSHOTDriver  DSHOTD3);
static DSHOTConfig dshotcfg3 = DSHOT_CONF3_DEF;
#endif
#if DSHOT_CONF_TIM4
static IN_DMA_SECTION_NOINIT(DSHOTDriver  DSHOTD4);
static DSHOTConfig dshotcfg4 = DSHOT_CONF4_DEF;
#endif
#if DSHOT_CONF_TIM5
static IN_DMA_SECTION_NOINIT(DSHOTDriver  DSHOTD5);
static DSHOTConfig dshotcfg5 = DSHOT_CONF5_DEF;
#endif
#if DSHOT_CONF_TIM8
static IN_DMA_SECTION_NOINIT(DSHOTDriver  DSHOTD8);
static DSHOTConfig dshotcfg8 = DSHOT_CONF8_DEF;
#endif
#if DSHOT_CONF_TIM9
static IN_DMA_SECTION_NOINIT(DSHOTDriver  DSHOTD9);
static DSHOTConfig dshotcfg9 = DSHOT_CONF9_DEF;
#endif


void actuators_dshot_arch_init(void)
{
  /*----------------
   * Configure GPIO
   *----------------*/
#ifdef DSHOT_SERVO_0
  gpio_setup_pin_af(DSHOT_SERVO_0_GPIO, DSHOT_SERVO_0_PIN, DSHOT_SERVO_0_AF, true);
#endif
#ifdef DSHOT_SERVO_1
  gpio_setup_pin_af(DSHOT_SERVO_1_GPIO, DSHOT_SERVO_1_PIN, DSHOT_SERVO_1_AF, true);
#endif
#ifdef DSHOT_SERVO_2
  gpio_setup_pin_af(DSHOT_SERVO_2_GPIO, DSHOT_SERVO_2_PIN, DSHOT_SERVO_2_AF, true);
#endif
#ifdef DSHOT_SERVO_3
  gpio_setup_pin_af(DSHOT_SERVO_3_GPIO, DSHOT_SERVO_3_PIN, DSHOT_SERVO_3_AF, true);
#endif
#ifdef DSHOT_SERVO_4
  gpio_setup_pin_af(DSHOT_SERVO_4_GPIO, DSHOT_SERVO_4_PIN, DSHOT_SERVO_4_AF, true);
#endif
#ifdef DSHOT_SERVO_5
  gpio_setup_pin_af(DSHOT_SERVO_5_GPIO, DSHOT_SERVO_5_PIN, DSHOT_SERVO_5_AF, true);
#endif
#ifdef DSHOT_SERVO_6
  gpio_setup_pin_af(DSHOT_SERVO_6_GPIO, DSHOT_SERVO_6_PIN, DSHOT_SERVO_6_AF, true);
#endif
#ifdef DSHOT_SERVO_7
  gpio_setup_pin_af(DSHOT_SERVO_7_GPIO, DSHOT_SERVO_7_PIN, DSHOT_SERVO_7_AF, true);
#endif
#ifdef DSHOT_SERVO_8
  gpio_setup_pin_af(DSHOT_SERVO_8_GPIO, DSHOT_SERVO_8_PIN, DSHOT_SERVO_8_AF, true);
#endif
#ifdef DSHOT_SERVO_9
  gpio_setup_pin_af(DSHOT_SERVO_9_GPIO, DSHOT_SERVO_9_PIN, DSHOT_SERVO_9_AF, true);
#endif
#ifdef DSHOT_SERVO_10
  gpio_setup_pin_af(DSHOT_SERVO_10_GPIO, DSHOT_SERVO_10_PIN, DSHOT_SERVO_10_AF, true);
#endif
#ifdef DSHOT_SERVO_11
  gpio_setup_pin_af(DSHOT_SERVO_11_GPIO, DSHOT_SERVO_11_PIN, DSHOT_SERVO_11_AF, true);
#endif

  /*---------------
   * Configure DSHOT
   *---------------*/
#if DSHOT_CONF_TIM1
  dshotStart(&DSHOTD1, &dshotcfg1);
#endif
#if DSHOT_CONF_TIM2
  dshotStart(&DSHOTD2, &dshotcfg2);
#endif
#if DSHOT_CONF_TIM3
  dshotStart(&DSHOTD3, &dshotcfg3);
#endif
#if DSHOT_CONF_TIM4
  dshotStart(&DSHOTD4, &dshotcfg4);
#endif
#if DSHOT_CONF_TIM5
  dshotStart(&DSHOTD5, &dshotcfg5);
#endif
#if DSHOT_CONF_TIM8
  dshotStart(&DSHOTD8, &dshotcfg8);
#endif
#if DSHOT_CONF_TIM9
  dshotStart(&DSHOTD9, &dshotcfg9);
#endif
}


void actuators_dshot_arch_commit(void)
{
#ifdef DSHOT_SERVO_0
  dshotSetThrottle(&DSHOT_SERVO_0_DRIVER, DSHOT_SERVO_0_CHANNEL, actuators_dshot_values[DSHOT_SERVO_0]);
#endif
#ifdef DSHOT_SERVO_1
  dshotSetThrottle(&DSHOT_SERVO_1_DRIVER, DSHOT_SERVO_1_CHANNEL, actuators_dshot_values[DSHOT_SERVO_1]);
#endif
#ifdef DSHOT_SERVO_2
  dshotSetThrottle(&DSHOT_SERVO_2_DRIVER, DSHOT_SERVO_2_CHANNEL, actuators_dshot_values[DSHOT_SERVO_2]);
#endif
#ifdef DSHOT_SERVO_3
  dshotSetThrottle(&DSHOT_SERVO_3_DRIVER, DSHOT_SERVO_3_CHANNEL, actuators_dshot_values[DSHOT_SERVO_3]);
#endif
#ifdef DSHOT_SERVO_4
  dshotSetThrottle(&DSHOT_SERVO_4_DRIVER, DSHOT_SERVO_4_CHANNEL, actuators_dshot_values[DSHOT_SERVO_4]);
#endif
#ifdef DSHOT_SERVO_5
  dshotSetThrottle(&DSHOT_SERVO_5_DRIVER, DSHOT_SERVO_5_CHANNEL, actuators_dshot_values[DSHOT_SERVO_5]);
#endif
#ifdef DSHOT_SERVO_6
  dshotSetThrottle(&DSHOT_SERVO_6_DRIVER, DSHOT_SERVO_6_CHANNEL, actuators_dshot_values[DSHOT_SERVO_6]);
#endif
#ifdef DSHOT_SERVO_7
  dshotSetThrottle(&DSHOT_SERVO_7_DRIVER, DSHOT_SERVO_7_CHANNEL, actuators_dshot_values[DSHOT_SERVO_7]);
#endif
#ifdef DSHOT_SERVO_8
  dshotSetThrottle(&DSHOT_SERVO_8_DRIVER, DSHOT_SERVO_8_CHANNEL, actuators_dshot_values[DSHOT_SERVO_8]);
#endif
#ifdef DSHOT_SERVO_9
  dshotSetThrottle(&DSHOT_SERVO_9_DRIVER, DSHOT_SERVO_9_CHANNEL, actuators_dshot_values[DSHOT_SERVO_9]);
#endif
#ifdef DSHOT_SERVO_10
  dshotSetThrottle(&DSHOT_SERVO_10_DRIVER, DSHOT_SERVO_10_CHANNEL, actuators_dshot_values[DSHOT_SERVO_10]);
#endif
#ifdef DSHOT_SERVO_11
  dshotSetThrottle(&DSHOT_SERVO_11_DRIVER, DSHOT_SERVO_11_CHANNEL, actuators_dshot_values[DSHOT_SERVO_11]);
#endif

#if DSHOT_CONF_TIM1
  dshotSendFrame(&DSHOTD1);
#endif
#if DSHOT_CONF_TIM2
  dshotSendFrame(&DSHOTD2);
#endif
#if DSHOT_CONF_TIM3
  dshotSendFrame(&DSHOTD3);
#endif
#if DSHOT_CONF_TIM4
  dshotSendFrame(&DSHOTD4);
#endif
#if DSHOT_CONF_TIM5
  dshotSendFrame(&DSHOTD5);
#endif
#if DSHOT_CONF_TIM8
  dshotSendFrame(&DSHOTD8);
#endif
#if DSHOT_CONF_TIM9
  dshotSendFrame(&DSHOTD9);
#endif
}
