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
#include "modules/core/abi.h"
#include "mcu_periph/ram_arch.h"
#include "mcu_periph/gpio.h"
#include BOARD_CONFIG

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "modules/energy/electrical.h"
#endif

struct dshot_private {
  DSHOTDriver *driver;
  uint32_t channel;
};

struct dshot actuators_dshot_values[ACTUATORS_DSHOT_NB];
struct dshot_private actuators_dshot_private[ACTUATORS_DSHOT_NB];

#if DSHOT_CONF_TIM1
static DSHOTDriver DSHOTD1;
static IN_DMA_SECTION_NOINIT(DshotDmaBuffer dshot1DmaBuffer);
static DSHOTConfig dshotcfg1 = DSHOT_CONF1_DEF;
#endif
#if DSHOT_CONF_TIM2
static DSHOTDriver  DSHOTD2;
static IN_DMA_SECTION_NOINIT(DshotDmaBuffer dshot2DmaBuffer);
static DSHOTConfig dshotcfg2 = DSHOT_CONF2_DEF;
#endif
#if DSHOT_CONF_TIM3
static DSHOTDriver  DSHOTD3;
static IN_DMA_SECTION_NOINIT(DshotDmaBuffer dshot3DmaBuffer);
static DSHOTConfig dshotcfg3 = DSHOT_CONF3_DEF;
#endif
#if DSHOT_CONF_TIM4
static DSHOTDriver  DSHOTD4;
static IN_DMA_SECTION_NOINIT(DshotDmaBuffer dshot4DmaBuffer);
static DSHOTConfig dshotcfg4 = DSHOT_CONF4_DEF;
#endif
#if DSHOT_CONF_TIM5
static DSHOTDriver  DSHOTD5;
static IN_DMA_SECTION_NOINIT(DshotDmaBuffer dshot5DmaBuffer);
static DSHOTConfig dshotcfg5 = DSHOT_CONF5_DEF;
#endif
#if DSHOT_CONF_TIM8
static DSHOTDriver  DSHOTD8;
static IN_DMA_SECTION_NOINIT(DshotDmaBuffer dshot8DmaBuffer);
static DSHOTConfig dshotcfg8 = DSHOT_CONF8_DEF;
#endif
#if DSHOT_CONF_TIM9
static DSHOTDriver  DSHOTD9;
static IN_DMA_SECTION_NOINIT(DshotDmaBuffer dshot9DmaBuffer);
static DSHOTConfig dshotcfg9 = DSHOT_CONF9_DEF;
#endif

#if PERIODIC_TELEMETRY
static void esc_msg_send(struct transport_tx *trans, struct link_device *dev) {
  for (uint8_t i = 0; i < ACTUATORS_DSHOT_NB; i++) {
    if (actuators_dshot_values[i].activated) {
      const DshotTelemetry *dtelem = dshotGetTelemetry(actuators_dshot_private[i].driver, actuators_dshot_private[i].channel);

      actuators_dshot_values[i].current = (float)dtelem->current * 0.01f;
      actuators_dshot_values[i].voltage = (float)dtelem->voltage * 0.01f;
      actuators_dshot_values[i].rpm = (float)dtelem->rpm;
      float bat_voltage = electrical.vsupply;
      float power = actuators_dshot_values[i].current * bat_voltage;
      float energy = (float)dtelem->consumption;
      pprz_msg_send_ESC(trans, dev, AC_ID,
          &actuators_dshot_values[i].current,
          &bat_voltage,
          &power,
          &actuators_dshot_values[i].rpm,
          &actuators_dshot_values[i].voltage,
          &energy,
          &i);
    }
  }
}
#endif

static void dshot_init_struct(struct dshot * ds)
{
  ds->cmd = 0;
  ds->rpm = 0;
  ds->current = 0;
  ds->voltage = 0;
  ds->activated = false;
}

static void dshot_set_struct(struct dshot * ds, struct dshot_private * dsp, DSHOTDriver * driver, uint32_t channel)
{
  ds->activated = true;
  dsp->driver = driver;
  dsp->channel = channel;
}

#define _CONCAT_GPIO(num, name) UART ## num ## _GPIO_ ## name
#define CONCAT_GPIO(num, name) _CONCAT_GPIO(num, name)

void actuators_dshot_arch_init(void)
{
  // init dshot structure to zero
  for (int i = 0; i < ACTUATORS_DSHOT_NB; i++) {
    dshot_init_struct(&actuators_dshot_values[i]);
  }

  // configure telemetry pin if needed
  // the serial device interface might have to be activated
  // by hand (ChibiOS HAL), but PPRZ one disabled
#ifdef DSHOT_TIM1_TELEMETRY_NUM
  gpio_setup_pin_af(
      CONCAT_GPIO(DSHOT_TIM1_TELEMETRY_NUM, PORT_RX),
      CONCAT_GPIO(DSHOT_TIM1_TELEMETRY_NUM, RX),
      CONCAT_GPIO(DSHOT_TIM1_TELEMETRY_NUM, AF), FALSE);
#endif
#ifdef DSHOT_TIM2_TELEMETRY_NUM
  gpio_setup_pin_af(
      CONCAT_GPIO(DSHOT_TIM2_TELEMETRY_NUM, PORT_RX),
      CONCAT_GPIO(DSHOT_TIM2_TELEMETRY_NUM, RX),
      CONCAT_GPIO(DSHOT_TIM2_TELEMETRY_NUM, AF), FALSE);
#endif
#ifdef DSHOT_TIM3_TELEMETRY_NUM
  gpio_setup_pin_af(
      CONCAT_GPIO(DSHOT_TIM3_TELEMETRY_NUM, PORT_RX),
      CONCAT_GPIO(DSHOT_TIM3_TELEMETRY_NUM, RX),
      CONCAT_GPIO(DSHOT_TIM3_TELEMETRY_NUM, AF), FALSE);
#endif
#ifdef DSHOT_TIM4_TELEMETRY_NUM
  gpio_setup_pin_af(
      CONCAT_GPIO(DSHOT_TIM4_TELEMETRY_NUM, PORT_RX),
      CONCAT_GPIO(DSHOT_TIM4_TELEMETRY_NUM, RX),
      CONCAT_GPIO(DSHOT_TIM4_TELEMETRY_NUM, AF), FALSE);
#endif
#ifdef DSHOT_TIM5_TELEMETRY_NUM
  gpio_setup_pin_af(
      CONCAT_GPIO(DSHOT_TIM5_TELEMETRY_NUM, PORT_RX),
      CONCAT_GPIO(DSHOT_TIM5_TELEMETRY_NUM, RX),
      CONCAT_GPIO(DSHOT_TIM5_TELEMETRY_NUM, AF), FALSE);
#endif
#ifdef DSHOT_TIM8_TELEMETRY_NUM
  gpio_setup_pin_af(
      CONCAT_GPIO(DSHOT_TIM8_TELEMETRY_NUM, PORT_RX),
      CONCAT_GPIO(DSHOT_TIM8_TELEMETRY_NUM, RX),
      CONCAT_GPIO(DSHOT_TIM8_TELEMETRY_NUM, AF), FALSE);
#endif
#ifdef DSHOT_TIM9_TELEMETRY_NUM
  gpio_setup_pin_af(
      CONCAT_GPIO(DSHOT_TIM9_TELEMETRY_NUM, PORT_RX),
      CONCAT_GPIO(DSHOT_TIM9_TELEMETRY_NUM, RX),
      CONCAT_GPIO(DSHOT_TIM9_TELEMETRY_NUM, AF), FALSE);
#endif

  /*----------------
   * Configure GPIO
   *----------------*/
#ifdef DSHOT_SERVO_0
  gpio_setup_pin_af(DSHOT_SERVO_0_GPIO, DSHOT_SERVO_0_PIN, DSHOT_SERVO_0_AF, true);
  dshot_set_struct(&actuators_dshot_values[DSHOT_SERVO_0], &actuators_dshot_private[DSHOT_SERVO_0], &DSHOT_SERVO_0_DRIVER, DSHOT_SERVO_0_CHANNEL);
#endif
#ifdef DSHOT_SERVO_1
  gpio_setup_pin_af(DSHOT_SERVO_1_GPIO, DSHOT_SERVO_1_PIN, DSHOT_SERVO_1_AF, true);
  dshot_set_struct(&actuators_dshot_values[DSHOT_SERVO_1], &actuators_dshot_private[DSHOT_SERVO_1], &DSHOT_SERVO_1_DRIVER, DSHOT_SERVO_1_CHANNEL);
#endif
#ifdef DSHOT_SERVO_2
  gpio_setup_pin_af(DSHOT_SERVO_2_GPIO, DSHOT_SERVO_2_PIN, DSHOT_SERVO_2_AF, true);
  dshot_set_struct(&actuators_dshot_values[DSHOT_SERVO_2], &actuators_dshot_private[DSHOT_SERVO_2], &DSHOT_SERVO_2_DRIVER, DSHOT_SERVO_2_CHANNEL);
#endif
#ifdef DSHOT_SERVO_3
  gpio_setup_pin_af(DSHOT_SERVO_3_GPIO, DSHOT_SERVO_3_PIN, DSHOT_SERVO_3_AF, true);
  dshot_set_struct(&actuators_dshot_values[DSHOT_SERVO_3], &actuators_dshot_private[DSHOT_SERVO_3], &DSHOT_SERVO_3_DRIVER, DSHOT_SERVO_3_CHANNEL);
#endif
#ifdef DSHOT_SERVO_4
  gpio_setup_pin_af(DSHOT_SERVO_4_GPIO, DSHOT_SERVO_4_PIN, DSHOT_SERVO_4_AF, true);
  dshot_set_struct(&actuators_dshot_values[DSHOT_SERVO_4], &actuators_dshot_private[DSHOT_SERVO_4], &DSHOT_SERVO_4_DRIVER, DSHOT_SERVO_4_CHANNEL);
#endif
#ifdef DSHOT_SERVO_5
  gpio_setup_pin_af(DSHOT_SERVO_5_GPIO, DSHOT_SERVO_5_PIN, DSHOT_SERVO_5_AF, true);
  dshot_set_struct(&actuators_dshot_values[DSHOT_SERVO_5], &actuators_dshot_private[DSHOT_SERVO_5], &DSHOT_SERVO_5_DRIVER, DSHOT_SERVO_5_CHANNEL);
#endif
#ifdef DSHOT_SERVO_6
  gpio_setup_pin_af(DSHOT_SERVO_6_GPIO, DSHOT_SERVO_6_PIN, DSHOT_SERVO_6_AF, true);
  dshot_set_struct(&actuators_dshot_values[DSHOT_SERVO_6], &actuators_dshot_private[DSHOT_SERVO_6], &DSHOT_SERVO_6_DRIVER, DSHOT_SERVO_6_CHANNEL);
#endif
#ifdef DSHOT_SERVO_7
  gpio_setup_pin_af(DSHOT_SERVO_7_GPIO, DSHOT_SERVO_7_PIN, DSHOT_SERVO_7_AF, true);
  dshot_set_struct(&actuators_dshot_values[DSHOT_SERVO_7], &actuators_dshot_private[DSHOT_SERVO_7], &DSHOT_SERVO_7_DRIVER, DSHOT_SERVO_7_CHANNEL);
#endif
#ifdef DSHOT_SERVO_8
  gpio_setup_pin_af(DSHOT_SERVO_8_GPIO, DSHOT_SERVO_8_PIN, DSHOT_SERVO_8_AF, true);
  dshot_set_struct(&actuators_dshot_values[DSHOT_SERVO_8], &actuators_dshot_private[DSHOT_SERVO_8], &DSHOT_SERVO_8_DRIVER, DSHOT_SERVO_8_CHANNEL);
#endif
#ifdef DSHOT_SERVO_9
  gpio_setup_pin_af(DSHOT_SERVO_9_GPIO, DSHOT_SERVO_9_PIN, DSHOT_SERVO_9_AF, true);
  dshot_set_struct(&actuators_dshot_values[DSHOT_SERVO_9], &actuators_dshot_private[DSHOT_SERVO_9], &DSHOT_SERVO_9_DRIVER, DSHOT_SERVO_9_CHANNEL);
#endif
#ifdef DSHOT_SERVO_10
  gpio_setup_pin_af(DSHOT_SERVO_10_GPIO, DSHOT_SERVO_10_PIN, DSHOT_SERVO_10_AF, true);
  dshot_set_struct(&actuators_dshot_values[DSHOT_SERVO_10], &actuators_dshot_private[DSHOT_SERVO_10], &DSHOT_SERVO_10_DRIVER, DSHOT_SERVO_10_CHANNEL);
#endif
#ifdef DSHOT_SERVO_11
  gpio_setup_pin_af(DSHOT_SERVO_11_GPIO, DSHOT_SERVO_11_PIN, DSHOT_SERVO_11_AF, true);
  dshot_set_struct(&actuators_dshot_values[DSHOT_SERVO_11], &actuators_dshot_private[DSHOT_SERVO_11], &DSHOT_SERVO_11_DRIVER, DSHOT_SERVO_11_CHANNEL);
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

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ESC, esc_msg_send);
#endif
}


void actuators_dshot_arch_commit(void)
{
#ifdef DSHOT_SERVO_0
  dshotSetThrottle(&DSHOT_SERVO_0_DRIVER, DSHOT_SERVO_0_CHANNEL, actuators_dshot_values[DSHOT_SERVO_0].cmd);
#endif
#ifdef DSHOT_SERVO_1
  dshotSetThrottle(&DSHOT_SERVO_1_DRIVER, DSHOT_SERVO_1_CHANNEL, actuators_dshot_values[DSHOT_SERVO_1].cmd);
#endif
#ifdef DSHOT_SERVO_2
  dshotSetThrottle(&DSHOT_SERVO_2_DRIVER, DSHOT_SERVO_2_CHANNEL, actuators_dshot_values[DSHOT_SERVO_2].cmd);
#endif
#ifdef DSHOT_SERVO_3
  dshotSetThrottle(&DSHOT_SERVO_3_DRIVER, DSHOT_SERVO_3_CHANNEL, actuators_dshot_values[DSHOT_SERVO_3].cmd);
#endif
#ifdef DSHOT_SERVO_4
  dshotSetThrottle(&DSHOT_SERVO_4_DRIVER, DSHOT_SERVO_4_CHANNEL, actuators_dshot_values[DSHOT_SERVO_4].cmd);
#endif
#ifdef DSHOT_SERVO_5
  dshotSetThrottle(&DSHOT_SERVO_5_DRIVER, DSHOT_SERVO_5_CHANNEL, actuators_dshot_values[DSHOT_SERVO_5].cmd);
#endif
#ifdef DSHOT_SERVO_6
  dshotSetThrottle(&DSHOT_SERVO_6_DRIVER, DSHOT_SERVO_6_CHANNEL, actuators_dshot_values[DSHOT_SERVO_6].cmd);
#endif
#ifdef DSHOT_SERVO_7
  dshotSetThrottle(&DSHOT_SERVO_7_DRIVER, DSHOT_SERVO_7_CHANNEL, actuators_dshot_values[DSHOT_SERVO_7].cmd);
#endif
#ifdef DSHOT_SERVO_8
  dshotSetThrottle(&DSHOT_SERVO_8_DRIVER, DSHOT_SERVO_8_CHANNEL, actuators_dshot_values[DSHOT_SERVO_8].cmd);
#endif
#ifdef DSHOT_SERVO_9
  dshotSetThrottle(&DSHOT_SERVO_9_DRIVER, DSHOT_SERVO_9_CHANNEL, actuators_dshot_values[DSHOT_SERVO_9].cmd);
#endif
#ifdef DSHOT_SERVO_10
  dshotSetThrottle(&DSHOT_SERVO_10_DRIVER, DSHOT_SERVO_10_CHANNEL, actuators_dshot_values[DSHOT_SERVO_10].cmd);
#endif
#ifdef DSHOT_SERVO_11
  dshotSetThrottle(&DSHOT_SERVO_11_DRIVER, DSHOT_SERVO_11_CHANNEL, actuators_dshot_values[DSHOT_SERVO_11].cmd);
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

  uint16_t rpm_list[ACTUATORS_DSHOT_NB] = { 0 };
  for (uint8_t i = 0; i < ACTUATORS_DSHOT_NB; i++) {
    if (actuators_dshot_values[i].activated) {
      const DshotTelemetry *dtelem = dshotGetTelemetry(actuators_dshot_private[i].driver, actuators_dshot_private[i].channel);
      rpm_list[i] = dtelem->rpm;
    }
  }
  AbiSendMsgRPM(RPM_DSHOT_ID, rpm_list, ACTUATORS_DSHOT_NB);
}
