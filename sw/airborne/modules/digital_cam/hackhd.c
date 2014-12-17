/*
 * Copyright (C) 2014 Gautier Hattenberger
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
 *
 */

/** @file modules/digital_cam/hackhd.c
 *  @brief Digital video/photo recorder HackHD control
 *
 * Provides the control of the HackHD power, start and stop of recording.
 * If you are using firmware >= 1.1.15, it is also possible to take pictures
 * according to the parameter in the config.txt file (on HackHD SD card).
 * It is not possible to have both video and photo at the same time.
 * This driver starts the HackHD in standby mode and trigger the start/stop
 * of recording or take a picture.
 * Minimum time between two pictures is 2 seconds.
 *
 * It is mandatory to configure the control GPIO:
 * @verbatim
 *   <configure name="HACKHD_GPIO" value="GPIOC,GPIO5"/>
 * @endverbatim
 *
 */

#include "modules/digital_cam/hackhd.h"
#include "generated/modules.h"
#include "generated/airframe.h"
#include "mcu_periph/gpio.h"
#include "mcu_periph/sys_time.h"

/** Trigger button is active low */
#define HACKHD_PUSH gpio_clear
#define HACKHD_RELEASE gpio_set

#ifndef HACKHD_GPIO
#error HACKHD: Please specify at least a HACKHD_GPIO (e.g. <define name="HACKHD_GPIO" value="GPIOC,GPIO5"/>)
#endif

static inline uint32_t port_of_gpio(uint32_t port, uint16_t __attribute__((unused)) pin) { return port; }
static inline uint16_t pin_of_gpio(uint32_t __attribute__((unused)) port, uint16_t pin) { return pin; }

/** time in seconds to press the button to power on/off */
#define HACKHD_POWER_DELAY 5.

/** time in seconds to start/stop recording or take a picture */
#define HACKHD_RECORD_DELAY 0.2

/** delay in milli-seconds before logging after a shot
 *  this has been estimated to 1s
 */
#define HACKHD_LOG_DELAY 1000

/** get timer from delay
 *  based on periodic freq from modules.h
 */
#define HACKHD_TIMER_OF_DELAY(_delay) ((uint32_t)(_delay * HACKHD_PERIODIC_FREQ))

/** autoshoot timer delay
 *  based on periodic freq from modules.h
 */
#ifndef HACKHD_AUTOSHOOT_DELAY
#define HACKHD_AUTOSHOOT_DELAY 4.0
#endif
#define HACKHD_AUTOSHOOT_TIMER_OF_DELAY(_delay) ((uint32_t)(_delay * HACKHD_AUTOSHOOT_FREQ))

/** send report */
#if HACKHD_SYNC_SEND

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "state.h"
#include "subsystems/gps.h"

static inline void hackhd_send_shot_position(void)
{
  // angles in decideg
  int16_t phi = DegOfRad(stateGetNedToBodyEulers_f()->phi * 10.0f);
  int16_t theta = DegOfRad(stateGetNedToBodyEulers_f()->theta * 10.0f);
  int16_t psi = DegOfRad(stateGetNedToBodyEulers_f()->psi * 10.0f);
  // course in decideg
  int16_t course = DegOfRad(*stateGetHorizontalSpeedDir_f()) * 10;
  // ground speed in cm/s
  uint16_t speed = (*stateGetHorizontalSpeedNorm_f()) * 10;

  DOWNLINK_SEND_DC_SHOT(DefaultChannel, DefaultDevice,
                        &hackhd.photo_nr,
                        &stateGetPositionLla_i()->lat,
                        &stateGetPositionLla_i()->lon,
                        &stateGetPositionLla_i()->alt,
                        &gps.hmsl,
                        &phi,
                        &theta,
                        &psi,
                        &course,
                        &speed,
                        &gps.tow);
}
#endif

#if HACKHD_LOG
#include "sdLog.h"
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
#include "state.h"
#include "subsystems/gps.h"

static inline void hackhd_log_shot_position(void)
{
  // For unknown reason, the first shot is not taken
  // so we start logging at photo_nr = 1
  if (pprzLogFile.fs != NULL && hackhd.photo_nr > 0) {
    struct FloatEulers att = *stateGetNedToBodyEulers_f();
    struct EnuCoor_f pos = *stateGetPositionEnu_f();
    uint32_t time = get_sys_time_msec();
    sdLogWriteLog(&pprzLogFile, "%d %d %d %d %d %d %d %u\n",
                  hackhd.photo_nr,
                  (int32_t)(DegOfRad(att.phi * 10.0f)),
                  (int32_t)(DegOfRad(att.theta * 10.0f)),
                  (int32_t)(DegOfRad(att.psi * 10.0f)),
                  (int32_t)(pos.x * 100.0f),
                  (int32_t)(pos.y * 100.0f),
                  (int32_t)(pos.z * 100.0f),
                  time);
  }
}
#endif

struct HackHD hackhd;

void hackhd_init(void)
{
  hackhd.status = HACKHD_NONE;
  hackhd.timer = 0;
  hackhd.photo_nr = 0;
  hackhd.autoshoot = 0;
  hackhd.log_delay = 0;

#ifndef SITL
  gpio_setup_output(HACKHD_GPIO);
  // set gpio as open-drain, only possible on stm32f4
  gpio_set_output_options(
    port_of_gpio(HACKHD_GPIO),
    GPIO_OTYPE_OD,
    GPIO_OSPEED_25MHZ,
    pin_of_gpio(HACKHD_GPIO));
  HACKHD_RELEASE(HACKHD_GPIO);
#endif
}

void hackhd_periodic(void)
{
  if (hackhd.timer) {
    hackhd.timer--;
  } else {
    HACKHD_RELEASE(HACKHD_GPIO);
  }
  // test log delay if set
  if (hackhd.log_delay) {
#ifndef SITL
    if (get_sys_time_msec() > hackhd.log_delay) {
#endif
#if HACKHD_LOG
      hackhd_log_shot_position();
#endif
#if HACKHD_SYNC_SEND
      hackhd_send_shot_position();
#endif
      // increment photo
      hackhd.photo_nr++;
      // unset log delay
      hackhd.log_delay = 0;
#ifndef SITL
    }
#endif
  }
}

/* Command the powering and recording */
void hackhd_command(enum hackhd_status cmd)
{
  hackhd.status = cmd;
  switch (cmd) {
    case HACKHD_POWER_ON:
    case HACKHD_POWER_OFF:
      hackhd.timer = HACKHD_TIMER_OF_DELAY(HACKHD_POWER_DELAY);
      HACKHD_PUSH(HACKHD_GPIO);
      break;
    case HACKHD_START_RECORD:
    case HACKHD_STOP_RECORD:
      hackhd.timer = HACKHD_TIMER_OF_DELAY(HACKHD_RECORD_DELAY);
      HACKHD_PUSH(HACKHD_GPIO);
      break;
    case HACKHD_SHOOT:
    case HACKHD_AUTOSHOOT_START:
      hackhd.timer = HACKHD_TIMER_OF_DELAY(HACKHD_RECORD_DELAY);
      HACKHD_PUSH(HACKHD_GPIO);
      hackhd.log_delay = get_sys_time_msec() + HACKHD_LOG_DELAY;
      hackhd.last_shot_pos = *stateGetPositionEnu_f();
      break;
    default:
      break;
  }
}

void hackhd_autoshoot(void)
{
// at least wait a minimum time before two shoots
  if (hackhd.autoshoot) {
    hackhd.autoshoot--;
  } else {
    // test distance if needed
    // or take picture if first of the sequence
#ifdef HACKHD_AUTOSHOOT_DIST
    struct EnuCoor_f pos = *stateGetPositionEnu_f();
    struct FloatVect2 d_pos;
    d_pos.x = pos.x - hackhd.last_shot_pos.x;
    d_pos.y = pos.y - hackhd.last_shot_pos.y;
    if (VECT2_NORM2(d_pos) > (HACKHD_AUTOSHOOT_DIST * HACKHD_AUTOSHOOT_DIST)
        || hackhd.status == HACKHD_AUTOSHOOT_START) {
#endif
      // take a picture
      hackhd_command(HACKHD_SHOOT);
      // reset timer
      hackhd.autoshoot = HACKHD_AUTOSHOOT_TIMER_OF_DELAY(HACKHD_AUTOSHOOT_DELAY);
#ifdef HACKHD_AUTOSHOOT_DIST
    }
#endif
  }
}

void hackhd_autoshoot_start(void)
{
  // start taking a picture immediately
  hackhd.autoshoot = 0;
  hackhd.status = HACKHD_AUTOSHOOT_START;
}

