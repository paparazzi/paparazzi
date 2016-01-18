/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

/*
 *
 * test baro onboard
 *
 */

/* ChibiOS includes */
#include "ch.h"

#include BOARD_CONFIG

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#define DATALINK_C
#include "subsystems/datalink/downlink.h"

#include "subsystems/sensors/baro.h"

#define ABI_C
#include "subsystems/abi.h"

#include "test_baro_board_imu.h"

static inline void main_init(void);

#ifndef BARO_PERIODIC_FREQUENCY
#define BARO_PERIODIC_FREQUENCY 50
#endif
PRINT_CONFIG_VAR(BARO_PERIODIC_FREQUENCY)

#ifdef BARO_LED
PRINT_CONFIG_VAR(BARO_LED)
#endif

/** ABI bindings
 */
#ifndef BARO_ABS_ID
#define BARO_ABS_ID ABI_BROADCAST
#endif
static abi_event pressure_abs_ev;

/*
 * Red LEDs blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThdBlinker, 128);
static void ThdBlinker(void *arg)
{
  (void) arg;
  chRegSetThreadName("blinker");
  while (TRUE) {
#ifdef SYS_TIME_LED
    LED_TOGGLE(SYS_TIME_LED);
#endif
    DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
    uint32_t sec = sys_time.nb_sec;
    DOWNLINK_SEND_TIME(DefaultChannel, DefaultDevice, &sec);
    sys_time_ssleep(1);
  }
}

int main(void)
{
  main_init();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThdBlinker, sizeof(waThdBlinker), NORMALPRIO, ThdBlinker,
      NULL);

  while (1) {
    baro_periodic();
    sys_time_msleep(1);
  }

  return 0;
}

static void pressure_abs_cb(uint8_t __attribute__((unused)) sender_id,
    float pressure)
{
  float p = pressure;
  float foo = 42.;
  DOWNLINK_SEND_BARO_RAW(DefaultChannel, DefaultDevice, &p, &foo);
}

static inline void main_init(void)
{
  mcu_init();
  downlink_init();
  baro_init();

  AbiBindMsgBARO_ABS(BARO_ABS_ID, &pressure_abs_ev, pressure_abs_cb);
}
