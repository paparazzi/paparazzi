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

#include BOARD_CONFIG

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "modules/datalink/downlink.h"
#include "modules/datalink/pprz_dl.h"

#include "modules/sensors/baro.h"

#define ABI_C
#include "modules/core/abi.h"

#include "test_baro_board_imu.h"

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);



__attribute__((weak)) void test_baro_board_imu_init(void)
{
  /* Optionally, to be overriden by board specific code */
}


__attribute__((weak)) void test_baro_board_imu_periodic_task(void)
{
  /* Optionally, to be overriden by board specific code */
}


__attribute__((weak)) void test_baro_board_imu_event_task(void)
{
  /* Optionally, to be overriden by board specific code */
}

/* BARO_PERIODIC_FREQUENCY is defined in the shared/baro_board.makefile and defaults to 50Hz */

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

tid_t baro_tid;

int main(void)
{
  main_init();

  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
    if (sys_time_check_and_ack_timer(baro_tid)) {
      baro_periodic();
    }
    main_event_task();
  }

  return 0;
}

static void pressure_abs_cb(uint8_t __attribute__((unused)) sender_id, __attribute__((unused)) uint32_t stamp, float pressure)
{
  float p = pressure;
  float foo = 42.;
  DOWNLINK_SEND_BARO_RAW(DefaultChannel, DefaultDevice, &p, &foo);
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  datalink_init();
  downlink_init();
  pprz_dl_init();
  test_baro_board_imu_init();
  baro_init();

  baro_tid = sys_time_register_timer(1. / BARO_PERIODIC_FREQUENCY, NULL);

  AbiBindMsgBARO_ABS(BARO_ABS_ID, &pressure_abs_ev, pressure_abs_cb);
}

static inline void main_periodic_task(void)
{
  LED_PERIODIC();
  RunOnceEvery(256, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);});
  test_baro_board_imu_periodic_task();
}

static inline void main_event_task(void)
{
  mcu_event();
  test_baro_board_imu_event_task();
  BaroEvent();
}
