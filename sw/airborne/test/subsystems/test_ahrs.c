/*
 * Copyright (C) 2011 The Paparazzi Team
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

#include <inttypes.h>

/* PERIODIC_C_MAIN is defined before generated/periodic_telemetry.h
 * in order to implement telemetry_mode_Main_*
 */
#define PERIODIC_C_MAIN

#include "generated/periodic_telemetry.h"

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/i2c.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/telemetry.h"

#include "subsystems/imu.h"
#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_aligner.h"


static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);
static inline void main_report(void);

static inline void on_gyro_event(void);
static inline void on_accel_event(void);
static inline void on_mag_event(void);

int main(void)
{
  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
    main_event_task();
  }
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  imu_init();
  ahrs_aligner_init();
  ahrs_init();

  mcu_int_enable();
}

static inline void main_periodic_task(void)
{
  if (sys_time.nb_sec > 1) {
    imu_periodic();
  }
  RunOnceEvery(10, { LED_PERIODIC();});
  main_report();
}

static inline void main_event_task(void)
{
  ImuEvent(on_gyro_event, on_accel_event, on_mag_event);
}

static inline void on_gyro_event(void)
{
  // timestamp in usec when last callback was received
  static uint32_t last_ts = 0;
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();
  // dt between this and last callback in seconds
  float dt = (float)(now_ts - last_ts) / 1e6;
  last_ts = now_ts;

  imu_scale_gyro(&imu);
  if (ahrs.status == AHRS_UNINIT) {
    ahrs_aligner_run();
    if (ahrs_aligner.status == AHRS_ALIGNER_LOCKED) {
      ahrs_align();
    }
  } else {
    ahrs_propagate(dt);
  }
}

static inline void on_accel_event(void)
{
  // timestamp in usec when last callback was received
  static uint32_t last_ts = 0;
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();
  // dt between this and last callback in seconds
  float dt = (float)(now_ts - last_ts) / 1e6;
  last_ts = now_ts;

  imu_scale_accel(&imu);
  if (ahrs.status != AHRS_UNINIT) {
    ahrs_update_accel(dt);
  }
}

static inline void on_mag_event(void)
{
  // timestamp in usec when last callback was received
  static uint32_t last_ts = 0;
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();
  // dt between this and last callback in seconds
  float dt = (float)(now_ts - last_ts) / 1e6;
  last_ts = now_ts;

  imu_scale_mag(&imu);
  if (ahrs.status == AHRS_RUNNING) {
    ahrs_update_mag(dt);
  }
}


static inline void main_report(void)
{
  RunOnceEvery(512, DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM));

  periodic_telemetry_send_Main(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
}
