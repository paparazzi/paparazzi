/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#define DATALINK_C
#define ABI_C

/* ChibiOS includes */
#include "ch.h"

#ifdef BOARD_CONFIG
#include BOARD_CONFIG
#endif
#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/i2c.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "subsystems/imu.h"
#include "subsystems/abi.h"

static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev;
static void gyro_cb(uint8_t sender_id __attribute__((unused)),
    uint32_t stamp __attribute__((unused)), struct Int32Rates *gyro);
static void accel_cb(uint8_t sender_id __attribute__((unused)),
    uint32_t stamp __attribute__((unused)), struct Int32Vect3 *accel);
static void mag_cb(uint8_t sender_id __attribute__((unused)),
    uint32_t stamp __attribute__((unused)), struct Int32Vect3 *mag);

static inline void main_init(void);

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
    sys_time_ssleep(1);
  }
}

int main(void)
{
  main_init();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThdBlinker, sizeof(waThdBlinker), NORMALPRIO, ThdBlinker, NULL);

  while (1) {
    imu_periodic();
    ImuEvent();
    sys_time_msleep(1);
  }
  return 0;
}

static inline void main_init(void)
{

  mcu_init();

  imu_init();

  downlink_init();

  AbiBindMsgIMU_GYRO_INT32(ABI_BROADCAST, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(ABI_BROADCAST, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_INT32(ABI_BROADCAST, &mag_ev, mag_cb);
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
    uint32_t stamp __attribute__((unused)), struct Int32Vect3 *accel)
{
#if USE_LED_3
  RunOnceEvery(50, LED_TOGGLE(3));
#endif
  static uint8_t cnt;
  cnt++;
  if (cnt > 15) {
    cnt = 0;
  }
  if (cnt == 0) {
    DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice,
        &imu.accel_unscaled.x, &imu.accel_unscaled.y, &imu.accel_unscaled.z);
  } else if (cnt == 7) {
    DOWNLINK_SEND_IMU_ACCEL_SCALED(DefaultChannel, DefaultDevice, &accel->x,
        &accel->y, &accel->z);
  }
}

static void gyro_cb(uint8_t sender_id __attribute__((unused)),
    uint32_t stamp __attribute__((unused)), struct Int32Rates *gyro)
{
#if USE_LED_2
  RunOnceEvery(50, LED_TOGGLE(2));
#endif
  static uint8_t cnt;
  cnt++;
  if (cnt > 15) {
    cnt = 0;
  }

  if (cnt == 0) {
    DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice,
        &imu.gyro_unscaled.p, &imu.gyro_unscaled.q, &imu.gyro_unscaled.r);
  } else if (cnt == 7) {
    DOWNLINK_SEND_IMU_GYRO_SCALED(DefaultChannel, DefaultDevice, &gyro->p,
        &gyro->q, &gyro->r);
  }
}

static void mag_cb(uint8_t sender_id __attribute__((unused)),
    uint32_t stamp __attribute__((unused)), struct Int32Vect3 *mag)
{
  static uint8_t cnt;
  cnt++;
  if (cnt > 10) {
    cnt = 0;
  }

  if (cnt == 0) {
    DOWNLINK_SEND_IMU_MAG_SCALED(DefaultChannel, DefaultDevice, &mag->x,
        &mag->y, &mag->z);
  } else if (cnt == 5) {
    DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice,
        &imu.mag_unscaled.x, &imu.mag_unscaled.y, &imu.mag_unscaled.z);
  }
}
