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
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro);
static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel);
static void mag_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct Int32Vect3 *mag);

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);

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

  mcu_int_enable();

  downlink_init();

  AbiBindMsgIMU_GYRO_INT32(ABI_BROADCAST, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(ABI_BROADCAST, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_INT32(ABI_BROADCAST, &mag_ev, mag_cb);
}

static inline void led_toggle(void)
{

#ifdef BOARD_LISA_L
  LED_TOGGLE(7);
#endif
}

static inline void main_periodic_task(void)
{
  RunOnceEvery(100, {
    led_toggle();
    DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
  });

#if USE_I2C1
  RunOnceEvery(111, {
      uint16_t i2c1_wd_reset_cnt          = i2c1.errors->wd_reset_cnt;
      uint16_t i2c1_queue_full_cnt        = i2c1.errors->queue_full_cnt;
      uint16_t i2c1_ack_fail_cnt          = i2c1.errors->ack_fail_cnt;
      uint16_t i2c1_miss_start_stop_cnt   = i2c1.errors->miss_start_stop_cnt;
      uint16_t i2c1_arb_lost_cnt          = i2c1.errors->arb_lost_cnt;
      uint16_t i2c1_over_under_cnt        = i2c1.errors->over_under_cnt;
      uint16_t i2c1_pec_recep_cnt         = i2c1.errors->pec_recep_cnt;
      uint16_t i2c1_timeout_tlow_cnt      = i2c1.errors->timeout_tlow_cnt;
      uint16_t i2c1_smbus_alert_cnt       = i2c1.errors->smbus_alert_cnt;
      uint16_t i2c1_unexpected_event_cnt  = i2c1.errors->unexpected_event_cnt;
      uint32_t i2c1_last_unexpected_event = i2c1.errors->last_unexpected_event;
      uint8_t _bus1 = 1;
      DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, DefaultDevice,
                               &i2c1_wd_reset_cnt,
                               &i2c1_queue_full_cnt,
                               &i2c1_ack_fail_cnt,
                               &i2c1_miss_start_stop_cnt,
                               &i2c1_arb_lost_cnt,
                               &i2c1_over_under_cnt,
                               &i2c1_pec_recep_cnt,
                               &i2c1_timeout_tlow_cnt,
                               &i2c1_smbus_alert_cnt,
                               &i2c1_unexpected_event_cnt,
                               &i2c1_last_unexpected_event,
                               &_bus1);
    });
#endif
#if USE_I2C2
  RunOnceEvery(111, {
      uint16_t i2c2_wd_reset_cnt          = i2c2.errors->wd_reset_cnt;
      uint16_t i2c2_queue_full_cnt        = i2c2.errors->queue_full_cnt;
      uint16_t i2c2_ack_fail_cnt          = i2c2.errors->ack_fail_cnt;
      uint16_t i2c2_miss_start_stop_cnt   = i2c2.errors->miss_start_stop_cnt;
      uint16_t i2c2_arb_lost_cnt          = i2c2.errors->arb_lost_cnt;
      uint16_t i2c2_over_under_cnt        = i2c2.errors->over_under_cnt;
      uint16_t i2c2_pec_recep_cnt         = i2c2.errors->pec_recep_cnt;
      uint16_t i2c2_timeout_tlow_cnt      = i2c2.errors->timeout_tlow_cnt;
      uint16_t i2c2_smbus_alert_cnt       = i2c2.errors->smbus_alert_cnt;
      uint16_t i2c2_unexpected_event_cnt  = i2c2.errors->unexpected_event_cnt;
      uint32_t i2c2_last_unexpected_event = i2c2.errors->last_unexpected_event;
      uint8_t _bus2 = 2;
      DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, DefaultDevice,
                               &i2c2_wd_reset_cnt,
                               &i2c2_queue_full_cnt,
                               &i2c2_ack_fail_cnt,
                               &i2c2_miss_start_stop_cnt,
                               &i2c2_arb_lost_cnt,
                               &i2c2_over_under_cnt,
                               &i2c2_pec_recep_cnt,
                               &i2c2_timeout_tlow_cnt,
                               &i2c2_smbus_alert_cnt,
                               &i2c2_unexpected_event_cnt,
                               &i2c2_last_unexpected_event,
                               &_bus2);
    });
#endif

  if (sys_time.nb_sec > 1) { imu_periodic(); }
  RunOnceEvery(10, { LED_PERIODIC();});
}

static inline void main_event_task(void)
{
  mcu_event();
  ImuEvent();
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
#if USE_LED_3
  RunOnceEvery(50, LED_TOGGLE(3));
#endif
  static uint8_t cnt;
  cnt++;
  if (cnt > 15) { cnt = 0; }
  if (cnt == 0) {
    DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice,
                                &imu.accel_unscaled.x,
                                &imu.accel_unscaled.y,
                                &imu.accel_unscaled.z);
  } else if (cnt == 7) {
    DOWNLINK_SEND_IMU_ACCEL_SCALED(DefaultChannel, DefaultDevice,
                                   &accel->x,
                                   &accel->y,
                                   &accel->z);
  }
}

static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro)
{
#if USE_LED_2
  RunOnceEvery(50, LED_TOGGLE(2));
#endif
  static uint8_t cnt;
  cnt++;
  if (cnt > 15) { cnt = 0; }

  if (cnt == 0) {
    DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice,
                               &imu.gyro_unscaled.p,
                               &imu.gyro_unscaled.q,
                               &imu.gyro_unscaled.r);
  } else if (cnt == 7) {
    DOWNLINK_SEND_IMU_GYRO_SCALED(DefaultChannel, DefaultDevice,
                                  &gyro->p,
                                  &gyro->q,
                                  &gyro->r);
  }
}


static void mag_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct Int32Vect3 *mag)
{
  static uint8_t cnt;
  cnt++;
  if (cnt > 10) { cnt = 0; }

  if (cnt == 0) {
    DOWNLINK_SEND_IMU_MAG_SCALED(DefaultChannel, DefaultDevice,
                                 &mag->x,
                                 &mag->y,
                                 &mag->z);
  } else if (cnt == 5) {
    DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice,
                              &imu.mag_unscaled.x,
                              &imu.mag_unscaled.y,
                              &imu.mag_unscaled.z);
  }
}
