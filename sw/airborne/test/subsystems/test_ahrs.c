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
  downlink_init();

  mcu_int_enable();
}

static inline void main_periodic_task(void)
{
  if (sys_time.nb_sec > 1) {
    imu_periodic();
  }
  RunOnceEvery(10, { LED_PERIODIC();});

  RunOnceEvery(20, main_report());
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

  PeriodicPrescaleBy10( {
    DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice,
                                &imu.accel_unscaled.x,
                                &imu.accel_unscaled.y,
                                &imu.accel_unscaled.z);
  }, {
    DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice,
                               &imu.gyro_unscaled.p,
                               &imu.gyro_unscaled.q,
                               &imu.gyro_unscaled.r);
  }, {
    DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice,
                              &imu.mag_unscaled.x,
                              &imu.mag_unscaled.y,
                              &imu.mag_unscaled.z);
  }, {
    DOWNLINK_SEND_IMU_ACCEL_SCALED(DefaultChannel, DefaultDevice,
                                   &imu.accel.x,
                                   &imu.accel.y,
                                   &imu.accel.z);
  }, {
    DOWNLINK_SEND_IMU_GYRO_SCALED(DefaultChannel, DefaultDevice,
                                  &imu.gyro.p,
                                  &imu.gyro.q,
                                  &imu.gyro.r);
  },

  {
    DOWNLINK_SEND_IMU_MAG_SCALED(DefaultChannel, DefaultDevice,
                                 &imu.mag.x,
                                 &imu.mag.y,
                                 &imu.mag.z);
  },

  {
    DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
  }, {
#if USE_I2C2
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
    const uint8_t _bus2 = 2;
    DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, DefaultDevice,
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
#endif
  }, {
#ifdef AHRS_FLOAT
    struct FloatEulers ltp_to_imu_euler;
    float_eulers_of_quat(&ltp_to_imu_euler, &ahrs_impl.ltp_to_imu_quat);
    struct Int32Eulers euler_i;
    EULERS_BFP_OF_REAL(euler_i, ltp_to_imu_euler);
    struct Int32Eulers* eulers_body = stateGetNedToBodyEulers_i();
    DOWNLINK_SEND_AHRS_EULER_INT(DefaultChannel, DefaultDevice,
                                 &euler_i.phi,
                                 &euler_i.theta,
                                 &euler_i.psi,
                                 &(eulers_body->phi),
                                 &(eulers_body->theta),
                                 &(eulers_body->psi));
#else
    struct Int32Eulers ltp_to_imu_euler;
    int32_eulers_of_quat(&ltp_to_imu_euler, &ahrs_impl.ltp_to_imu_quat);
    struct Int32Eulers* eulers = stateGetNedToBodyEulers_i();
    DOWNLINK_SEND_AHRS_EULER_INT(DefaultChannel, DefaultDevice,
                                 &ltp_to_imu_euler.phi,
                                 &ltp_to_imu_euler.theta,
                                 &ltp_to_imu_euler.psi,
                                 &(eulers->phi),
                                 &(eulers->theta),
                                 &(eulers->psi));
#endif
  }, {
#ifndef AHRS_FLOAT
    DOWNLINK_SEND_AHRS_GYRO_BIAS_INT(DefaultChannel, DefaultDevice,
                                     &ahrs_impl.gyro_bias.p,
                                     &ahrs_impl.gyro_bias.q,
                                     &ahrs_impl.gyro_bias.r);
#endif
  });
}
