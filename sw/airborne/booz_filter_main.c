/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
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

#include "booz_filter_main.h"

#include "std.h"
#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"

#include "booz_imu.h"
#include "booz_still_detection.h"
#include "booz_ahrs.h"

#include "gps.h"
#include "booz_ins.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"
#include "booz_filter_telemetry.h"

#include "booz_link_mcu.h"

static inline void on_imu_accel( void );
static inline void on_imu_gyro( void );
static inline void on_imu_mag( void );
static inline void on_imu_baro( void );
static inline void on_gps( void );

#ifndef SITL
int main( void ) {
  { uint32_t foo = 0; while (foo < 1e5) foo++;}

  booz_filter_main_init();

  while (1) {
    if (sys_time_periodic())
      booz_filter_main_periodic_task();
    booz_filter_main_event_task();
  }
  return 0;
}
#endif

STATIC_INLINE void booz_filter_main_init( void ) {

  hw_init();

  sys_time_init();
  //FIXME
#ifndef SITL
  uart0_init_tx();
  uart1_init_tx();
#endif
  booz_imu_init();

  booz_still_detection_init();

  booz_ahrs_init();

  booz_ins_init();

  booz_link_mcu_init();

  int_enable();

}

// static uint32_t t0, t1, diff;

STATIC_INLINE void booz_filter_main_event_task( void ) {
  /* check if measurements are available */
  BoozImuEvent(on_imu_accel, on_imu_gyro, on_imu_mag, on_imu_baro);
  
  GpsEventCheckAndHandle(on_gps, FALSE);
}

STATIC_INLINE void booz_filter_main_periodic_task( void ) {

  booz_imu_periodic();
  RunOnceEvery(4, {booz_filter_telemetry_periodic_task();})

}

static inline void on_imu_accel( void ) {
  if (booz_ahrs_status == BOOZ_AHRS_STATUS_RUNNING) {
    RunOnceEvery(4, {booz_ahrs_update_accel(imu_accel);}); 
  }
  if (booz_ins_status == BOOZ_INS_STATUS_RUNNING) {
    booz_ins_predict(imu_accel); 
  }
}

static inline void on_imu_gyro( void ) {

  switch (booz_ahrs_status) {

  case BOOZ_AHRS_STATUS_UNINIT :
    booz_still_detection_run();
    if (booz_still_detection_status == BSD_STATUS_LOCKED) {
      booz_ahrs_start(booz_still_detection_accel, 
		      booz_still_detection_gyro, 
		      booz_still_detection_mag);
      booz_ins_start(booz_still_detection_accel, 
		     booz_still_detection_pressure);
    }
    break;

  case BOOZ_AHRS_STATUS_RUNNING :
    // t0 = T0TC;
    booz_ahrs_predict(imu_gyro);
    // t1 = T0TC;
    // diff = t1 - t0;
    // DOWNLINK_SEND_TIME(&diff);
    break;

  }
  //  DOWNLINK_SEND_IMU_GYRO(&imu_gyro[AXIS_P], &imu_gyro[AXIS_Q], &imu_gyro[AXIS_R]);
  booz_link_mcu_send();
}

static inline void on_imu_mag( void ) {
  //  DOWNLINK_SEND_IMU_MAG(&imu_mag[AXIS_X], &imu_mag[AXIS_Y], &imu_mag[AXIS_Z]);
  if (booz_ahrs_status == BOOZ_AHRS_STATUS_RUNNING) {
    booz_ahrs_update_mag(imu_mag);
  }
}

static inline void on_imu_baro( void ) {
  //  DOWNLINK_SEND_IMU_PRESSURE(&imu_pressure);
  booz_ins_update_pressure(imu_pressure);
}

static inline void on_gps( void ) {


}
