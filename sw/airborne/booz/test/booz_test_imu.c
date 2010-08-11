/*
 * $Id$
 *  
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

#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "uart.h"
#include "messages.h"
#include "downlink.h"

#include "booz_imu.h"

#include "interrupt_hw.h"

#include "my_debug_servo.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void on_gyro_accel_event(void);
static inline void on_mag_event(void);

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
    main_event_task();
  }
  return 0;
}

static inline void main_init( void ) {

  hw_init();
  sys_time_init();
  booz_imu_init();

  DEBUG_SERVO1_INIT();
  DEBUG_SERVO2_INIT();


  int_enable();
}

static inline void main_periodic_task( void ) {
  RunOnceEvery(100, {
      LED_TOGGLE(3);
      DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);
    });
#ifdef USE_I2C2
  RunOnceEvery(111, {
      DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, 
			       &i2c2_errors.ack_fail_cnt,
			       &i2c2_errors.miss_start_stop_cnt,
			       &i2c2_errors.arb_lost_cnt,
			       &i2c2_errors.over_under_cnt,
			       &i2c2_errors.pec_recep_cnt,
			       &i2c2_errors.timeout_tlow_cnt,
			       &i2c2_errors.smbus_alert_cnt,
			       &i2c2_errors.unexpected_event_cnt,
			       &i2c2_errors.last_unexpected_event);
    });
#endif
  booz_imu_periodic();
  RunOnceEvery(10, { LED_PERIODIC();});
}

static inline void main_event_task( void ) {

  BoozImuEvent(on_gyro_accel_event, on_mag_event);

}

static inline void on_gyro_accel_event(void) {
  BoozImuScaleGyro(booz_imu);
  BoozImuScaleAccel(booz_imu);

  LED_TOGGLE(2);
  static uint8_t cnt;
  cnt++;
  if (cnt > 15) cnt = 0;

  if (cnt == 0) {
    DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel,
			       &booz_imu.gyro_unscaled.p,
			       &booz_imu.gyro_unscaled.q,
			       &booz_imu.gyro_unscaled.r);
    
    DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel,
				&booz_imu.accel_unscaled.x,
				&booz_imu.accel_unscaled.y,
				&booz_imu.accel_unscaled.z);
  }
  else if (cnt == 7) {
    DOWNLINK_SEND_BOOZ2_GYRO(DefaultChannel,
			     &booz_imu.gyro.p,
			     &booz_imu.gyro.q,
			     &booz_imu.gyro.r);
    
    DOWNLINK_SEND_BOOZ2_ACCEL(DefaultChannel,
			      &booz_imu.accel.x,
			      &booz_imu.accel.y,
			      &booz_imu.accel.z);
  }
}


static inline void on_mag_event(void) {
  BoozImuScaleMag();
  static uint8_t cnt;
  cnt++;
  if (cnt > 1) cnt = 0;

  if (cnt%2) {
    DOWNLINK_SEND_BOOZ2_MAG(DefaultChannel,
			    &booz_imu.mag.x,
			    &booz_imu.mag.y,
			    &booz_imu.mag.z);
  }
  else {
    DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel,
			      &booz_imu.mag_unscaled.x,
			      &booz_imu.mag_unscaled.y,
			      &booz_imu.mag_unscaled.z);
  }
}
