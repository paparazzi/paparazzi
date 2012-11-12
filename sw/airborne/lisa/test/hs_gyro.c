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

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "subsystems/imu.h"


#ifndef MEASURED_SENSOR
#define MEASURED_SENSOR gyro_unscaled.p
#define MEASURED_SENSOR_NB 0
#endif


static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void on_gyro_accel_event(void);
static inline void on_accel_event(void);
static inline void on_mag_event(void);

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic_task();
    main_event_task();
  }
  return 0;
}

static inline void main_init( void ) {

  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  imu_init();

  mcu_int_enable();
}

static inline void main_periodic_task( void ) {
  RunOnceEvery(100, {
      LED_TOGGLE(3);
      DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
    });
  imu_periodic();
  RunOnceEvery(10, { LED_PERIODIC();});
}

static inline void main_event_task( void ) {

  ImuEvent(on_gyro_accel_event, on_accel_event, on_mag_event);

}


static inline void on_accel_event(void) {
}

#define NB_SAMPLES 20

static inline void on_gyro_accel_event(void) {
  ImuScaleGyro();
  ImuScaleAccel();

  LED_TOGGLE(2);

  static uint8_t cnt;
  static int32_t samples[NB_SAMPLES];
  const uint8_t axis = MEASURED_SENSOR_NB;
  cnt++;
  if (cnt > NB_SAMPLES) cnt = 0;
  samples[cnt] = imu.MEASURED_SENSOR;
  if (cnt == NB_SAMPLES-1) {
    DOWNLINK_SEND_IMU_HS_GYRO(DefaultChannel, DefaultDevice, &axis, NB_SAMPLES, samples);
  }

  if (cnt == 10) {
    DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice,
			       &imu.gyro_unscaled.p,
			       &imu.gyro_unscaled.q,
			       &imu.gyro_unscaled.r);
  }


}


static inline void on_mag_event(void) {

}
