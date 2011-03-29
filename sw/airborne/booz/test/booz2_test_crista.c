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
#include "mcu.h"
#include "sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#include "subsystems/imu.h"

#include "interrupt_hw.h"


static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void on_imu_event(void);

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
  mcu_init();
  sys_time_init();
  led_init();

/*   LED_ON(4); */
/*   LED_ON(5); */
/*   LED_ON(6); */
/*   LED_ON(7); */

  uart0_init();
  imu_impl_init();
  imu_init();

  mcu_int_enable();
}

static inline void main_periodic_task( void ) {
  //#if 0
  RunOnceEvery(100, {
      //    LED_TOGGLE(7);
    DOWNLINK_SEND_ALIVE(16, MD5SUM);
  });
  //  uint16_t foo = ami601_status;
  //  DOWNLINK_SEND_BOOT(&foo);
  //#endif
  //  if (cpu_time_sec > 2)

  imu_periodic();
}

static inline void main_event_task( void ) {

  ImuEvent(on_imu_event);

}

static inline void on_imu_event(void) {
  ImuScaleGyro();
  ImuScaleAccel();

  //  LED_TOGGLE(6);
  static uint8_t cnt;
  cnt++;
  if (cnt > 15) cnt = 0;

  if (cnt == 0) {
    DOWNLINK_SEND_IMU_GYRO_RAW(&imu_gyro_unscaled.x,
                   &imu_gyro_unscaled.y,
                   &imu_gyro_unscaled.z);

    DOWNLINK_SEND_IMU_ACCEL_RAW(&imu_accel_unscaled.x,
                &imu_accel_unscaled.y,
                &imu_accel_unscaled.z);
  }
  else if (cnt == 7) {
    DOWNLINK_SEND_IMU_GYRO_SCALED(&imu_gyro.x,
                 &imu_gyro.y,
                 &imu_gyro.z);

    DOWNLINK_SEND_IMU_ACCEL_SCALED(&imu_accel.x,
                  &imu_accel.y,
                  &imu_accel.z);
  }
}
