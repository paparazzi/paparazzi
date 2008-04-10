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

#include "std.h"
#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"

#include "booz_imu.h"
#include "booz_still_detection.h"

static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

uint32_t t0, t1, diff;

int main( void ) {
  main_init();
  while (1) {
    if (sys_time_periodic())
      main_periodic();
    main_event();
  }
  return 0;
}


static inline void main_init(void) {
  hw_init();

  sys_time_init();

  uart1_init_tx();

  booz_imu_init();

  booz_still_detection_init();

  int_enable();
}


static inline void main_periodic(void) {
  booz_imu_periodic();
}


static inline void on_imu_accel(void) {
  booz_still_detection_run();
  uint16_t sd_ax = bsd_accel_raw_avg[AXIS_X];
  uint16_t sd_ay = bsd_accel_raw_avg[AXIS_Y];
  uint16_t sd_az = bsd_accel_raw_avg[AXIS_Z];
  DOWNLINK_SEND_IMU_ACCEL_RAW(&sd_ax, &sd_ay, &sd_az);
  DOWNLINK_SEND_IMU_ACCEL(&bsd_accel_raw_var[AXIS_X], &bsd_accel_raw_var[AXIS_Y], &bsd_accel_raw_var[AXIS_Z]);
}

static inline void on_imu_gyro(void) {
}

static inline void on_imu_mag(void) {
}

static inline void on_imu_baro(void) {
}

static inline void main_event(void) {
  BoozImuEvent(on_imu_accel, on_imu_gyro, on_imu_mag, on_imu_baro);
}

