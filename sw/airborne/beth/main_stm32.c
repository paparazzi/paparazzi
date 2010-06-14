/*
 * $Id$
 *  
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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


#include BOARD_CONFIG
#include "init_hw.h"
#include "sys_time.h"
#include "downlink.h"
#include "booz/booz2_commands.h"
#include "booz/booz_actuators.h"
//#include "booz/booz_radio_control.h"
#include "booz/booz_imu.h"
#include "lisa/lisa_overo_link.h"
#include "beth/bench_sensors.h"

static inline void main_init( void );
static inline void main_periodic( void );
static inline void main_event( void );

static inline void on_gyro_accel_event(void);
static inline void on_mag_event(void);

static inline void main_on_overo_msg_received(void);
static inline void main_on_overo_link_lost(void);
static inline void main_on_bench_sensors( void );

static int16_t my_cnt;

int main(void) {

  main_init();

  while (1) {
    if (sys_time_periodic())
      main_periodic();
    main_event();
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  actuators_init();
  //  radio_control_init();
  //  booz_imu_init();
  //  overo_link_init();
  bench_sensors_init();
}

static inline void main_periodic( void ) {
  //  booz_imu_periodic();
  actuators_set(FALSE);
  //  OveroLinkPeriodic(main_on_overo_link_lost)
  RunOnceEvery(10, {LED_PERIODIC(); DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});

  read_bench_sensors();
 
}

static inline void main_event( void ) {
  //    BoozImuEvent(on_gyro_accel_event, on_mag_event);
  //    OveroLinkEvent(main_on_overo_msg_received);

  BenchSensorsEvent(main_on_bench_sensors);

}

static inline void main_on_overo_msg_received(void) {
  struct AutopilotMessageBethUp* msg_out = (struct AutopilotMessageBethUp*)overo_link.msg_out;
  msg_out->gyro.x = booz_imu.gyro.p;
  msg_out->gyro.y = booz_imu.gyro.q;
  msg_out->gyro.z = booz_imu.gyro.r;
  msg_out->accel.x = booz_imu.accel.x;
  msg_out->accel.y = booz_imu.accel.y;
  msg_out->accel.z = booz_imu.accel.z;
  msg_out->bench_sensor.x = my_cnt;
  msg_out->bench_sensor.y = my_cnt;
  msg_out->bench_sensor.z = my_cnt;
  my_cnt++;
}

static inline void main_on_overo_link_lost(void) {
  my_cnt = 0;
}



static inline void on_gyro_accel_event(void) {
  BoozImuScaleGyro();
  BoozImuScaleAccel();

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


static inline void main_on_bench_sensors( void ) {
  
  DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, 
			    &bench_sensors.angle_1,
                            &bench_sensors.angle_2);
  
}
