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
#include "std.h"
#include "init_hw.h"
#include "can.h"
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
static inline void main_on_overo_link_error(void);

static uint32_t spi_msg_cnt = 0;
static uint16_t spi_errors = 0;

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
  //radio_control_init();
  booz_imu_init();
  overo_link_init();
  bench_sensors_init();
  booz2_commands[COMMAND_ROLL] = 0;
  booz2_commands[COMMAND_YAW] = 0;
}

#define PITCH_MAGIC_NUMBER (121)

static inline void main_periodic( void ) {
  int8_t pitch_out,thrust_out;
  booz_imu_periodic();

  OveroLinkPeriodic(main_on_overo_link_lost)

  RunOnceEvery(10, {LED_PERIODIC(); DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});

  //RunOnceEvery(5, {DOWNLINK_SEND_BETH(DefaultChannel, &bench_sensors.angle_1,
  //  &bench_sensors.angle_2,&bench_sensors.angle_3, &bench_sensors.current);});

  //RunOnceEvery(5, {DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, &overo_link.down.msg.foo,&overo_link.down.msg.bar);});

  read_bench_sensors();

  pitch_out = (int8_t)((0xFF) & overo_link.down.msg.pitch);
  thrust_out = (int8_t)((0xFF) & overo_link.down.msg.thrust);

  Bound(pitch_out,-30,30);
  Bound(thrust_out,0,80);

  overo_link.up.msg.thrust_out = thrust_out;
  overo_link.up.msg.pitch_out = pitch_out;

  booz2_commands[COMMAND_PITCH] = pitch_out;
  booz2_commands[COMMAND_THRUST] = thrust_out;

  //stop the motors if we've lost SPI or CAN link
  //If SPI has had CRC error we don't stop motors
  if ((spi_msg_cnt == 0) || (can_err_flags != 0)) {
    booz2_commands[COMMAND_PITCH] = 0;
    booz2_commands[COMMAND_THRUST] = 0;
    actuators_set(FALSE);
    overo_link.up.msg.can_errs = can_err_flags;
    overo_link.up.msg.pitch_out = PITCH_MAGIC_NUMBER;
  } else {
    actuators_set(TRUE);
  }
}

static inline void main_event( void ) {
  BoozImuEvent(on_gyro_accel_event, on_mag_event);
  OveroLinkEvent(main_on_overo_msg_received,main_on_overo_link_error);
}

static inline void main_on_overo_msg_received(void) {

  overo_link.up.msg.bench_sensor.x = bench_sensors.angle_1;
  overo_link.up.msg.bench_sensor.y = bench_sensors.angle_2;
  overo_link.up.msg.bench_sensor.z = bench_sensors.angle_3;

  overo_link.up.msg.accel.x = booz_imu.accel_unscaled.x;
  overo_link.up.msg.accel.y = booz_imu.accel_unscaled.y;
  overo_link.up.msg.accel.z = booz_imu.accel_unscaled.z;

  overo_link.up.msg.gyro.p = booz_imu.gyro_unscaled.p;
  overo_link.up.msg.gyro.q = booz_imu.gyro_unscaled.q;
  overo_link.up.msg.gyro.r = booz_imu.gyro_unscaled.r;
  
  //can_err_flags (uint16) represents the board number that is not communicating regularly
  //spi_errors (uint16) reflects the number of crc errors on the spi link
  //TODO: if >10% of messages are coming in with crc errors, assume something is really wrong
  //and disable the motors.
  overo_link.up.msg.can_errs = can_err_flags;
  overo_link.up.msg.spi_errs = spi_errors;

  //spi_msg_cnt shows number of spi transfers since last link lost event
  overo_link.up.msg.cnt = spi_msg_cnt++;

}

static inline void main_on_overo_link_lost(void) {
  //actuators_set(FALSE);
  spi_msg_cnt = 0;
}


static inline void on_gyro_accel_event(void) {
  BoozImuScaleGyro(booz_imu);
  BoozImuScaleAccel(booz_imu);

  //LED_TOGGLE(2);
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
  BoozImuScaleMag(booz_imu);
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

static inline void main_on_overo_link_error(void){
  spi_errors++;
}
