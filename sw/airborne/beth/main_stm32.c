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
  //radio_control_init();
  booz_imu_init();
  overo_link_init();
  bench_sensors_init();
  //LED_ON(7);
}


static inline void main_periodic( void ) {
  int8_t pitch;
  booz_imu_periodic();

  OveroLinkPeriodic(main_on_overo_link_lost)

  RunOnceEvery(10, {LED_PERIODIC(); DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});

  //RunOnceEvery(5, {DOWNLINK_SEND_BETH(DefaultChannel, &bench_sensors.angle_1,
  //  &bench_sensors.angle_2,&bench_sensors.angle_3, &bench_sensors.current);});

  //RunOnceEvery(5, {DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, &overo_link.down.msg.foo,&overo_link.down.msg.bar);});

  /*Request reception of values from coder boards :
    When configured for I2C, lisa stm32 is master and requests data from the
    beth board slaves.
    When configured for CAN, data is automatically available as CAN reception is
    always ongoing, and new data generates a flag by the IST. */
  read_bench_sensors();

  pitch = (int8_t)((0xFF) & overo_link.down.msg.pitch);
  if (pitch > 10) pitch = 10; else 
   if (pitch < -10) pitch = -10; 

  booz2_commands[COMMAND_PITCH] = pitch;
  booz2_commands[COMMAND_ROLL] = 0;
  booz2_commands[COMMAND_YAW] = 0;
  if ( overo_link.down.msg.thrust < 20) {
    booz2_commands[COMMAND_THRUST] = overo_link.down.msg.thrust;
  } else { 
    booz2_commands[COMMAND_THRUST] = 20;
  }
  if (my_cnt == 0) {
    actuators_set(FALSE);
  } else {
    actuators_set(TRUE);
  }
}


static inline void main_event( void ) {
  BoozImuEvent(on_gyro_accel_event, on_mag_event);
  OveroLinkEvent(main_on_overo_msg_received);

}

static inline void main_on_overo_msg_received(void) {

  overo_link.up.msg.bench_sensor.x = bench_sensors.angle_1;
  overo_link.up.msg.bench_sensor.y = bench_sensors.angle_2;
  overo_link.up.msg.bench_sensor.z = bench_sensors.angle_3;

/*  overo_link.up.msg.accel.x = booz_imu.accel.x;
  overo_link.up.msg.accel.y = booz_imu.accel.y;
  overo_link.up.msg.accel.z = booz_imu.accel.z;

  overo_link.up.msg.gyro.p = booz_imu.gyro.p;
  overo_link.up.msg.gyro.q = booz_imu.gyro.q;
  overo_link.up.msg.gyro.r = booz_imu.gyro.r;*/

  overo_link.up.msg.accel.x = booz_imu.accel_unscaled.x;
  overo_link.up.msg.accel.y = booz_imu.accel_unscaled.y;
  overo_link.up.msg.accel.z = booz_imu.accel_unscaled.z;

  overo_link.up.msg.gyro.p = booz_imu.gyro_unscaled.p;
  overo_link.up.msg.gyro.q = booz_imu.gyro_unscaled.q;
  overo_link.up.msg.gyro.r = booz_imu.gyro_unscaled.r;

  my_cnt=1;
  //actuators_set(TRUE);
}

static inline void main_on_overo_link_lost(void) {
  //actuators_set(FALSE);
  my_cnt = 0;
/* didn't work: */
//  overo_link_arch_prepare_next_transfert();
//  overo_link.status = IDLE;	

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

