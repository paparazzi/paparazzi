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

#include <inttypes.h>

#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart.h"

#include "messages.h"
#include "downlink.h"
#include "booz2_telemetry.h"
#include "datalink.h"

#include "booz2_commands.h"
#include "i2c.h"
#include ACTUATORS
#include "radio_control.h"


#include "booz2_imu.h"
#include "booz2_analog_baro.h"
#include "booz2_battery.h"

#include "AMI601.h"

#include "booz2_fms.h"
#include "booz2_autopilot.h"
#include "booz2_stabilization_rate.h"
#include "booz2_stabilization_attitude.h"

#include "booz2_gps.h"
#include "booz2_guidance_h.h"
#include "booz2_guidance_v.h"

#include "booz2_filter_aligner.h"
#include "booz2_filter_attitude.h"
#include "booz2_ins.h"

#include "booz2_main.h"

static inline void on_imu_event( void );
static inline void on_baro_event( void );
static inline void on_gps_event( void );
static inline void on_mag_event( void );

uint32_t t0, t1, diff;

#ifndef SITL
int main( void ) {
  booz2_main_init();
  while(1) {
    if (sys_time_periodic())
      booz2_main_periodic();
    booz2_main_event();
  }
  return 0;
}
#endif /* SITL */


STATIC_INLINE void booz2_main_init( void ) {
  hw_init();
  sys_time_init();
  led_init();
  uart0_init_tx();
  i2c_init();
  actuators_init();
  ppm_init();
  radio_control_init();

  booz2_analog_init();
  booz2_analog_baro_init();
  booz2_battery_init();
  booz2_imu_impl_init();
  booz2_imu_init();
  i2c1_init();
  ami601_init();

  booz_fms_init();
  booz2_autopilot_init();
  booz2_nav_init();
  booz2_guidance_h_init();
  booz2_guidance_v_init();
  booz2_stabilization_rate_init();
  booz2_stabilization_attitude_init();

  booz2_filter_aligner_init();
  booz2_ahrs_init();

  booz_ins_init();

  uart1_init_tx();
  booz2_gps_init();

  int_enable();
}


STATIC_INLINE void booz2_main_periodic( void ) {
  //  t0 = T0TC;

  booz2_imu_periodic();
  /* run control loops */
  booz2_autopilot_periodic();
  /* set actuators     */
  SetActuatorsFromCommands(booz2_autopilot_motors_on);

  PeriodicPrescaleBy10(							\
    {						                        \
      radio_control_periodic_task();		                        \
      if (rc_status != RC_OK)						\
	booz2_autopilot_set_mode(BOOZ2_AP_MODE_FAILSAFE);		\
    },									\
    {									\
      Booz2TelemetryPeriodic();						\
    },									\
    {									\
      ami601_read();							\
    },									\
    {									\
      booz_fms_periodic();						\
    },									\
    {},									\
    {},									\
    {},									\
    {},									\
    {},									\
    {}									\
    );									\

  //  t1 = T0TC;
  //  diff = t1 - t0;
  //  RunOnceEvery(100, {DOWNLINK_SEND_TIME(&diff);});
  //  t0 = t1;

}

STATIC_INLINE void booz2_main_event( void ) {

  DatalinkEvent();

  RadioControlEventCheckAndHandle(booz2_autopilot_on_rc_event);

  Booz2ImuEvent(on_imu_event);

  Booz2AnalogBaroEvent(on_baro_event);

  Booz2GpsEvent(on_gps_event);

  AMI601Event(on_mag_event);

}


static inline void on_imu_event( void ) {

  //  LED_TOGGLE(7);
  // 480 ???
  Booz2ImuScaleGyro();
  Booz2ImuScaleAccel();

  if (booz_ahrs.status == BOOZ2_AHRS_UNINIT) {
    // 150
    booz2_filter_aligner_run();
    if (booz2_filter_aligner_status == BOOZ2_FILTER_ALIGNER_LOCKED)
      booz2_ahrs_align();
  }
  else {
    //    LED_ON(7);
    booz2_ahrs_propagate();
    //    booz2_filter_attitude_update();

    //    LED_OFF(7);
    booz_ins_propagate();
  }
}

static inline void on_baro_event( void ) {
  RunOnceEvery(20, {
      DOWNLINK_SEND_ADC_GENERIC(&booz2_analog_baro_offset, &booz2_analog_baro_value_filtered);
    });
  booz_ins_update_baro();
}


static inline void on_gps_event(void) {
 
  booz_ins_update_gps();

}

static inline void on_mag_event(void) {
  booz2_imu_mag_unscaled.x = ami601_val[IMU_MAG_X_CHAN];
  booz2_imu_mag_unscaled.y = ami601_val[IMU_MAG_Y_CHAN];
  booz2_imu_mag_unscaled.z = ami601_val[IMU_MAG_Z_CHAN];

  //  LED_TOGGLE(2);
  Booz2ImuScaleMag();
  ami601_status = AMI601_IDLE;
}
