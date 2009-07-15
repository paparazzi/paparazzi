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
//#include "booz2_servos_direct_hw.h"
//#include "booz2_control_surfaces.h"
#include "radio_control.h"


#include "booz2_imu.h"
#include "booz2_analog_baro.h"
#include "booz2_battery.h"

#include "booz2_fms.h"
#include "booz2_autopilot.h"
#include "booz2_stabilization_rate.h"
#include "booz2_stabilization_attitude.h"

#include "booz2_gps.h"
#include "booz2_guidance_h.h"
#include "booz2_guidance_v.h"

#include "booz_ahrs_aligner.h"
#include "booz_ahrs.h"
#include "booz2_ins.h"

#include "booz2_main.h"

static inline void on_gyro_accel_event( void );
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
  uart0_init();
  i2c0_init();
  actuators_init();
#if defined USE_BOOZ2_SERVOS_DIRECT
  booz2_servos_direct_init();
#endif
  // FIXME move that in radio_control_ppm initialisation
  //#if defined RADIO_CONTROL_TYPE && RADIO_CONTROL_TYPE == RADIO_CONTROL_PPM
  ppm_init();
  //#endif
  radio_control_init();

  booz2_analog_init();
  booz2_analog_baro_init();
  booz2_battery_init();
  booz2_imu_impl_init();
  booz2_imu_init();
#ifdef USE_AMI601
  i2c1_init();
  //  ami601_init();
#endif

  booz_fms_init();
  booz2_autopilot_init();
  booz2_nav_init();
  booz2_guidance_h_init();
  booz2_guidance_v_init();
  booz2_stabilization_rate_init();
  booz2_stabilization_attitude_init();

  booz_ahrs_aligner_init();
  booz_ahrs_init();

  booz_ins_init();

  uart1_init();
#ifdef USE_GPS
  booz2_gps_init();
#endif

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
      radio_control_periodic_task();			                \
      if (rc_status != RC_OK)						\
	booz2_autopilot_set_mode(BOOZ2_AP_MODE_FAILSAFE);		\
    },									\
    {									\
      Booz2TelemetryPeriodic();						\
    },									\
    {									\
      booz_fms_periodic();						\
    },									\
    {									\
      /*BoozControlSurfacesSetFromCommands();*/				\
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

  //  RadioControlEvent(booz2_autopilot_on_rc_frame);
  RadioControlEventCheckAndHandle(booz2_autopilot_on_rc_event);

  Booz2ImuEvent(on_gyro_accel_event, on_mag_event);
  
  Booz2AnalogBaroEvent(on_baro_event);
 
#ifdef USE_GPS
  Booz2GpsEvent(on_gps_event);
#endif

}


static inline void on_gyro_accel_event( void ) {

  //  LED_TOGGLE(7);
  // 480 ???
  Booz2ImuScaleGyro();
  Booz2ImuScaleAccel();

  if (booz_ahrs.status == BOOZ_AHRS_UNINIT) {
    // 150
    booz_ahrs_aligner_run();
    if (booz_ahrs_aligner.status == BOOZ_AHRS_ALIGNER_LOCKED)
      booz_ahrs_align();
  }
  else {
    //    LED_ON(7);
    booz_ahrs_propagate();
    //    booz2_filter_attitude_update();
    
    //    LED_OFF(7);
    booz_ins_propagate();
  }
}

static inline void on_baro_event( void ) {
  booz_ins_update_baro();
}


static inline void on_gps_event(void) {
  
  booz_ins_update_gps();
  
}

static inline void on_mag_event(void) {

  Booz2ImuScaleMag();

}
