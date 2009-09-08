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

#include "downlink.h"
#include "booz2_telemetry.h"
#include "datalink.h"

#include "booz2_commands.h"
#include "booz_actuators.h"
#include "booz_radio_control.h"

#include "booz_imu.h"
#include "booz2_gps.h"

#include "booz2_analog_baro.h"
#include "booz2_battery.h"

#include "booz_fms.h"
#include "booz2_autopilot.h"

#include "booz_stabilization.h"
#include "booz_guidance.h"

#include "booz_ahrs.h"
#include "booz2_ins.h"

#ifdef USE_CAM
#include "booz2_cam.h"
#endif

#ifdef BOOZ2_SONAR
#include "booz2_sonar.h"
#endif

#include "booz2_main.h"

#ifdef SITL
#include "nps_autopilot_booz.h"
#endif

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

uint32_t init_done_time;

STATIC_INLINE void booz2_main_init( void ) {

  hw_init();
  sys_time_init();

  actuators_init();
  radio_control_init();

  booz2_analog_init();
  booz2_analog_baro_init();
  booz2_battery_init();
  booz_imu_init();

  booz_fms_init();
  booz2_autopilot_init();
  booz2_nav_init();
  booz2_guidance_h_init();
  booz2_guidance_v_init();
  booz_stabilization_init();

  booz_ahrs_aligner_init();
  booz_ahrs_init();
  booz_ahrs_init_accel_rb();

  booz_ins_init();

#ifdef USE_GPS
  booz2_gps_init();
#endif

#ifdef USE_CAM
  booz2_cam_init();
#endif

#ifdef BOOZ2_SONAR
  booz2_sonar_init();
#endif

  int_enable();

  init_done_time = T0TC;

}


STATIC_INLINE void booz2_main_periodic( void ) {
  //  t0 = T0TC;

  booz_imu_periodic();
#ifdef BOOZ_START_DELAY
  if ((uint32_t)(T0TC-init_done_time) < SYS_TICS_OF_USEC((uint32_t)(BOOZ_START_DELAY*1e6))) return;
#endif

  /* run control loops */
  booz2_autopilot_periodic();
  /* set actuators     */
  actuators_set(booz2_autopilot_motors_on);

  PeriodicPrescaleBy10(                             \
    {                                               \
      radio_control_periodic();						\
      if (radio_control.status != RADIO_CONTROL_OK && booz2_autopilot_mode != BOOZ2_AP_MODE_KILL && booz2_autopilot_mode != BOOZ2_AP_MODE_NAV)\
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

#ifdef USE_CAM
  RunOnceEvery(50,booz2_cam_periodic());
#endif

#ifdef BOOZ2_SONAR
  booz2_analog_periodic();
#endif

  //  t1 = T0TC;
  //  diff = t1 - t0;
  //  RunOnceEvery(100, {DOWNLINK_SEND_TIME(&diff);});
  //  t0 = t1;

}

STATIC_INLINE void booz2_main_event( void ) {

  DatalinkEvent();

  RadioControlEvent(booz2_autopilot_on_rc_frame);

  BoozImuEvent(on_gyro_accel_event, on_mag_event);

  Booz2AnalogBaroEvent(on_baro_event);

#ifdef USE_GPS
  Booz2GpsEvent(on_gps_event);
#endif

#ifdef BOOZ_FAILSAFE_GROUND_DETECT
  BoozDetectGroundEvent();
#endif

}

static inline void on_gyro_accel_event( void ) {

  BoozImuScaleGyro();
  BoozImuScaleAccel();

  booz_ahrs_store_accel();

  if (booz_ahrs.status == BOOZ_AHRS_UNINIT) {
    booz_ahrs_aligner_run();
    if (booz_ahrs_aligner.status == BOOZ_AHRS_ALIGNER_LOCKED)
      booz_ahrs_align();
  }
  else {
    booz_ahrs_propagate();
#ifdef USE_AHRS_LKF
    RunOnceEvery(50, booz_ahrs_update_accel());
#endif
    //    booz2_filter_attitude_update();
#ifdef SITL
    if (nps_bypass_ahrs) {
        sim_overwrite_ahrs();
    }
#endif
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
  BoozImuScaleMag();
#ifdef USE_AHRS_LKF
  if (booz_ahrs.status == BOOZ_AHRS_RUNNING) {
	  RunOnceEvery(10, booz_ahrs_update_mag());
  }
#endif
}
