/*
 * Copyright (C) 2008-2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file firmwares/rotorcraft/main.c
 *
 * Rotorcraft main loop.
 */

#define MODULES_C

#include <inttypes.h>
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "subsystems/datalink/downlink.h"
#include "firmwares/rotorcraft/telemetry.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/settings.h"
#include "subsystems/datalink/xbee.h"

#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#if USE_MOTOR_MIXING
#include "subsystems/actuators/motor_mixing.h"
#endif

#include "subsystems/imu.h"
#include "subsystems/gps.h"

#include "subsystems/sensors/baro.h"
#include "baro_board.h"

#include "subsystems/electrical.h"

#include "firmwares/rotorcraft/autopilot.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance.h"

#include "subsystems/ahrs.h"
#include "subsystems/ins.h"

#include "state.h"

#include "firmwares/rotorcraft/main.h"

#ifdef SITL
#include "nps_autopilot_rotorcraft.h"
#endif

#include "generated/modules.h"

#ifndef BARO_PERIODIC_FREQUENCY
#define BARO_PERIODIC_FREQUENCY 50
#endif

static inline void on_gyro_event( void );
static inline void on_accel_event( void );
static inline void on_baro_abs_event( void );
static inline void on_baro_dif_event( void );
static inline void on_gps_event( void );
static inline void on_mag_event( void );


tid_t main_periodic_tid; ///< id for main_periodic() timer
tid_t failsafe_tid;      ///< id for failsafe_check() timer
tid_t radio_control_tid; ///< id for radio_control_periodic_task() timer
tid_t electrical_tid;    ///< id for electrical_periodic() timer
tid_t baro_tid;          ///< id for baro_periodic() timer
tid_t telemetry_tid;     ///< id for telemetry_periodic() timer

#ifndef SITL
int main( void ) {
  main_init();

  while(1) {
    handle_periodic_tasks();
    main_event();
  }
  return 0;
}
#endif /* SITL */

STATIC_INLINE void main_init( void ) {

  mcu_init();

  electrical_init();

  stateInit();

  actuators_init();
#if USE_MOTOR_MIXING
  motor_mixing_init();
#endif

  radio_control_init();

  baro_init();
  imu_init();
  autopilot_init();
  nav_init();
  guidance_h_init();
  guidance_v_init();
  stabilization_init();

  ahrs_aligner_init();
  ahrs_init();

  ins_init();

#if USE_GPS
  gps_init();
#endif

  modules_init();

  settings_init();

  mcu_int_enable();

#if DATALINK == XBEE
  xbee_init();
#endif

  // register the timers for the periodic functions
  main_periodic_tid = sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  radio_control_tid = sys_time_register_timer((1./60.), NULL);
  failsafe_tid = sys_time_register_timer(0.05, NULL);
  electrical_tid = sys_time_register_timer(0.1, NULL);
  baro_tid = sys_time_register_timer(1./BARO_PERIODIC_FREQUENCY, NULL);
  telemetry_tid = sys_time_register_timer((1./60.), NULL);
}

STATIC_INLINE void handle_periodic_tasks( void ) {
  if (sys_time_check_and_ack_timer(main_periodic_tid))
    main_periodic();
  if (sys_time_check_and_ack_timer(radio_control_tid))
    radio_control_periodic_task();
  if (sys_time_check_and_ack_timer(failsafe_tid))
    failsafe_check();
  if (sys_time_check_and_ack_timer(electrical_tid))
    electrical_periodic();
  if (sys_time_check_and_ack_timer(baro_tid))
    baro_periodic();
  if (sys_time_check_and_ack_timer(telemetry_tid))
    telemetry_periodic();
}

STATIC_INLINE void main_periodic( void ) {

  imu_periodic();

  /* run control loops */
  autopilot_periodic();
  /* set actuators     */
  //actuators_set(autopilot_motors_on);
  SetActuatorsFromCommands(commands);

  modules_periodic_task();

  if (autopilot_in_flight) {
    RunOnceEvery(PERIODIC_FREQUENCY, { autopilot_flight_time++; datalink_time++; });
  }

  RunOnceEvery(10, LED_PERIODIC());
}

STATIC_INLINE void telemetry_periodic(void) {
  PeriodicSendMain(DefaultChannel,DefaultDevice);
}

STATIC_INLINE void failsafe_check( void ) {
  if (radio_control.status != RC_OK &&
      autopilot_mode != AP_MODE_KILL &&
      autopilot_mode != AP_MODE_NAV)
  {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }

#if USE_GPS
  if (autopilot_mode == AP_MODE_NAV &&
#if NO_GPS_LOST_WITH_RC_VALID
      radio_control.status != RC_OK &&
#endif
      GpsIsLost())
  {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }
#endif
}

STATIC_INLINE void main_event( void ) {

  i2c_event();

  DatalinkEvent();

  if (autopilot_rc) {
    RadioControlEvent(autopilot_on_rc_frame);
  }

  ImuEvent(on_gyro_event, on_accel_event, on_mag_event);

  BaroEvent(on_baro_abs_event, on_baro_dif_event);

#if USE_GPS
  GpsEvent(on_gps_event);
#endif

#ifdef FAILSAFE_GROUND_DETECT
  DetectGroundEvent();
#endif

  modules_event_task();

}

static inline void on_accel_event( void ) {
  ImuScaleAccel(imu);

  if (ahrs.status != AHRS_UNINIT) {
    ahrs_update_accel();
  }
}

static inline void on_gyro_event( void ) {

  ImuScaleGyro(imu);

  if (ahrs.status == AHRS_UNINIT) {
    ahrs_aligner_run();
    if (ahrs_aligner.status == AHRS_ALIGNER_LOCKED)
      ahrs_align();
  }
  else {
    ahrs_propagate();
#ifdef SITL
    if (nps_bypass_ahrs) sim_overwrite_ahrs();
#endif
    ins_propagate();
  }
#ifdef USE_VEHICLE_INTERFACE
  vi_notify_imu_available();
#endif
}

static inline void on_baro_abs_event( void ) {
  ins_update_baro();
#ifdef USE_VEHICLE_INTERFACE
  vi_notify_baro_abs_available();
#endif
}

static inline void on_baro_dif_event( void ) {

}

static inline void on_gps_event(void) {
  ins_update_gps();
#ifdef USE_VEHICLE_INTERFACE
  if (gps.fix == GPS_FIX_3D)
    vi_notify_gps_available();
#endif
}

static inline void on_mag_event(void) {
  ImuScaleMag(imu);

#if USE_MAGNETOMETER
  if (ahrs.status == AHRS_RUNNING) {
    ahrs_update_mag();
  }
#endif

#ifdef USE_VEHICLE_INTERFACE
  vi_notify_mag_available();
#endif
}
