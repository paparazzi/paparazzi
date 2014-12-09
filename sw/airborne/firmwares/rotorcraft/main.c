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

#define ABI_C

#include <inttypes.h>
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/uart.h"
#if USE_UDP
#include "mcu_periph/udp.h"
#endif
#include "led.h"

#include "subsystems/datalink/telemetry.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/settings.h"

#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#if USE_MOTOR_MIXING
#include "subsystems/actuators/motor_mixing.h"
#endif

#include "subsystems/imu.h"
#include "subsystems/gps.h"

#if USE_BARO_BOARD
#include "subsystems/sensors/baro.h"
PRINT_CONFIG_MSG_VALUE("USE_BARO_BOARD is TRUE, reading onboard baro: ", BARO_BOARD)
#endif

#include "subsystems/electrical.h"

#include "firmwares/rotorcraft/autopilot.h"

#include "subsystems/radio_control.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance.h"

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "subsystems/ins.h"

#include "state.h"

#include "firmwares/rotorcraft/main.h"

#ifdef SITL
#include "nps_autopilot.h"
#endif

#include "generated/modules.h"
#include "subsystems/abi.h"

#if USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif


/* if PRINT_CONFIG is defined, print some config options */
PRINT_CONFIG_VAR(PERIODIC_FREQUENCY)

/* TELEMETRY_FREQUENCY is defined in generated/periodic_telemetry.h
 * defaults to 60Hz or set by TELEMETRY_FREQUENCY configure option in airframe file
 */
PRINT_CONFIG_VAR(TELEMETRY_FREQUENCY)

/* MODULES_FREQUENCY is defined in generated/modules.h
 * according to main_freq parameter set for modules in airframe file
 */
PRINT_CONFIG_VAR(MODULES_FREQUENCY)

#ifndef BARO_PERIODIC_FREQUENCY
#define BARO_PERIODIC_FREQUENCY 50
#endif
PRINT_CONFIG_VAR(BARO_PERIODIC_FREQUENCY)

#if USE_AHRS && USE_IMU && (defined AHRS_PROPAGATE_FREQUENCY)
#if (AHRS_PROPAGATE_FREQUENCY > PERIODIC_FREQUENCY)
#warning "PERIODIC_FREQUENCY should be least equal or greater than AHRS_PROPAGATE_FREQUENCY"
INFO_VALUE("it is recommended to configure in your airframe PERIODIC_FREQUENCY to at least ",AHRS_PROPAGATE_FREQUENCY)
#endif
#endif

static inline void on_gyro_event( void );
static inline void on_accel_event( void );
static inline void on_gps_event( void );
static inline void on_mag_event( void );


tid_t main_periodic_tid; ///< id for main_periodic() timer
tid_t modules_tid;       ///< id for modules_periodic_task() timer
tid_t failsafe_tid;      ///< id for failsafe_check() timer
tid_t radio_control_tid; ///< id for radio_control_periodic_task() timer
tid_t electrical_tid;    ///< id for electrical_periodic() timer
tid_t telemetry_tid;     ///< id for telemetry_periodic() timer
#if USE_BARO_BOARD
tid_t baro_tid;          ///< id for baro_periodic() timer
#endif

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

#if USE_BARO_BOARD
  baro_init();
#endif
  imu_init();
  ahrs_aligner_init();
  ahrs_init();
  ins_init();

#if USE_GPS
  gps_init();
#endif

  autopilot_init();

  modules_init();

  settings_init();

  mcu_int_enable();

#if DOWNLINK
  downlink_init();
#endif

  // register the timers for the periodic functions
  main_periodic_tid = sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  modules_tid = sys_time_register_timer(1./MODULES_FREQUENCY, NULL);
  radio_control_tid = sys_time_register_timer((1./60.), NULL);
  failsafe_tid = sys_time_register_timer(0.05, NULL);
  electrical_tid = sys_time_register_timer(0.1, NULL);
  telemetry_tid = sys_time_register_timer((1./TELEMETRY_FREQUENCY), NULL);
#if USE_BARO_BOARD
  baro_tid = sys_time_register_timer(1./BARO_PERIODIC_FREQUENCY, NULL);
#endif
}

STATIC_INLINE void handle_periodic_tasks( void ) {
  if (sys_time_check_and_ack_timer(main_periodic_tid))
    main_periodic();
  if (sys_time_check_and_ack_timer(modules_tid))
    modules_periodic_task();
  if (sys_time_check_and_ack_timer(radio_control_tid))
    radio_control_periodic_task();
  if (sys_time_check_and_ack_timer(failsafe_tid))
    failsafe_check();
  if (sys_time_check_and_ack_timer(electrical_tid))
    electrical_periodic();
  if (sys_time_check_and_ack_timer(telemetry_tid))
    telemetry_periodic();
#if USE_BARO_BOARD
  if (sys_time_check_and_ack_timer(baro_tid))
    baro_periodic();
#endif
}

STATIC_INLINE void main_periodic( void ) {

  imu_periodic();

  /* run control loops */
  autopilot_periodic();
  /* set actuators     */
  //actuators_set(autopilot_motors_on);
  SetActuatorsFromCommands(commands, autopilot_mode);

  if (autopilot_in_flight) {
    RunOnceEvery(PERIODIC_FREQUENCY, { autopilot_flight_time++;
#if defined DATALINK || defined SITL
        datalink_time++;
#endif
        });
  }

  RunOnceEvery(10, LED_PERIODIC());
}

STATIC_INLINE void telemetry_periodic(void) {
#if PERIODIC_TELEMETRY
  periodic_telemetry_send_Main(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif
}

/** mode to enter when RC is lost while using a mode with RC input (not AP_MODE_NAV) */
#ifndef RC_LOST_MODE
#define RC_LOST_MODE AP_MODE_FAILSAFE
#endif

STATIC_INLINE void failsafe_check( void ) {
  if (radio_control.status == RC_REALLY_LOST &&
      autopilot_mode != AP_MODE_KILL &&
      autopilot_mode != AP_MODE_HOME &&
      autopilot_mode != AP_MODE_FAILSAFE &&
      autopilot_mode != AP_MODE_NAV)
  {
    autopilot_set_mode(RC_LOST_MODE);
  }

#if FAILSAFE_ON_BAT_CRITICAL
  if (autopilot_mode != AP_MODE_KILL &&
      electrical.bat_critical)
  {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }
#endif

#if USE_GPS
  gps_periodic_check();
  if (autopilot_mode == AP_MODE_NAV &&
      autopilot_motors_on &&
#if NO_GPS_LOST_WITH_RC_VALID
      radio_control.status != RC_OK &&
#endif
      GpsIsLost())
  {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }

  if (autopilot_mode == AP_MODE_HOME &&
      autopilot_motors_on && GpsIsLost())
  {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }
#endif

  autopilot_check_in_flight(autopilot_motors_on);
}

STATIC_INLINE void main_event( void ) {

  i2c_event();

#ifndef SITL
  uart_event();
#endif

#if USE_UDP
  udp_event();
#endif

#if USE_USB_SERIAL
  VCOM_event();
#endif

  DatalinkEvent();

  if (autopilot_rc) {
    RadioControlEvent(autopilot_on_rc_frame);
  }

  ImuEvent(on_gyro_event, on_accel_event, on_mag_event);

#if USE_BARO_BOARD
  BaroEvent();
#endif

#if USE_GPS
  GpsEvent(on_gps_event);
#endif

#if FAILSAFE_GROUND_DETECT || KILL_ON_GROUND_DETECT
  DetectGroundEvent();
#endif

  modules_event_task();

}

static inline void on_accel_event( void ) {
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_CORRECT_FREQUENCY)
PRINT_CONFIG_MSG("Calculating dt for AHRS accel update.")
  // timestamp in usec when last callback was received
  static uint32_t last_ts = 0;
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();
  // dt between this and last callback
  float dt = (float)(now_ts - last_ts) / 1e6;
  last_ts = now_ts;
#else
PRINT_CONFIG_MSG("Using fixed AHRS_CORRECT_FREQUENCY for AHRS accel update.")
PRINT_CONFIG_VAR(AHRS_CORRECT_FREQUENCY)
  const float dt = 1. / (AHRS_CORRECT_FREQUENCY);
#endif

  imu_scale_accel(&imu);

  if (ahrs.status != AHRS_UNINIT) {
    ahrs_update_accel(dt);
  }
}

static inline void on_gyro_event( void ) {
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_PROPAGATE_FREQUENCY)
PRINT_CONFIG_MSG("Calculating dt for AHRS/INS propagation.")
  // timestamp in usec when last callback was received
  static uint32_t last_ts = 0;
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();
  // dt between this and last callback in seconds
  float dt = (float)(now_ts - last_ts) / 1e6;
  last_ts = now_ts;
#else
PRINT_CONFIG_MSG("Using fixed AHRS_PROPAGATE_FREQUENCY for AHRS/INS propagation.")
PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
  const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
#endif

  imu_scale_gyro(&imu);

  if (ahrs.status == AHRS_UNINIT) {
    ahrs_aligner_run();
    if (ahrs_aligner.status == AHRS_ALIGNER_LOCKED)
      ahrs_align();
  }
  else {
    ahrs_propagate(dt);
#ifdef SITL
    if (nps_bypass_ahrs) sim_overwrite_ahrs();
#endif
    ins_propagate(dt);
  }
#ifdef USE_VEHICLE_INTERFACE
  vi_notify_imu_available();
#endif
}

static inline void on_gps_event(void) {
  ahrs_update_gps();
  ins_update_gps();
#ifdef USE_VEHICLE_INTERFACE
  if (gps.fix == GPS_FIX_3D)
    vi_notify_gps_available();
#endif
}

static inline void on_mag_event(void) {
  imu_scale_mag(&imu);

#if USE_MAGNETOMETER
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_MAG_CORRECT_FREQUENCY)
PRINT_CONFIG_MSG("Calculating dt for AHRS mag update.")
  // timestamp in usec when last callback was received
  static uint32_t last_ts = 0;
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();
  // dt between this and last callback in seconds
  float dt = (float)(now_ts - last_ts) / 1e6;
  last_ts = now_ts;
#else
PRINT_CONFIG_MSG("Using fixed AHRS_MAG_CORRECT_FREQUENCY for AHRS mag update.")
PRINT_CONFIG_VAR(AHRS_MAG_CORRECT_FREQUENCY)
  const float dt = 1. / (AHRS_MAG_CORRECT_FREQUENCY);
#endif

  if (ahrs.status == AHRS_RUNNING) {
    ahrs_update_mag(dt);
  }
#endif // USE_MAGNETOMETER

#ifdef USE_VEHICLE_INTERFACE
  vi_notify_mag_available();
#endif
}
