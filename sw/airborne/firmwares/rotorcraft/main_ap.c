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
 * @file firmwares/rotorcraft/main_ap.c
 *
 * Rotorcraft main loop.
 */

#define MODULES_C

#define ABI_C

#include <inttypes.h>
#include "led.h"

#include "subsystems/radio_control.h"

#include "firmwares/rotorcraft/main_ap.h"

#ifdef SITL
#include "nps_autopilot.h"
#endif

#include "generated/modules.h"
#include "subsystems/abi.h"

/* if PRINT_CONFIG is defined, print some config options */
PRINT_CONFIG_VAR(PERIODIC_FREQUENCY)
/* SYS_TIME_FREQUENCY/PERIODIC_FREQUENCY should be an integer, otherwise the timer will not be correct */
#if !(SYS_TIME_FREQUENCY/PERIODIC_FREQUENCY*PERIODIC_FREQUENCY == SYS_TIME_FREQUENCY)
#warning "The SYS_TIME_FREQUENCY can not be divided by PERIODIC_FREQUENCY. Make sure this is the case for correct timing."
#endif

/* TELEMETRY_FREQUENCY is defined in generated/periodic_telemetry.h
 * defaults to 60Hz or set by TELEMETRY_FREQUENCY configure option in airframe file
 */
PRINT_CONFIG_VAR(TELEMETRY_FREQUENCY)

/* MODULES_FREQUENCY is defined in generated/modules.h
 * according to main_freq parameter set for modules in airframe file
 */
PRINT_CONFIG_VAR(MODULES_FREQUENCY)

#if USE_AHRS && USE_IMU && (defined AHRS_PROPAGATE_FREQUENCY)
#if (AHRS_PROPAGATE_FREQUENCY > PERIODIC_FREQUENCY)
#warning "PERIODIC_FREQUENCY should be least equal or greater than AHRS_PROPAGATE_FREQUENCY"
INFO_VALUE("it is recommended to configure in your airframe PERIODIC_FREQUENCY to at least ", AHRS_PROPAGATE_FREQUENCY)
#endif
#endif

/**
 * IDs for timers
 */
tid_t modules_mcu_core_tid; // single step
tid_t modules_sensors_tid;
tid_t modules_estimation_tid;
tid_t modules_radio_control_tid;
tid_t modules_control_actuators_tid; // single step
tid_t modules_datalink_tid;
tid_t modules_default_tid;
tid_t failsafe_tid;      ///< id for failsafe_check() timer FIXME

#define SYS_PERIOD (1.f / PERIODIC_FREQUENCY)
#define DATALINK_PERIOD (1.f / TELEMETRY_FREQUENCY)

#ifndef ESTIMATION_OFFSET
#define ESTIMATION_OFFSET 6e-4f
#endif

#ifndef CONTROL_OFFSET
#define CONTROL_OFFSET 7e-4f
#endif

#ifndef DEFAULT_OFFSET
#define DEFAULT_OFFSET 8e-4f
#endif

void main_init(void)
{
  modules_mcu_init();
  modules_core_init();
  modules_sensors_init();
  modules_estimation_init();
#ifndef INTER_MCU_AP
  radio_control_init();
  // modules_radio_control_init(); FIXME
#endif
  modules_control_init();
  modules_actuators_init();
  modules_datalink_init();
  modules_default_init();

  // call autopilot implementation init after guidance modules init
  // it will set startup mode
#if USE_GENERATED_AUTOPILOT
  autopilot_generated_init();
#else
  autopilot_static_init();
#endif

  // register timers with temporal dependencies
  modules_sensors_tid = sys_time_register_timer(SYS_PERIOD, NULL);
  modules_estimation_tid = sys_time_register_timer_offset(modules_sensors_tid, ESTIMATION_OFFSET, NULL);
  modules_control_actuators_tid = sys_time_register_timer_offset(modules_sensors_tid, CONTROL_OFFSET, NULL);
  modules_default_tid = sys_time_register_timer_offset(modules_sensors_tid, DEFAULT_OFFSET, NULL); // should it be an offset ?

  // register the timers for the periodic functions
  modules_mcu_core_tid = sys_time_register_timer(SYS_PERIOD, NULL);
  modules_radio_control_tid = sys_time_register_timer((1. / 60.), NULL); // FIXME
  modules_datalink_tid = sys_time_register_timer(DATALINK_PERIOD, NULL);
  failsafe_tid = sys_time_register_timer(0.05, NULL); // FIXME

#if USE_IMU
  // send body_to_imu from here for now
  AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
#endif

  // Do a failsafe check first
  failsafe_check();

}

void handle_periodic_tasks(void)
{
  if (sys_time_check_and_ack_timer(modules_sensors_tid)) {
    modules_sensors_periodic_task();
  }

  if (sys_time_check_and_ack_timer(modules_estimation_tid)) {
    modules_estimation_periodic_task();
  }

  if (sys_time_check_and_ack_timer(modules_radio_control_tid)) {
    radio_control_periodic_task();
    modules_radio_control_periodic_task(); // FIXME integrate above
  }

  if (sys_time_check_and_ack_timer(modules_control_actuators_tid)) {
    modules_control_periodic_task();

#if USE_THROTTLE_CURVES
    throttle_curve_run(commands, autopilot_get_mode());
#endif

#ifndef INTER_MCU_AP
    SetActuatorsFromCommands(commands, autopilot_get_mode());
#else
    intermcu_set_actuators(commands, autopilot_get_mode());
#endif
    modules_actuators_periodic_task(); // FIXME integrate above in actuators periodic

    if (autopilot_in_flight()) {
      RunOnceEvery(PERIODIC_FREQUENCY, autopilot.flight_time++); // TODO make it 1Hz periodic ?
    }
  }

  if (sys_time_check_and_ack_timer(modules_default_tid)) {
    modules_default_periodic_task();
  }

  if (sys_time_check_and_ack_timer(modules_mcu_core_tid)) {
    modules_mcu_periodic_task();
    modules_core_periodic_task();
    RunOnceEvery(10, LED_PERIODIC()); // FIXME periodic in led module
  }

  if (sys_time_check_and_ack_timer(modules_datalink_tid)) {
    telemetry_periodic();
    modules_datalink_periodic_task(); // FIXME integrate above
#if defined DATALINK || defined SITL
    RunOnceEvery(TELEMETRY_FREQUENCY, datalink_time++);
#endif
  }

  if (sys_time_check_and_ack_timer(failsafe_tid)) {
    failsafe_check(); // FIXME integrate somewhere else
  }

}

void telemetry_periodic(void)
{
  static uint8_t boot = true;

  /* initialisation phase during boot */
  if (boot) {
#if DOWNLINK
    autopilot_send_version();
#endif
    boot = false;
  }
  /* then report periodicly */
  else {
#if PERIODIC_TELEMETRY
    periodic_telemetry_send_Main(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif
  }
}

/** mode to enter when RC is lost while using a mode with RC input (not AP_MODE_NAV) */
#ifndef RC_LOST_MODE
#define RC_LOST_MODE AP_MODE_FAILSAFE
#endif

void failsafe_check(void)
{
#if !USE_GENERATED_AUTOPILOT
  if (radio_control.status == RC_REALLY_LOST &&
      autopilot_get_mode() != AP_MODE_KILL &&
      autopilot_get_mode() != AP_MODE_HOME &&
      autopilot_get_mode() != AP_MODE_FAILSAFE &&
      autopilot_get_mode() != AP_MODE_NAV &&
      autopilot_get_mode() != AP_MODE_MODULE &&
      autopilot_get_mode() != AP_MODE_FLIP &&
      autopilot_get_mode() != AP_MODE_GUIDED) {
    autopilot_set_mode(RC_LOST_MODE);
  }

#if FAILSAFE_ON_BAT_CRITICAL
  if (autopilot_get_mode() != AP_MODE_KILL &&
      electrical.bat_critical) {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }
#endif

#if USE_GPS
  if (autopilot_get_mode() == AP_MODE_NAV &&
      autopilot_get_motors_on() &&
#if NO_GPS_LOST_WITH_RC_VALID
      radio_control.status != RC_OK &&
#endif
#ifdef NO_GPS_LOST_WITH_DATALINK_TIME
      datalink_time > NO_GPS_LOST_WITH_DATALINK_TIME && 
#endif
      GpsIsLost()) {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }

  if (autopilot_get_mode() == AP_MODE_HOME &&
      autopilot_get_motors_on() && GpsIsLost()) {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }
#endif

#endif // !USE_GENERATED_AUTOPILOT

  autopilot_check_in_flight(autopilot_get_motors_on());
}

void main_event(void)
{
  modules_mcu_event_task();
  modules_core_event_task();
  modules_sensors_event_task();
  modules_estimation_event_task();
  modules_radio_control_event_task(); // FIXME
  if (autopilot.use_rc) {
    RadioControlEvent(autopilot_on_rc_frame);
  }
  modules_actuators_event_task();
  modules_datalink_event_task();
  modules_default_event_task();
}
