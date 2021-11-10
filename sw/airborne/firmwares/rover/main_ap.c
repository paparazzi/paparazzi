/*
 * Copyright (C) 2018-2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file firmwares/rover/main_ap.c
 *
 * Rover main loop.
 */

#define MODULES_C

#define ABI_C

#include <inttypes.h>
#include "led.h"

#include "subsystems/radio_control.h"

#include "firmwares/rover/main_ap.h"

#include "generated/modules.h"
#include "modules/core/abi.h"

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
tid_t modules_radio_control_tid;
tid_t modules_gnc_tid; // estimation, control, actuators, default in a single step
tid_t modules_datalink_tid;
tid_t modules_default_tid;
tid_t failsafe_tid;      ///< id for failsafe_check() timer FIXME

#define SYS_PERIOD (1.f / PERIODIC_FREQUENCY)
#define SENSORS_PERIOD (1.f / PERIODIC_FREQUENCY)
#define DATALINK_PERIOD (1.f / TELEMETRY_FREQUENCY)

void main_init(void)
{
  modules_mcu_init();
  modules_core_init();
  modules_sensors_init();
  modules_estimation_init();
  radio_control_init();
  // modules_radio_control_init(); FIXME
  modules_control_init();
  modules_actuators_init();
  modules_datalink_init();
  modules_default_init();

  // call autopilot implementation init after guidance modules init
  // it will set startup mode
  autopilot_generated_init();

  // register timers with temporal dependencies
  modules_sensors_tid = sys_time_register_timer(SENSORS_PERIOD, NULL);

  // common GNC group (estimation, control, actuators, default)
  // is called with an offset of half the main period (1/PERIODIC_FREQUENCY)
  // which is the default resolution of SYS_TIME_FREQUENCY,
  // hence the resolution of the virtual timers.
  // In practice, this is the best compromised between having enough time between
  // the sensor readings (triggerd in sensors task group) and the lag between
  // the state update and control/actuators update
  //
  //      |      PERIODIC_FREQ       |
  //      |            |             |
  //      read         gnc
  //
  modules_gnc_tid = sys_time_register_timer_offset(modules_sensors_tid, 1.f / (2.f * PERIODIC_FREQUENCY), NULL);

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

  if (sys_time_check_and_ack_timer(modules_radio_control_tid)) {
    radio_control_periodic_task();
    modules_radio_control_periodic_task(); // FIXME integrate above
  }

  if (sys_time_check_and_ack_timer(modules_gnc_tid)) {
    modules_estimation_periodic_task();
    modules_control_periodic_task();
    modules_default_periodic_task();
    SetActuatorsFromCommands(commands, autopilot_get_mode());
    modules_actuators_periodic_task(); // FIXME integrate above in actuators periodic
    if (autopilot_in_flight()) {
      RunOnceEvery(PERIODIC_FREQUENCY, autopilot.flight_time++); // TODO make it 1Hz periodic ?
    }
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
  modules_control_event_task();
  modules_actuators_event_task();
  modules_datalink_event_task();
  modules_default_event_task();
}

