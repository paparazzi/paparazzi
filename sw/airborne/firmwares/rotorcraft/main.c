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

#if USE_IMU
#include "subsystems/imu.h"
#endif
#if USE_GPS
#include "subsystems/gps.h"
#endif

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
#if USE_AHRS_ALIGNER
#include "subsystems/ahrs/ahrs_aligner.h"
#endif
#include "subsystems/ins.h"

#include "state.h"

#include "firmwares/rotorcraft/main.h"

#ifdef SITL
#include "nps_autopilot.h"
#endif

#include "generated/modules.h"
#include "subsystems/abi.h"

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
INFO_VALUE("it is recommended to configure in your airframe PERIODIC_FREQUENCY to at least ", AHRS_PROPAGATE_FREQUENCY)
#endif
#endif

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
int main(void)
{
  main_init();

#if LIMIT_EVENT_POLLING
  /* Limit main loop frequency to 1kHz.
   * This is a kludge until we can better leverage threads and have real events.
   * Without this limit the event flags will constantly polled as fast as possible,
   * resulting on 100% cpu load on boards with an (RT)OS.
   * On bare metal boards this is not an issue, as you have nothing else running anyway.
   */
  uint32_t t_begin = 0;
  uint32_t t_diff = 0;
  while (1) {
    t_begin = get_sys_time_usec();

    handle_periodic_tasks();
    main_event();

    /* sleep remaining time to limit to 1kHz */
    t_diff = get_sys_time_usec() - t_begin;
    if (t_diff < 1000) {
      sys_time_usleep(1000 - t_diff);
    }
  }
#else
  while (1) {
    handle_periodic_tasks();
    main_event();
  }
#endif

  return 0;
}
#endif /* SITL */

STATIC_INLINE void main_init(void)
{
  mcu_init();

#if defined(PPRZ_TRIG_INT_COMPR_FLASH)
  pprz_trig_int_init();
#endif

  electrical_init();

  stateInit();

#ifndef INTER_MCU_AP
  actuators_init();
#else
  intermcu_init();
#endif

#if USE_MOTOR_MIXING
  motor_mixing_init();
#endif

#ifndef INTER_MCU_AP
  radio_control_init();
#endif

#if USE_BARO_BOARD
  baro_init();
#endif
#if USE_IMU
  imu_init();
#endif
#if USE_AHRS_ALIGNER
  ahrs_aligner_init();
#endif

#if USE_AHRS
  ahrs_init();
#endif

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

#ifdef INTER_MCU_AP
  intermcu_init();
#endif

  // register the timers for the periodic functions
  main_periodic_tid = sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  modules_tid = sys_time_register_timer(1. / MODULES_FREQUENCY, NULL);
  radio_control_tid = sys_time_register_timer((1. / 60.), NULL);
  failsafe_tid = sys_time_register_timer(0.05, NULL);
  electrical_tid = sys_time_register_timer(0.1, NULL);
  telemetry_tid = sys_time_register_timer((1. / TELEMETRY_FREQUENCY), NULL);
#if USE_BARO_BOARD
  baro_tid = sys_time_register_timer(1. / BARO_PERIODIC_FREQUENCY, NULL);
#endif

#if USE_IMU
  // send body_to_imu from here for now
  AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
#endif

  // Do a failsafe check first
  failsafe_check();
}

STATIC_INLINE void handle_periodic_tasks(void)
{
  if (sys_time_check_and_ack_timer(main_periodic_tid)) {
    main_periodic();
  }
  if (sys_time_check_and_ack_timer(modules_tid)) {
    modules_periodic_task();
  }
  if (sys_time_check_and_ack_timer(radio_control_tid)) {
    radio_control_periodic_task();
  }
  if (sys_time_check_and_ack_timer(failsafe_tid)) {
    failsafe_check();
  }
  if (sys_time_check_and_ack_timer(electrical_tid)) {
    electrical_periodic();
  }
  if (sys_time_check_and_ack_timer(telemetry_tid)) {
    telemetry_periodic();
  }
#if USE_BARO_BOARD
  if (sys_time_check_and_ack_timer(baro_tid)) {
    baro_periodic();
  }
#endif
}

STATIC_INLINE void main_periodic(void)
{

#if USE_IMU
  imu_periodic();
#endif

  //FIXME: temporary hack, remove me
#ifdef InsPeriodic
  InsPeriodic();
#endif

  /* run control loops */
  autopilot_periodic();
  /* set actuators     */
  //actuators_set(autopilot_motors_on);
#ifndef INTER_MCU_AP
  SetActuatorsFromCommands(commands, autopilot_mode);
#else
  intermcu_set_actuators(commands, autopilot_mode);
#endif

  if (autopilot_in_flight) {
    RunOnceEvery(PERIODIC_FREQUENCY, autopilot_flight_time++);
  }

#if defined DATALINK || defined SITL
  RunOnceEvery(PERIODIC_FREQUENCY, datalink_time++);
#endif

  RunOnceEvery(10, LED_PERIODIC());
}

STATIC_INLINE void telemetry_periodic(void)
{
  static uint8_t boot = true;

  /* initialisation phase during boot */
  if (boot) {
#if DOWNLINK
    send_autopilot_version(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
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

STATIC_INLINE void failsafe_check(void)
{
  if (radio_control.status == RC_REALLY_LOST &&
      autopilot_mode != AP_MODE_KILL &&
      autopilot_mode != AP_MODE_HOME &&
      autopilot_mode != AP_MODE_FAILSAFE &&
      autopilot_mode != AP_MODE_NAV) {
    autopilot_set_mode(RC_LOST_MODE);
  }

#if FAILSAFE_ON_BAT_CRITICAL
  if (autopilot_mode != AP_MODE_KILL &&
      electrical.bat_critical) {
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
      GpsIsLost()) {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }

  if (autopilot_mode == AP_MODE_HOME &&
      autopilot_motors_on && GpsIsLost()) {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }
#endif

  autopilot_check_in_flight(autopilot_motors_on);
}

STATIC_INLINE void main_event(void)
{
  /* event functions for mcu peripherals: i2c, usb_serial.. */
  mcu_event();

  DatalinkEvent();

  if (autopilot_rc) {
    RadioControlEvent(autopilot_on_rc_frame);
  }

#if USE_IMU
  ImuEvent();
#endif

#ifdef InsEvent
  TODO("calling InsEvent, remove me..")
  InsEvent();
#endif

#if USE_BARO_BOARD
  BaroEvent();
#endif

#if USE_GPS
  GpsEvent();
#endif

#if FAILSAFE_GROUND_DETECT || KILL_ON_GROUND_DETECT
  DetectGroundEvent();
#endif

  modules_event_task();
}
