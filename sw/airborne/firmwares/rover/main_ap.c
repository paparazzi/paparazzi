/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "subsystems/datalink/telemetry.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/settings.h"

#include "subsystems/commands.h"
#include "subsystems/actuators.h"

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

#include "autopilot.h"

#include "subsystems/radio_control.h"

#include "subsystems/ahrs.h"

#include "state.h"

#include "firmwares/rover/main_ap.h"

#include "generated/modules.h"
#include "subsystems/abi.h"

// needed for stop-gap measure waypoints_localize_all()
#include "subsystems/navigation/waypoints.h"


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

void main_init(void)
{
  mcu_init();

#if defined(PPRZ_TRIG_INT_COMPR_FLASH)
  pprz_trig_int_init();
#endif

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

#if USE_AHRS
  ahrs_init();
#endif

  autopilot_init();

  modules_init();

  // call autopilot implementation init after guidance modules init
  // it will set startup mode
  autopilot_generated_init();

  /* temporary hack:
   * Since INS is now a module, LTP_DEF is not yet initialized when autopilot_init is called
   * This led to the problem that global waypoints were not "localized",
   * so as a stop-gap measure we localize them all (again) here..
   */
  //waypoints_localize_all();

  settings_init();

  mcu_int_enable();

#if DOWNLINK
  downlink_init();
#endif

  // register the timers for the periodic functions
  main_periodic_tid = sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
#if PERIODIC_FREQUENCY != MODULES_FREQUENCY
  modules_tid = sys_time_register_timer(1. / MODULES_FREQUENCY, NULL);
#endif
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

void handle_periodic_tasks(void)
{
  if (sys_time_check_and_ack_timer(main_periodic_tid)) {
    main_periodic();
#if PERIODIC_FREQUENCY == MODULES_FREQUENCY
    /* Use the main periodc freq timer for modules if the freqs are the same
     * This is mainly useful for logging each step.
     */
    modules_periodic_task();
#else
  }
  /* separate timer for modules, since it has a different freq than main */
  if (sys_time_check_and_ack_timer(modules_tid)) {
    modules_periodic_task();
#endif
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

void main_periodic(void)
{
  /* run control loops */
  autopilot_periodic();
  /* set actuators     */
  //actuators_set(autopilot_get_motors_on());
  SetActuatorsFromCommands(commands, autopilot_get_mode());

  if (autopilot_in_flight()) {
    RunOnceEvery(PERIODIC_FREQUENCY, autopilot.flight_time++);
  }

#if defined DATALINK || defined SITL
  RunOnceEvery(PERIODIC_FREQUENCY, datalink_time++);
#endif

  RunOnceEvery(10, LED_PERIODIC());
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
  /* event functions for mcu peripherals: i2c, usb_serial.. */
  mcu_event();

  if (autopilot.use_rc) {
    RadioControlEvent(autopilot_on_rc_frame);
  }

#if USE_BARO_BOARD
  BaroEvent();
#endif

  autopilot_event();

  modules_event_task();
}

