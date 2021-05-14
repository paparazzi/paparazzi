/*
 * Copyright (C) 2003-2010  The Paparazzi Team
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

/**
 * @file firmwares/fixedwing/main_ap.c
 *
 * AP ( AutoPilot ) tasks
 *
 * This process is reponsible for the collecting the different sensors data,
 * calling the appropriate estimation algorithms and running the different control loops.
 */

#define MODULES_C

#define ABI_C

#include <math.h>

#include "firmwares/fixedwing/main_ap.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "inter_mcu.h"
#include "link_mcu.h"

// Sensors
#if USE_GPS
#include "subsystems/gps.h"
#endif
#if USE_IMU
#include "subsystems/imu.h"
#endif

// autopilot
#include "autopilot.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/flight_plan.h"

// datalink & telemetry
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

// modules & settings
#include "subsystems/settings.h"
#include "generated/modules.h"
#include "generated/settings.h"
#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
#include "modules/settings/rc_settings.h"
#endif
#include "subsystems/abi.h"

#include "led.h"

#ifdef USE_NPS
#include "nps_autopilot.h"
#endif

/* Default trim commands for roll, pitch and yaw */
#ifndef COMMAND_ROLL_TRIM
#define COMMAND_ROLL_TRIM 0
#endif

#ifndef COMMAND_PITCH_TRIM
#define COMMAND_PITCH_TRIM 0
#endif

#ifndef COMMAND_YAW_TRIM
#define COMMAND_YAW_TRIM 0
#endif

/* if PRINT_CONFIG is defined, print some config options */
PRINT_CONFIG_VAR(PERIODIC_FREQUENCY)
#if !USE_GENERATED_AUTOPILOT
PRINT_CONFIG_VAR(NAVIGATION_FREQUENCY)
#endif
PRINT_CONFIG_VAR(CONTROL_FREQUENCY)

/* TELEMETRY_FREQUENCY is defined in generated/periodic_telemetry.h
 * defaults to 60Hz or set by TELEMETRY_FREQUENCY configure option in airframe file
 */
#ifndef TELEMETRY_FREQUENCY
#define TELEMETRY_FREQUENCY 60
#endif
PRINT_CONFIG_VAR(TELEMETRY_FREQUENCY)

/* MODULES_FREQUENCY is defined in generated/modules.h
 * according to main_freq parameter set for modules in airframe file
 */
PRINT_CONFIG_VAR(MODULES_FREQUENCY)


#if USE_IMU
#ifdef AHRS_PROPAGATE_FREQUENCY
#if (AHRS_PROPAGATE_FREQUENCY > PERIODIC_FREQUENCY)
#warning "PERIODIC_FREQUENCY should be least equal or greater than AHRS_PROPAGATE_FREQUENCY"
INFO_VALUE("it is recommended to configure in your airframe PERIODIC_FREQUENCY to at least ", AHRS_PROPAGATE_FREQUENCY)
#endif
#endif
#endif // USE_IMU


tid_t modules_tid;     ///< id for modules_periodic_task() timer
tid_t telemetry_tid;   ///< id for telemetry_periodic() timer
tid_t sensors_tid;     ///< id for sensors_task() timer
tid_t attitude_tid;    ///< id for attitude_loop() timer
#if !USE_GENERATED_AUTOPILOT
tid_t navigation_tid;  ///< id for navigation_task() timer
#endif
tid_t monitor_tid;     ///< id for monitor_task() timer

void init_ap(void)
{
#ifndef SINGLE_MCU /** init done in main_fbw in single MCU */
  mcu_init();
#endif /* SINGLE_MCU */

  /************* Sensors initialization ***************/

  /************* Links initialization ***************/
#if defined MCU_SPI_LINK || defined MCU_UART_LINK || defined MCU_CAN_LINK
  link_mcu_init();
#endif

  /************ Internal status ***************/
  autopilot_init();

  modules_init();

  // call autopilot implementation init after guidance modules init
  // it will set startup mode
#if USE_GENERATED_AUTOPILOT
  autopilot_generated_init();
#else
  autopilot_static_init();
#endif

  settings_init();

  /**** start timers for periodic functions *****/
  sensors_tid = sys_time_register_timer(1. / PERIODIC_FREQUENCY, NULL);
#if !USE_GENERATED_AUTOPILOT
  navigation_tid = sys_time_register_timer(1. / NAVIGATION_FREQUENCY, NULL);
#endif
  attitude_tid = sys_time_register_timer(1. / CONTROL_FREQUENCY, NULL);
  modules_tid = sys_time_register_timer(1. / MODULES_FREQUENCY, NULL);
  telemetry_tid = sys_time_register_timer(1. / TELEMETRY_FREQUENCY, NULL);
  monitor_tid = sys_time_register_timer(1.0, NULL);

#if DOWNLINK
  downlink_init();
#endif

  /* set initial trim values.
   * these are passed to fbw via inter_mcu.
   */
  PPRZ_MUTEX_LOCK(ap_state_mtx);
  ap_state->command_roll_trim = COMMAND_ROLL_TRIM;
  ap_state->command_pitch_trim = COMMAND_PITCH_TRIM;
  ap_state->command_yaw_trim = COMMAND_YAW_TRIM;
  PPRZ_MUTEX_UNLOCK(ap_state_mtx);

#if USE_IMU
  // send body_to_imu from here for now
  AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
#endif
}


void handle_periodic_tasks_ap(void)
{

  if (sys_time_check_and_ack_timer(sensors_tid)) {
    sensors_task();
  }

#if USE_GENERATED_AUTOPILOT
  if (sys_time_check_and_ack_timer(attitude_tid)) {
    autopilot_periodic();
  }
#else
  // static autopilot
  if (sys_time_check_and_ack_timer(navigation_tid)) {
    navigation_task();
  }

#ifndef AHRS_TRIGGERED_ATTITUDE_LOOP
  if (sys_time_check_and_ack_timer(attitude_tid)) {
    attitude_loop();
  }
#endif

#endif

  if (sys_time_check_and_ack_timer(modules_tid)) {
    modules_periodic_task();
  }

  if (sys_time_check_and_ack_timer(monitor_tid)) {
    monitor_task();
  }

  if (sys_time_check_and_ack_timer(telemetry_tid)) {
    reporting_task();
    LED_PERIODIC();
  }

}



/**************************** Periodic tasks ***********************************/

/**
 * Send a series of initialisation messages followed by a stream of periodic ones.
 * Called at 60Hz.
 */
void reporting_task(void)
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
    //PeriodicSendAp(DefaultChannel, DefaultDevice);
#if PERIODIC_TELEMETRY
    periodic_telemetry_send_Ap(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif
  }
}



/** Run at PERIODIC_FREQUENCY (60Hz if not defined) */
void sensors_task(void)
{
  //FIXME: this is just a kludge
#if USE_AHRS && defined SITL && !USE_NPS
  update_ahrs_from_sim();
#endif
}

#ifdef LOW_BATTERY_KILL_DELAY
#warning LOW_BATTERY_KILL_DELAY has been renamed to CATASTROPHIC_BAT_KILL_DELAY, please update your airframe file!
#endif

/** Maximum time allowed for catastrophic battery level before going into kill mode */
#ifndef CATASTROPHIC_BAT_KILL_DELAY
#define CATASTROPHIC_BAT_KILL_DELAY 5
#endif

/** Maximum distance from HOME waypoint before going into kill mode */
#ifndef KILL_MODE_DISTANCE
#define KILL_MODE_DISTANCE (1.5*MAX_DIST_FROM_HOME)
#endif

/** Default minimal speed for takeoff in m/s */
#ifndef MIN_SPEED_FOR_TAKEOFF
#define MIN_SPEED_FOR_TAKEOFF 5.
#endif

/** monitor stuff run at 1Hz */
void monitor_task(void)
{
  if (autopilot.flight_time) {
    autopilot.flight_time++;
  }
#if defined DATALINK || defined SITL
  datalink_time++;
#endif

  static uint8_t t = 0;
  if (ap_electrical.vsupply < CATASTROPHIC_BAT_LEVEL) {
    t++;
  } else {
    t = 0;
  }
#if !USE_GENERATED_AUTOPILOT
  // only check for static autopilot
  autopilot.kill_throttle |= (t >= CATASTROPHIC_BAT_KILL_DELAY);
  autopilot.kill_throttle |= autopilot.launch && (dist2_to_home > Square(KILL_MODE_DISTANCE));
#endif

  if (!autopilot.flight_time &&
      stateGetHorizontalSpeedNorm_f() > MIN_SPEED_FOR_TAKEOFF) {
    autopilot.flight_time = 1;
    autopilot.launch = true; /* Not set in non auto launch */
#if DOWNLINK
    uint16_t time_sec = sys_time.nb_sec;
    DOWNLINK_SEND_TAKEOFF(DefaultChannel, DefaultDevice, &time_sec);
#endif
  }

}


/*********** EVENT ***********************************************************/
void event_task_ap(void)
{

#ifndef SINGLE_MCU
  /* for SINGLE_MCU done in main_fbw */
  /* event functions for mcu peripherals: i2c, usb_serial.. */
  mcu_event();
#endif /* SINGLE_MCU */

  modules_event_task();

#if defined MCU_SPI_LINK || defined MCU_UART_LINK
  link_mcu_event_task();
#endif

  if (inter_mcu_received_fbw) {
    /* receive radio control task from fbw */
    inter_mcu_received_fbw = false;
    autopilot_on_rc_frame();
  }

  autopilot_event();

} /* event_task_ap() */

