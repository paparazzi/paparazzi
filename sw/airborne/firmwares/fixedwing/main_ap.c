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

#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "generated/modules.h"
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


#if USE_IMU
#ifdef AHRS_PROPAGATE_FREQUENCY
#if (AHRS_PROPAGATE_FREQUENCY > PERIODIC_FREQUENCY)
#warning "PERIODIC_FREQUENCY should be least equal or greater than AHRS_PROPAGATE_FREQUENCY"
INFO_VALUE("it is recommended to configure in your airframe PERIODIC_FREQUENCY to at least ", AHRS_PROPAGATE_FREQUENCY)
#endif
#endif
#endif // USE_IMU

/**
 * IDs for timers
 */
tid_t modules_mcu_core_tid; // single step
tid_t modules_sensors_tid;
tid_t modules_estimation_tid;
//tid_t modules_radio_control_tid; // done in FBW
tid_t modules_control_actuators_tid; // single step
tid_t modules_datalink_tid;
tid_t modules_default_tid;
tid_t monitor_tid;     ///< id for monitor_task() timer FIXME

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

void init_ap(void)
{
#ifndef SINGLE_MCU
  modules_mcu_init();
#endif
  modules_core_init();
  modules_sensors_init();
  modules_estimation_init();
  //radio_control_init(); FIXME done in FBW
  // modules_radio_control_init(); FIXME
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
  //modules_radio_control_tid = sys_time_register_timer((1. / 60.), NULL); // FIXME
  modules_datalink_tid = sys_time_register_timer(DATALINK_PERIOD, NULL);
  monitor_tid = sys_time_register_timer(1., NULL); // FIXME

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
  if (sys_time_check_and_ack_timer(modules_sensors_tid)) {
    modules_sensors_periodic_task();
  }

  if (sys_time_check_and_ack_timer(modules_estimation_tid)) {
    modules_estimation_periodic_task();
  }

  // done in FBW
  //if (sys_time_check_and_ack_timer(modules_radio_control_tid)) {
  //  radio_control_periodic_task();
  //  modules_radio_control_periodic_task(); // FIXME integrate above
  //}

  if (sys_time_check_and_ack_timer(modules_control_actuators_tid)) {
    modules_control_periodic_task();
  }

  if (sys_time_check_and_ack_timer(modules_default_tid)) {
    modules_default_periodic_task();
  }

  if (sys_time_check_and_ack_timer(modules_mcu_core_tid)) {
    modules_mcu_periodic_task();
    modules_core_periodic_task();
    LED_PERIODIC(); // FIXME periodic in led module
  }

  if (sys_time_check_and_ack_timer(modules_datalink_tid)) {
    reporting_task();
    modules_datalink_periodic_task(); // FIXME integrate above
#if defined DATALINK || defined SITL
    RunOnceEvery(TELEMETRY_FREQUENCY, datalink_time++);
#endif
  }

  if (sys_time_check_and_ack_timer(monitor_tid)) {
    monitor_task();
  }
}



/**************************** Periodic tasks ***********************************/

/**
 * Send a series of initialisation messages followed by a stream of periodic ones.
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
#if PERIODIC_TELEMETRY
    periodic_telemetry_send_Ap(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif
  }
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
  modules_mcu_event_task();
#endif /* SINGLE_MCU */
  modules_core_event_task();
  modules_sensors_event_task();
  modules_estimation_event_task();
  modules_datalink_event_task();
  modules_default_event_task();


  // TODO integrate in modules
#if defined MCU_SPI_LINK || defined MCU_UART_LINK
  link_mcu_event_task();
#endif
  if (inter_mcu_received_fbw) {
    /* receive radio control task from fbw */
    inter_mcu_received_fbw = false;
    autopilot_on_rc_frame();
  }

} /* event_task_ap() */

