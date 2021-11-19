/*
 * Copyright (C) 2003-2021  The Paparazzi Team
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
//tid_t modules_radio_control_tid; // done in FBW
tid_t modules_gnc_tid; // estimation, control, actuators, default in a single step
tid_t modules_datalink_tid;

#define SYS_PERIOD (1.f / PERIODIC_FREQUENCY)
#define SENSORS_PERIOD (1.f / PERIODIC_FREQUENCY)
#define DATALINK_PERIOD (1.f / TELEMETRY_FREQUENCY)

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
  modules_datalink_tid = sys_time_register_timer(DATALINK_PERIOD, NULL);

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

  if (sys_time_check_and_ack_timer(modules_gnc_tid)) {
    modules_estimation_periodic_task();
    modules_control_periodic_task();
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
  modules_control_event_task();
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

