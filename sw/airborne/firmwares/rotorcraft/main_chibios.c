/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
 * @file main_chibios.c
 *
 * @brief Main file for ChibiOS/RT Paparazzi
 * @details Includes both Paparazzi and ChibiOs files
 * 			Threads are static, the memory allocation is
 * 			just approximate at this point. Eventually
 * 			most of the variables should be static.
 *
 * @author {Michal Podhradsky, Calvin Coopmans}
 */

/**
 * Chibios includes
 */
#include "ch.h"
#include "hal.h"

/**
 * Paparazzi includes
 */
#include "led.h"
#include "mcu.h"

#define MODULES_C

#define ABI_C

#include "subsystems/datalink/downlink.h"
#include "firmwares/rotorcraft/telemetry.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/settings.h"
#include "subsystems/datalink/xbee.h"
#if DATALINK == UDP
#include "subsystems/datalink/udp.h"
#endif

#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#if USE_MOTOR_MIXING
#include "subsystems/actuators/motor_mixing.h"
#endif

#include "subsystems/imu.h"
#include "subsystems/gps.h"
#include "subsystems/air_data.h"

#if USE_BARO_BOARD
#include "subsystems/sensors/baro.h"
#endif

#include "subsystems/electrical.h"

#include "firmwares/rotorcraft/autopilot.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance.h"

#include "subsystems/ahrs.h"
#include "subsystems/ins.h"

#include "state.h"

#include "generated/modules.h"
#include "subsystems/abi.h"

/*
 * Thread definitions
 */
static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg);

#ifdef DOWNLINK
__attribute__((noreturn)) msg_t thd_telemetry_tx(void *arg);
__attribute__((noreturn)) msg_t thd_telemetry_rx(void *arg);
#endif

#if USE_GPS
static WORKING_AREA(wa_thd_gps_rx, CH_THREAD_AREA_GPS_RX);
#endif

#ifdef USE_IMU
static WORKING_AREA(wa_thd_imu_rx, CH_THREAD_AREA_IMU_RX);
static WORKING_AREA(wa_thd_imu_tx, CH_THREAD_AREA_IMU_TX);
#endif


#ifdef MODULES_C
__attribute__((noreturn)) msg_t thd_modules_periodic(void *arg);
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

/*
 * HeartBeat & System Info
 */
static WORKING_AREA(wa_thd_heartbeat, 128);
static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg)
{
  chRegSetThreadName("pprz_heartbeat");
  (void) arg;
  systime_t time = chTimeNow();
  static uint32_t last_idle_counter = 0;
  static uint32_t last_nb_sec = 0;

  while (TRUE) {
    time += S2ST(1);
    LED_TOGGLE(SYS_TIME_LED);
    sys_time.nb_sec++;

    core_free_memory = chCoreStatus();
    thread_counter = 0;

    Thread *tp;
    tp = chRegFirstThread();
    do {
      thread_counter++;
      if (tp ==chSysGetIdleThread()) {
    	  idle_counter =  (uint32_t)tp->p_time;
      }
      tp = chRegNextThread(tp);
    } while (tp != NULL);

    cpu_counter = (idle_counter-last_idle_counter)/(sys_time.nb_sec-last_nb_sec);
    cpu_frequency = (1 - (float)cpu_counter/CH_FREQUENCY)*100;

    last_idle_counter = idle_counter;
    last_nb_sec = sys_time.nb_sec;

    chThdSleepUntil(time);
  }
}

#ifdef DOWNLINK
/*
 *  Telemetry TX
 *  Replaces telemetryPeriodic()
 */
static WORKING_AREA(wa_thd_telemetry_tx, 1024);
__attribute__((noreturn)) msg_t thd_telemetry_tx(void *arg)
{
  chRegSetThreadName("pprz_telemetry_tx");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE) {
      time += US2ST(1000000/TELEMETRY_FREQUENCY);
      PeriodicSendMain(DefaultChannel,DefaultDevice);
      chThdSleepUntil(time);
  }
}

/*
 *  Telemetry RX
 *  Replaces DatalinkEvent()
 *  @note: assumes PprziDwonlink for now
 *  @todo General definition for different links
 */
static WORKING_AREA(wa_thd_telemetry_rx, 1024);
__attribute__((noreturn)) msg_t thd_telemetry_rx(void *arg)
{
  chRegSetThreadName("pprz_telemetry_rx");
  (void) arg;
  EventListener elTelemetryRx;
  flagsmask_t flags;
  chEvtRegisterMask((EventSource *)chnGetEventSource((SerialDriver*)DOWNLINK_PORT.reg_addr), &elTelemetryRx, EVENT_MASK(1));
  while (TRUE)
  {
    chEvtWaitOneTimeout(EVENT_MASK(1), S2ST(1));
    chSysLock();
    flags = chEvtGetAndClearFlags(&elTelemetryRx);
    chSysUnlock();
    if ((flags & (SD_FRAMING_ERROR | SD_OVERRUN_ERROR |
                  SD_NOISE_ERROR)) != 0) {
        if (flags & SD_OVERRUN_ERROR) {
            DOWNLINK_PORT.ore++;
        }
        if (flags & SD_NOISE_ERROR) {
             DOWNLINK_PORT.ne_err++;
        }
        if (flags & SD_FRAMING_ERROR) {
             DOWNLINK_PORT.fe_err++;
        }
    }
    if (flags & CHN_INPUT_AVAILABLE)
    {
      msg_t charbuf;
      do
      {
        charbuf = chnGetTimeout((SerialDriver*)DOWNLINK_PORT.reg_addr, TIME_IMMEDIATE);
        if ( charbuf != Q_TIMEOUT )
        {
          parse_pprz(&pprz_tp, charbuf);
        }
      }
      while (charbuf != Q_TIMEOUT);
    }
    if (pprz_tp.trans.msg_received) {
      pprz_parse_payload(&(pprz_tp));      \
      pprz_tp.trans.msg_received = FALSE;  \
      dl_parse_msg();
      dl_msg_available = FALSE;
    }
  }
}
#endif

#ifdef MODULES_C
/*
 * Modules periodic tasks
 */
static WORKING_AREA(wa_thd_modules_periodic, 1024);
__attribute__((noreturn)) msg_t thd_modules_periodic(void *arg)
{
  chRegSetThreadName("pprz_modules_periodic");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE) {
    time += MS2ST(1000/MODULES_FREQUENCY);
    modules_periodic_task();
    chThdSleepUntil(time);
  }
}
#endif

/*
 * Thread initialization
 */
void thread_init(void) {
chThdCreateStatic(wa_thd_heartbeat, sizeof(wa_thd_heartbeat), IDLEPRIO, thd_heartbeat, NULL);

#ifdef USE_IMU
  chThdCreateStatic(wa_thd_imu_rx, sizeof(wa_thd_imu_rx),NORMALPRIO, thd_imu_rx, NULL);
  chThdCreateStatic(wa_thd_imu_tx, sizeof(wa_thd_imu_tx),NORMALPRIO, thd_imu_tx, NULL);
#endif

#ifdef DOWNLINK
  chThdCreateStatic(wa_thd_telemetry_rx, sizeof(wa_thd_telemetry_rx),LOWPRIO, thd_telemetry_rx, NULL);
  chThdCreateStatic(wa_thd_telemetry_tx, sizeof(wa_thd_telemetry_tx),LOWPRIO, thd_telemetry_tx, NULL);
#endif

#ifdef USE_GPS
  chThdCreateStatic(wa_thd_gps_rx, sizeof(wa_thd_gps_rx),NORMALPRIO, thd_gps_rx, NULL);
#endif

#ifdef MODULES_C
  chThdCreateStatic(wa_thd_modules_periodic, sizeof(wa_thd_modules_periodic),LOWPRIO, thd_modules_periodic, NULL);
#endif
}

/*
 * Main loop
 * Initializes system (both chibios and paparazzi), then goes to sleep.
 * Eventually Main thread will be turned into Idle thread.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Paparazzi initialization
   */
  mcu_init();
  thread_init();

  electrical_init();
  stateInit();
#ifdef USE_ACTUATORS
  actuators_init();
#endif
#if USE_MOTOR_MIXING
  motor_mixing_init();
#endif
#ifdef USE_RADIO_CONTROL
  radio_control_init();
#endif
  air_data_init();
#if USE_BARO_BOARD
  baro_init();
#endif
  imu_init();
#if USE_IMU_FLOAT
  imu_float_init();
#endif
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

#if DATALINK == XBEE
  xbee_init();
#endif

#if DATALINK == UDP
  udp_init();
#endif


  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop. Eventually we want to modify main to an idle thread.
   */
  while (TRUE) {
    chThdSleepMilliseconds(500);
  }
  return 1;
}
