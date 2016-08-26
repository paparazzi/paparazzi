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

#include "subsystems/datalink/downlink.h"
#include "telemetry.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/settings.h"
#include "subsystems/datalink/xbee.h"

#include "subsystems/electrical.h"
#include "subsystems/gps.h"

/*
 * Thread definitions
 */
static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg);

#ifdef DOWNLINK
static __attribute__((noreturn)) msg_t thd_telemetry_tx(void *arg);
static __attribute__((noreturn)) msg_t thd_telemetry_rx(void *arg);
#endif

#ifdef USE_GPS
static __attribute__((noreturn)) msg_t thd_gps_rx(void *arg);
#endif

/* if PRINT_CONFIG is defined, print some config options */
PRINT_CONFIG_VAR(PERIODIC_FREQUENCY)

#ifndef TELEMETRY_FREQUENCY
#define TELEMETRY_FREQUENCY 60
#endif
#define TELEMETRY_MS 1000
PRINT_CONFIG_VAR(TELEMETRY_FREQUENCY)

#ifndef MODULES_FREQUENCY
#define MODULES_FREQUENCY 512
#endif
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
  chRegSetThreadName("pprz heartbeat");
  (void) arg;
  systime_t time = chTimeNow();     // Current time
  while (TRUE) {
    time += MS2ST(1000);            // Next deadline, sleep for one sec.
    LED_TOGGLE(SYS_TIME_LED);
    core_free_memory = chCoreStatus();
    heap_fragments = chHeapStatus(NULL, &heap_free_total);
    Thread *tp;
    thread_counter = 0;
    tp = chRegFirstThread();
    do {
      thread_counter++;
      tp = chRegNextThread(tp);
    } while (tp != NULL);
    cpu_frequency = idle_counter/cpu_counter;
    idle_counter = 0;
    cpu_counter = 0;
    chThdSleepUntil(time);
  }
}


#ifdef USE_GPS
/*
 * GPS, just RX thread for now
 * Replaces GpsEvent()
 * @note _gps_callback() has to be implemented, memory can be probably smaller
 */
static WORKING_AREA(wa_thd_gps_rx, 1024);
static __attribute__((noreturn)) msg_t thd_gps_rx(void *arg)
{
  chRegSetThreadName("pprz_gps_rx");
  (void) arg;
  // Only internals of the thread are defined for now
  //when it was properly defined in gps file, there were problems with dependencies
  GpsThread();
}
#endif


#ifdef DOWNLINK
/*
 *  Telemetry TX
 *  Replaces telemetryPeriodic()
 */
static WORKING_AREA(wa_thd_telemetry_tx, 1024);
static __attribute__((noreturn)) msg_t thd_telemetry_tx(void *arg)
{
  chRegSetThreadName("pprz_telemetry_tx");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE) {
	//Due to integer rounding, the actual frequency is 62.5Hz
    time += MS2ST(1000/TELEMETRY_FREQUENCY);
  	LED_TOGGLE(BARO_LED);//DEBUG
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
static __attribute__((noreturn)) msg_t thd_telemetry_rx(void *arg)
{
  chRegSetThreadName("pprz_telemetry_rx");
  (void) arg;
   EventListener elTelemetryRx;
   flagsmask_t flags;
   chEvtRegisterMask((EventSource *)chnGetEventSource(&SD2), &elTelemetryRx, EVENT_MASK(1));

   while (TRUE)
   {
      chEvtWaitOneTimeout(EVENT_MASK(1), MS2ST(10));
      chSysLock();
      flags = chEvtGetAndClearFlags(&elTelemetryRx);
      chSysUnlock();

      if (flags & CHN_INPUT_AVAILABLE)
      {
         msg_t charbuf;
         do
         {
            charbuf = chnGetTimeout(&SD2, TIME_IMMEDIATE);
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
        LED_TOGGLE(RADIO_CONTROL_LED);//DEBUG
      }
   }
}
#endif

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
  electrical_init();
#ifdef USE_GPS
  gps_init();
#endif


  /*
   * Thread initialization
   */
  chThdCreateStatic(wa_thd_heartbeat, sizeof(wa_thd_heartbeat), IDLEPRIO, thd_heartbeat, NULL);
#ifdef DOWNLINK
  chThdCreateStatic(wa_thd_telemetry_tx, sizeof(wa_thd_telemetry_tx),LOWPRIO, thd_telemetry_tx, NULL);
  chThdCreateStatic(wa_thd_telemetry_rx, sizeof(wa_thd_telemetry_rx),NORMALPRIO, thd_telemetry_rx, NULL);
#endif
#ifdef USE_GPS
  chThdCreateStatic(wa_thd_gps_rx, sizeof(wa_thd_gps_rx),NORMALPRIO, thd_gps_rx, NULL);
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
