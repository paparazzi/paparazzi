/*
 * Copyright (C) 2015 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
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
 * Main file for ChibiOS/RT Paparazzi fixedwing
 *
 * Includes both Paparazzi and ChibiOS files, threads are static.
 *
 * @author {Michal Podhradsky, Calvin Coopmans}
 */
#include "firmwares/fixedwing/main_threads.h"
#include "firmwares/fixedwing/main_chibios.h"

/**
 * HeartBeat & System Info
 *
 * Blinks LED and logs the cpu usage and other system info
 */
static THD_WORKING_AREA(wa_thd_heartbeat, CH_THREAD_AREA_HEARTBEAT);
void thd_heartbeat(void *arg)
{
  chRegSetThreadName("heartbeat");
  (void) arg;
  systime_t time = chVTGetSystemTime();
  static uint32_t last_idle_counter = 0;
  //static uint32_t last_nb_sec = 0;

  while (TRUE) {
    time += S2ST(1);

#ifdef SYS_TIME_LED
    LED_TOGGLE(SYS_TIME_LED);
#endif

    core_free_memory = chCoreGetStatusX();
    thread_counter = 0;

    thread_t *tp;
    tp = chRegFirstThread();
    do {
      thread_counter++;
      if (tp == chSysGetIdleThreadX()) {
#if CH_DBG_THREADS_PROFILING
        idle_counter = (uint32_t)tp->p_time;
#endif
      }
      tp = chRegNextThread(tp);
    } while (tp != NULL);

    // assume we call the counter once a second
    // so the difference in seconds is always one
    // NOTE: not perfectly precise due to low heartbeat priority -> +-5% margins
    // FIXME: add finer resolution than seconds?
    cpu_counter = (idle_counter - last_idle_counter);// / ((nb_sec - last_nb_sec)/CH_CFG_ST_FREQUENCY);
        // / (sys_time.nb_sec - last_nb_sec);
    cpu_frequency = (1 - (float) cpu_counter / CH_CFG_ST_FREQUENCY) * 100;

    last_idle_counter = idle_counter;
    //last_nb_sec = sys_time.nb_sec;
    //last_nb_sec = nb_sec;

    chThdSleepUntil(time);
  }
}

/*
 * Electrical Periodic Thread
 *
 * Calls electrical_periodic()
 */
static THD_WORKING_AREA(wa_thd_electrical, CH_THREAD_AREA_ELECTRICAL);
void thd_electrical(void *arg)
{
  chRegSetThreadName("electrical");
  (void) arg;
  electrical_init();
  systime_t time = chVTGetSystemTime();
  while (TRUE) {
    time += US2ST(1000000/ELECTRICAL_PERIODIC_FREQ);
    electrical_periodic();
    chThdSleepUntil(time);
  }
}

/**
 * Radio Control Periodic Thread
 *
 * Calls radio_control_periodic()
 */
static THD_WORKING_AREA(wa_thd_radio_control, CH_THREAD_AREA_RADIO_CONTROL);
void thd_radio_control(void *arg)
{
  chRegSetThreadName("radio_control");
  (void) arg;
  //radio_control_init();
  systime_t time = chVTGetSystemTime();
  while (TRUE) {
    time += US2ST(1000000/RADIO_CONTROL_FREQ);
    radio_control_periodic_task();
    chThdSleepUntil(time);
  }
}

/**
 * Radio Control Event Thread
 *
 * Waits for EVT_PPM_FRAME event flag to be broadcasted,
 * then executes RadioControlEvent()
 *
 * @note: It is a nice example how to use event listeners.
 * Optionally after the frame is processed, another event can be
 * broadcasted, so it is possible to chain data processing (i.e. in AHRS)
 * Maybe a similar structure can be used for GPS events etc.
 *
 * after receiving EVT_PPM_FRAM and processing it, we can call
 * chEvtBroadcastFlags(&initializedEventSource, SOME_DEFINED_EVENT);
 * to propagate event further
 */
static THD_WORKING_AREA(wa_thd_radio_event, CH_THREAD_AREA_RADIO_EVENT);
void thd_radio_event(void *arg)
{
  chRegSetThreadName("radio_event");
  (void) arg;

  event_listener_t elRadioEvt;
  chEvtRegister(&eventRadioFrame, &elRadioEvt, EVT_RADIO_FRAME);
  eventflags_t rc_flags;

  while (TRUE) {
    chEvtWaitOne(EVENT_MASK(EVT_RADIO_FRAME));
    rc_flags = chEvtGetAndClearFlags(&elRadioEvt);
    if (rc_flags & EVT_RADIO_FRAME) {
      RadioControlEvent(handle_rc_frame);
      chEvtBroadcastFlags(&eventRadioData, EVT_RADIO_DATA);
    }
  }
}

//#if PERIODIC_TELEMETRY
/**
 * Telemetry TX thread
 */
static THD_WORKING_AREA(wa_thd_telemetry_tx, CH_THREAD_AREA_DOWNLINK_TX);
void thd_telemetry_tx(void *arg)
{
  (void) arg;
  chRegSetThreadName("telemetry_tx");
  systime_t time = chVTGetSystemTime();
  while (TRUE) {
    time += US2ST(1000000 / TELEMETRY_FREQUENCY);
    reporting_task();
    chThdSleepUntil(time);
  }
}

/**
 *  Telemetry RX thread
 *
 *  Replaces DatalinkEvent()
 */
static THD_WORKING_AREA(wa_thd_telemetry_rx, CH_THREAD_AREA_DOWNLINK_RX);
void thd_telemetry_rx(void *arg)
{
  chRegSetThreadName("telemetry_rx");
  (void) arg;
  event_listener_t elTelemetryRx;
  eventflags_t flags;
  chEvtRegisterMask(
      (event_source_t *) chnGetEventSource(
          (SerialDriver*) DOWNLINK_DEVICE.reg_addr), &elTelemetryRx,
      EVENT_MASK(1));
  while (TRUE) {
    chEvtWaitOneTimeout(EVENT_MASK(1), S2ST(1));
    flags = chEvtGetAndClearFlags(&elTelemetryRx);
    // TODO: fix according to the EVENTs guide: http://chibios-book.readthedocs.org/en/latest/14_events/

     ch_uart_receive_downlink(DOWNLINK_DEVICE, flags, parse_pprz, &pprz_tp);
     if (pprz_tp.trans_rx.msg_received)
     {
     pprz_parse_payload(&(pprz_tp));
     pprz_tp.trans_rx.msg_received = FALSE;
     dl_parse_msg();
     dl_msg_available = FALSE;
     }

  }
}
//#endif /* PERIODIC_TELEMETRY */

/**
 * Modules periodic tasks
 */
static THD_WORKING_AREA(wa_thd_modules_periodic, CH_THREAD_AREA_MODULES);
void thd_modules_periodic(void *arg)
{
  chRegSetThreadName("modules_periodic");
  (void) arg;
  systime_t time = chVTGetSystemTime();
  while (TRUE) {
    time += US2ST(1000000/MODULES_FREQUENCY);
    modules_periodic_task();
    chThdSleepUntil(time);
  }
}


/**
 * Navigation task
 */
static THD_WORKING_AREA(wa_thd_navigation, CH_THREAD_AREA_NAVIGATION);
void thd_navigation(void *arg)
{
  chRegSetThreadName("navigation_task");
  (void) arg;
  systime_t time = chVTGetSystemTime();
  while (TRUE)
  {
    time += US2ST(1000000/NAVIGATION_FREQUENCY);
    navigation_task();
    chThdSleepUntil(time);
  }
}


/**
 * Monitoring Thread
 *
 * Runs Monitoring tasks, possibly also failsafe check if needed
 *
 * TODO: ChibiOS/RT failsafe check (hypervisor thread)
 */
static THD_WORKING_AREA(wa_thd_monitor, CH_THREAD_AREA_MONITOR);
void thd_monitor(void *arg)
{
  chRegSetThreadName("monitor_task");
  (void) arg;
  systime_t time = chVTGetSystemTime();
  while (TRUE)
  {
    time += US2ST(1000000/MONITOR_FREQUENCY);
    monitor_task();
    chThdSleepUntil(time);
  }
}


/**
 * Initialize threads
 */
void spawn_threads(void)
{
  chThdCreateStatic(wa_thd_heartbeat, sizeof(wa_thd_heartbeat), LOWPRIO,
      thd_heartbeat, NULL);
  chThdCreateStatic(wa_thd_radio_event, sizeof(wa_thd_radio_event), NORMALPRIO,
      thd_radio_event, NULL);
  chThdCreateStatic(wa_thd_radio_control, sizeof(wa_thd_radio_control),
      NORMALPRIO, thd_radio_control, NULL);
  chThdCreateStatic(wa_thd_electrical, sizeof(wa_thd_electrical), LOWPRIO,
      thd_electrical, NULL);

#if PERIODIC_TELEMETRY
  chThdCreateStatic(wa_thd_telemetry_tx, sizeof(wa_thd_telemetry_tx), LOWPRIO,
      thd_telemetry_tx, NULL);
  chThdCreateStatic(wa_thd_telemetry_rx, sizeof(wa_thd_telemetry_rx), LOWPRIO,
      thd_telemetry_rx, NULL);
#endif /* PERIODIC_TELEMETRY */

  chThdCreateStatic(wa_thd_monitor, sizeof(wa_thd_monitor),
        HIGHPRIO, thd_monitor, NULL);

  chThdCreateStatic(wa_thd_navigation, sizeof(wa_thd_navigation),
      NORMALPRIO, thd_navigation, NULL);

  chThdCreateStatic(wa_thd_modules_periodic, sizeof(wa_thd_modules_periodic),
      NORMALPRIO, thd_modules_periodic, NULL);
}
