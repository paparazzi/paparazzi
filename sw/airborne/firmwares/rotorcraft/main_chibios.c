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
 * Main file for ChibiOS/RT Paparazzi rotocraft
 *
 * Includes both Paparazzi and ChibiOs files, threads aare static.
 *
 * @author {Michal Podhradsky, Calvin Coopmans}
 */

/*
 * Chibios includes
 */
#include "ch.h"
#include "hal.h"

/*
 * Paparazzi includes
 */
#include "led.h"
#include "mcu.h"

#define MODULES_C

#define ABI_C

#include "subsystems/datalink/downlink.h"
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
PRINT_CONFIG_MSG("USE_BARO_BOARD is TRUE: Reading onboard baro.")
#include "subsystems/sensors/baro.h"
#endif

#include "subsystems/electrical.h"

#include "firmwares/rotorcraft/autopilot.h"

#include "subsystems/radio_control.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance.h"

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "subsystems/ins.h"

#include "state.h"

#include "generated/modules.h"
#include "subsystems/abi.h"

/*
 * Telemetry defines
 */
#if DOWNLINK
#include "subsystems/datalink/telemetry.h"
static void send_chibios_info(void) {
  static uint32_t time_now = 0;
  time_now = chTimeNow()/CH_FREQUENCY;
  DOWNLINK_SEND_CHIBIOS_INFO(DefaultChannel, DefaultDevice,
    &core_free_memory,
    &time_now,
    &thread_counter,
    &cpu_frequency,
    &electrical.cpu_temp);
}
#endif

/*
 * Thread Area Definitions
 */
#define CH_THREAD_AREA_HEARTBEAT 128
#define CH_THREAD_AREA_FAILSAFE 256
#define CH_THREAD_AREA_ELECTRICAL 256
#define CH_THREAD_AREA_RADIO_CONTROL 256
#define CH_THREAD_AREA_RADIO_EVENT 512

/*
 * Thread Area Initialization
 */
static WORKING_AREA(wa_thd_heartbeat, CH_THREAD_AREA_HEARTBEAT);
static WORKING_AREA(wa_thd_failsafe, CH_THREAD_AREA_FAILSAFE);
static WORKING_AREA(wa_thd_electrical, CH_THREAD_AREA_ELECTRICAL);
static WORKING_AREA(wa_thd_radio_control, CH_THREAD_AREA_RADIO_CONTROL);
static WORKING_AREA(wa_thd_radio_event, CH_THREAD_AREA_RADIO_EVENT);

/*
 * Static Thread Definitions
 */
static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg);
static __attribute__((noreturn)) msg_t thd_failsafe(void *arg);
static __attribute__((noreturn)) msg_t thd_electrical(void *arg);
static __attribute__((noreturn)) msg_t thd_radio_control(void *arg);
static __attribute__((noreturn)) msg_t thd_radio_event(void *arg);

/*
 * Static Auxilliary Functions Definitions
 */
static inline void failsafe_check(void);


#if USE_GPS
static WORKING_AREA(wa_thd_gps_rx, CH_THREAD_AREA_GPS_RX);

/**
 * GPS callback
 *
 * The same functionality as on_gps_event() in
 * standard paparazzi
 */
void on_gps_event(void)
{
  ahrs_update_gps();
  ins_update_gps();
  chEvtBroadcastFlags(&eventGpsData, EVT_GPS_DATA);
}
#endif /* USE_GPS */

#ifdef USE_IMU
#ifdef INIT_IMU_THREAD
  static WORKING_AREA(wa_thd_imu_rx, CH_THREAD_AREA_IMU_RX);
#endif

/**
 * IMU Accel callback
 */
void on_accel_event( void )
{
  ImuScaleAccel(imu);
  if (ahrs.status != AHRS_UNINIT)
  {
    ahrs_update_accel();
  }
}

/**
 * IMU Gyro callback
 */
void on_gyro_event( void )
{
  ImuScaleGyro(imu);
  if (ahrs.status == AHRS_UNINIT)
  {
    ahrs_aligner_run();
    if (ahrs_aligner.status == AHRS_ALIGNER_LOCKED)
      ahrs_align();
  }
  else {
    ahrs_propagate();
    ins_propagate();
  }
}

/**
 * IMU Mag callback
 */
void on_mag_event(void)
{
  ImuScaleMag(imu);
#if USE_MAGNETOMETER
  if (ahrs.status == AHRS_RUNNING)
  {
    ahrs_update_mag();
  }
#endif
}
#endif /* USE_IMU */

#ifdef MODULES_C
  __attribute__((noreturn)) msg_t thd_modules_periodic(void *arg);
#endif

/* if PRINT_CONFIG is defined, print some config options */
PRINT_CONFIG_VAR(PERIODIC_FREQUENCY)

/*
 *  TELEMETRY_FREQUENCY is defined in generated/periodic_telemetry.h
 * defaults to 60Hz or set by TELEMETRY_FREQUENCY configure option in airframe file
 */
PRINT_CONFIG_VAR(TELEMETRY_FREQUENCY)

/*
 * MODULES_FREQUENCY is defined in generated/modules.h
 * according to main_freq parameter set for modules in airframe file
 */
PRINT_CONFIG_VAR(MODULES_FREQUENCY)

#ifndef BARO_PERIODIC_FREQUENCY
#define BARO_PERIODIC_FREQUENCY 100
#endif
PRINT_CONFIG_VAR(BARO_PERIODIC_FREQUENCY)

#ifndef FAILSAFE_FREQUENCY
#define FAILSAFE_FREQUENCY 20
#endif
PRINT_CONFIG_VAR(FAILSAFE_FREQUENCY)

#ifndef ELECTRICAL_PERIODIC_FREQ
#define ELECTRICAL_PERIODIC_FREQ 10
#endif
PRINT_CONFIG_VAR(ELECTRICAL_PERIODIC_FREQ)

#ifndef RADIO_CONTROL_FREQ
#define RADIO_CONTROL_FREQ 60
#endif
PRINT_CONFIG_VAR(RADIO_CONTROL_FREQ)

/**
 * HeartBeat & System Info
 *
 * Blinks LED and logs the cpu usage and other system info
 */
static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg)
{
  chRegSetThreadName("pprz_heartbeat");
  (void) arg;
  systime_t time = chTimeNow();
  static uint32_t last_idle_counter = 0;
  static uint32_t last_nb_sec = 0;

  while (TRUE)
  {
    time += S2ST(1);
    LED_TOGGLE(SYS_TIME_LED);
    sys_time.nb_sec++;

    if (autopilot_in_flight)
    {
      autopilot_flight_time++;
      datalink_time++;
    }

    core_free_memory = chCoreStatus();
    thread_counter = 0;

    Thread *tp;
    tp = chRegFirstThread();
    do
    {
      thread_counter++;
      if (tp ==chSysGetIdleThread())
      {
        idle_counter =  (uint32_t)tp->p_time;
      }
      tp = chRegNextThread(tp);
    }
    while (tp != NULL);

    cpu_counter = (idle_counter-last_idle_counter)/(sys_time.nb_sec-last_nb_sec);
    cpu_frequency = (1 - (float)cpu_counter/CH_FREQUENCY)*100;

    last_idle_counter = idle_counter;
    last_nb_sec = sys_time.nb_sec;

    chThdSleepUntil(time);
  }
}

/**
 * Failsafe Thread
 *
 * Replaces failsafe_periodic()
 *
 * TODO: ChibiOS/RT failsafe check (hypervisor thread)
 */
static __attribute__((noreturn)) msg_t thd_failsafe(void *arg)
{
  chRegSetThreadName("pprz_failsafe");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += US2ST(1000000/FAILSAFE_FREQUENCY);
    failsafe_check();
    chThdSleepUntil(time);
  }
}

/*
 * Electrical Periodic Thread
 *
 * Calls electrical_periodic()
 */
static __attribute__((noreturn)) msg_t thd_electrical(void *arg)
{
  chRegSetThreadName("pprz_electrical");
  (void) arg;
  electrical_init();
  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += US2ST(1000000/ELECTRICAL_PERIODIC_FREQ);
    electrical_periodic();
    chEvtBroadcastFlags(&eventElectricalData, EVT_ELECTRICAL_DATA);
    chThdSleepUntil(time);
  }
}

/*
 * Radio Control Periodic Thread
 *
 * Calls radio_control_periodic()
 */
static __attribute__((noreturn)) msg_t thd_radio_control(void *arg)
{
  chRegSetThreadName("pprz_radio_control");
  (void) arg;
  radio_control_init();
  systime_t time = chTimeNow();
  while (TRUE)
  {
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
static __attribute__((noreturn)) msg_t thd_radio_event(void *arg)
{
  chRegSetThreadName("pprz_radio_event");
  (void) arg;

  EventListener elRadioEvt;
  chEvtRegister(&eventPpmFrame, &elRadioEvt, EVT_PPM_FRAME);
  flagsmask_t rc_flags;

  while (TRUE)
  {
    chEvtWaitOne(EVENT_MASK(EVT_PPM_FRAME));
    rc_flags = chEvtGetAndClearFlags(&elRadioEvt);
    if (rc_flags & EVT_PPM_FRAME)
    {
      if (autopilot_rc)
      {
        RadioControlEvent(autopilot_on_rc_frame);
        chEvtBroadcastFlags(&eventRadioData, EVT_RADIO_DATA);
      }
    }
  }
}

#if USE_BARO_BOARD
#define CH_THREAD_AREA_BARO 512
static WORKING_AREA(wa_thd_baro, CH_THREAD_AREA_BARO);
static __attribute__((noreturn)) msg_t thd_baro(void *arg);

/**
 * Baro thread
 */
static __attribute__((noreturn)) msg_t thd_baro(void *arg)
{
  chRegSetThreadName("pprz_baro");
  (void) arg;

  baro_init();

  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += US2ST(1000000/BARO_PERIODIC_FREQUENCY);
    baro_periodic();
    chThdSleepUntil(time);
  }
}
#endif /* USE_BARO_BOARD */


#ifdef DOWNLINK
#define CH_THREAD_AREA_DOWNLINK_TX 1024
#define CH_THREAD_AREA_DOWNLINK_RX 1024
__attribute__((noreturn)) msg_t thd_telemetry_tx(void *arg);
__attribute__((noreturn)) msg_t thd_telemetry_rx(void *arg);

/**
 *  Telemetry TX thread
 *
 *  Replaces telemetryPeriodic()
 */
static WORKING_AREA(wa_thd_telemetry_tx, 1024);
__attribute__((noreturn)) msg_t thd_telemetry_tx(void *arg)
{
  chRegSetThreadName("pprz_telemetry_tx");
  (void) arg;

#if DATALINK == XBEE
  xbee_init();
#endif

#if DATALINK == UDP
  udp_init();
#endif

  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += US2ST(1000000/TELEMETRY_FREQUENCY);
    periodic_telemetry_send_Main();
    chThdSleepUntil(time);
  }
}

/**
 *  Telemetry RX thread
 *
 *  Replaces DatalinkEvent()
 *
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
    flags = chEvtGetAndClearFlags(&elTelemetryRx);
    ch_uart_receive_downlink(DOWNLINK_PORT, flags, parse_pprz, &pprz_tp);
    if (pprz_tp.trans.msg_received)
    {
      pprz_parse_payload(&(pprz_tp));
      pprz_tp.trans.msg_received = FALSE;
      dl_parse_msg();
      dl_msg_available = FALSE;
    }
  }
}
#endif /* DOWNLINK */

#ifdef MODULES_C
/**
 * Modules periodic tasks
 */
static WORKING_AREA(wa_thd_modules_periodic, 1024);
__attribute__((noreturn)) msg_t thd_modules_periodic(void *arg)
{
  chRegSetThreadName("pprz_modules_periodic");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += MS2ST(1000/MODULES_FREQUENCY);
    modules_periodic_task();
    chThdSleepUntil(time);
  }
}
#endif /* MODULES_C */


/**
 * Paparazzi failsafe thread
 */
static inline void failsafe_check(void)
{
  if (radio_control.status != RC_OK &&
    autopilot_mode != AP_MODE_KILL &&
    autopilot_mode != AP_MODE_NAV)
  {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }

#if FAILSAFE_ON_BAT_CRITICAL
  if (autopilot_mode != AP_MODE_KILL &&
    electrical.bat_critical)
  {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }
#endif

#if USE_GPS
  if (autopilot_mode == AP_MODE_NAV &&
    autopilot_motors_on &&
#if NO_GPS_LOST_WITH_RC_VALID
    radio_control.status != RC_OK &&
#endif
    GpsIsLost())
  {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }
#endif

  autopilot_check_in_flight(autopilot_motors_on);
}


/**
 * Thread initialization
 *
 * Done here, not in the submodules, so we don't
 * have to include ChibiOs headers to each submodule
 */
static void thread_init(void) {
chThdCreateStatic(wa_thd_heartbeat, sizeof(wa_thd_heartbeat), IDLEPRIO, thd_heartbeat, NULL);
chThdCreateStatic(wa_thd_electrical, sizeof(wa_thd_electrical), LOWPRIO, thd_electrical, NULL);
chThdCreateStatic(wa_thd_radio_control, sizeof(wa_thd_radio_control), NORMALPRIO, thd_radio_control, NULL);
chThdCreateStatic(wa_thd_radio_event, sizeof(wa_thd_radio_event), NORMALPRIO, thd_radio_event, NULL);

#if USE_BARO_BOARD
  chThdCreateStatic(wa_thd_baro, sizeof(wa_thd_baro),LOWPRIO, thd_baro, NULL);
#endif

#ifdef USE_IMU
#ifdef INIT_IMU_THREAD
  chThdCreateStatic(wa_thd_imu_rx, sizeof(wa_thd_imu_rx), HIGHPRIO, thd_imu_rx, NULL);
#else
  imu_init();
  #if USE_IMU_FLOAT
    imu_float_init();
  #endif
#endif /* INIT_IMU_THREAD */
#endif /* USE_IMU */

#ifdef DOWNLINK
  chThdCreateStatic(wa_thd_telemetry_tx, sizeof(wa_thd_telemetry_tx),NORMALPRIO, thd_telemetry_tx, NULL);
  chThdCreateStatic(wa_thd_telemetry_rx, sizeof(wa_thd_telemetry_rx),NORMALPRIO, thd_telemetry_rx, NULL);
#endif

#ifdef USE_GPS
  chThdCreateStatic(wa_thd_gps_rx, sizeof(wa_thd_gps_rx),NORMALPRIO, thd_gps_rx, &on_gps_event);
#endif

#ifdef MODULES_C
  chThdCreateStatic(wa_thd_modules_periodic, sizeof(wa_thd_modules_periodic),LOWPRIO, thd_modules_periodic, NULL);
#endif

chThdCreateStatic(wa_thd_failsafe, sizeof(wa_thd_failsafe), HIGHPRIO, thd_failsafe, NULL);
}


/**
 * Main loop
 *
 * Initializes system (both chibios and paparazzi),
 * then turns into main thread - main_periodic()
 */
int main(void)
{
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

  stateInit();

  actuators_init();

#if USE_MOTOR_MIXING
  motor_mixing_init();
#endif

  air_data_init();
#if USE_BARO_BOARD
  baro_init();
#endif

  ahrs_aligner_init();
  ahrs_init();

  ins_init();

  autopilot_init();

  modules_init();

  settings_init();

  mcu_int_enable();

  thread_init();
  
#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, "CHIBIOS_INFO", send_chibios_info);
#endif

  chThdSetPriority (HIGHPRIO);

  chThdSleep(MS2ST(1500));
  systime_t main_time = chTimeNow();
  while (TRUE)
  {
    main_time += US2ST(1000000/PERIODIC_FREQUENCY);
    imu_periodic();
    ImuEvent(on_gyro_event, on_accel_event, on_mag_event);
    autopilot_periodic();
    SetActuatorsFromCommands(commands, autopilot_mode);
    chThdSleepUntil(main_time);
  }

  return TRUE;
}
