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
#if USE_AHRS
#include "subsystems/ahrs.h"
#endif
#if USE_AHRS_ALIGNER
#include "subsystems/ahrs/ahrs_aligner.h"
#endif
#if USE_BARO_BOARD
#include "subsystems/sensors/baro.h"
PRINT_CONFIG_MSG_VALUE("USE_BARO_BOARD is TRUE, reading onboard baro: ", BARO_BOARD)
#endif
#include "subsystems/ins.h"


// autopilot & control
#include "state.h"
#include "firmwares/fixedwing/autopilot.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include CTRL_TYPE_H
#include "firmwares/fixedwing/nav.h"
#include "generated/flight_plan.h"
#ifdef TRAFFIC_INFO
#include "subsystems/navigation/traffic_info.h"
#endif

// datalink & telemetry
#if DATALINK || SITL
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"
#endif
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif
#include "subsystems/settings.h"

// modules & settings
#include "generated/modules.h"
#include "generated/settings.h"
#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
#include "rc_settings.h"
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
PRINT_CONFIG_VAR(NAVIGATION_FREQUENCY)
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

#if USE_BARO_BOARD
#ifndef BARO_PERIODIC_FREQUENCY
#define BARO_PERIODIC_FREQUENCY 50
#endif
PRINT_CONFIG_VAR(BARO_PERIODIC_FREQUENCY)
#endif


#if USE_IMU
#ifdef AHRS_PROPAGATE_FREQUENCY
#if (AHRS_PROPAGATE_FREQUENCY > PERIODIC_FREQUENCY)
#warning "PERIODIC_FREQUENCY should be least equal or greater than AHRS_PROPAGATE_FREQUENCY"
INFO_VALUE("it is recommended to configure in your airframe PERIODIC_FREQUENCY to at least ", AHRS_PROPAGATE_FREQUENCY)
#endif
#endif
#endif // USE_IMU

#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
static uint8_t  mcu1_ppm_cpt;
#endif


tid_t modules_tid;     ///< id for modules_periodic_task() timer
tid_t telemetry_tid;   ///< id for telemetry_periodic() timer
tid_t sensors_tid;     ///< id for sensors_task() timer
tid_t attitude_tid;    ///< id for attitude_loop() timer
tid_t navigation_tid;  ///< id for navigation_task() timer
tid_t monitor_tid;     ///< id for monitor_task() timer
#if USE_BARO_BOARD
tid_t baro_tid;          ///< id for baro_periodic() timer
#endif


/// @todo, properly implement or remove
#ifdef AHRS_TRIGGERED_ATTITUDE_LOOP
volatile uint8_t new_ins_attitude = 0;
static abi_event new_att_ev;
static void new_att_cb(uint8_t sender_id __attribute__((unused)),
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *gyro __attribute__((unused)))
{
  new_ins_attitude = 1;
}
#endif


void init_ap(void)
{
#ifndef SINGLE_MCU /** init done in main_fbw in single MCU */
  mcu_init();
#endif /* SINGLE_MCU */

#if defined(PPRZ_TRIG_INT_COMPR_FLASH)
  pprz_trig_int_init();
#endif

  /****** initialize and reset state interface ********/

  stateInit();

  /************* Sensors initialization ***************/
#if USE_GPS
  gps_init();
#endif

#if USE_IMU
  imu_init();
#endif

#if USE_AHRS_ALIGNER
  ahrs_aligner_init();
#endif

  ///@todo: properly implement/fix a triggered attitude loop
#ifdef AHRS_TRIGGERED_ATTITUDE_LOOP
  AbiBindMsgIMU_GYRO_INT32(ABI_BROADCAST, &new_att_ev, &new_att_cb);
#endif

#if USE_AHRS
  ahrs_init();
#endif

  ins_init();

#if USE_BARO_BOARD
  baro_init();
#endif

  /************* Links initialization ***************/
#if defined MCU_SPI_LINK || defined MCU_UART_LINK || defined MCU_CAN_LINK
  link_mcu_init();
#endif
#if USE_AUDIO_TELEMETRY
  audio_telemetry_init();
#endif

  /************ Internal status ***************/
  autopilot_init();
  h_ctl_init();
  v_ctl_init();
  nav_init();

  modules_init();

  settings_init();

  /**** start timers for periodic functions *****/
  sensors_tid = sys_time_register_timer(1. / PERIODIC_FREQUENCY, NULL);
  navigation_tid = sys_time_register_timer(1. / NAVIGATION_FREQUENCY, NULL);
  attitude_tid = sys_time_register_timer(1. / CONTROL_FREQUENCY, NULL);
  modules_tid = sys_time_register_timer(1. / MODULES_FREQUENCY, NULL);
  telemetry_tid = sys_time_register_timer(1. / TELEMETRY_FREQUENCY, NULL);
  monitor_tid = sys_time_register_timer(1.0, NULL);
#if USE_BARO_BOARD
  baro_tid = sys_time_register_timer(1. / BARO_PERIODIC_FREQUENCY, NULL);
#endif

  /** - start interrupt task */
  mcu_int_enable();

#if DOWNLINK
  downlink_init();
#endif

#if defined AEROCOMM_DATA_PIN
  IO0DIR |= _BV(AEROCOMM_DATA_PIN);
  IO0SET = _BV(AEROCOMM_DATA_PIN);
#endif

  /************ Multi-uavs status ***************/

#ifdef TRAFFIC_INFO
  traffic_info_init();
#endif

  /* set initial trim values.
   * these are passed to fbw via inter_mcu.
   */
  ap_state->command_roll_trim = COMMAND_ROLL_TRIM;
  ap_state->command_pitch_trim = COMMAND_PITCH_TRIM;
  ap_state->command_yaw_trim = COMMAND_YAW_TRIM;

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

#if USE_BARO_BOARD
  if (sys_time_check_and_ack_timer(baro_tid)) {
    baro_periodic();
  }
#endif

  if (sys_time_check_and_ack_timer(navigation_tid)) {
    navigation_task();
  }

#ifndef AHRS_TRIGGERED_ATTITUDE_LOOP
  if (sys_time_check_and_ack_timer(attitude_tid)) {
    attitude_loop();
  }
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


/******************** Interaction with FBW *****************************/

/** Update paparazzi mode.
 */
#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
static inline uint8_t pprz_mode_update(void)
{
  if ((pprz_mode != PPRZ_MODE_HOME &&
       pprz_mode != PPRZ_MODE_GPS_OUT_OF_ORDER)
#ifdef UNLOCKED_HOME_MODE
      || TRUE
#endif
     ) {
#ifndef RADIO_AUTO_MODE
    return ModeUpdate(pprz_mode, PPRZ_MODE_OF_PULSE(fbw_state->channels[RADIO_MODE]));
#else
    INFO("Using RADIO_AUTO_MODE to switch between AUTO1 and AUTO2.")
    /* If RADIO_AUTO_MODE is enabled mode swithing will be seperated between two switches/channels
     * RADIO_MODE will switch between PPRZ_MODE_MANUAL and any PPRZ_MODE_AUTO mode selected by RADIO_AUTO_MODE.
     *
     * This is mainly a cludge for entry level radios with no three-way switch but two available two-way switches which can be used.
     */
    if (PPRZ_MODE_OF_PULSE(fbw_state->channels[RADIO_MODE]) == PPRZ_MODE_MANUAL) {
      /* RADIO_MODE in MANUAL position */
      return ModeUpdate(pprz_mode, PPRZ_MODE_MANUAL);
    } else {
      /* RADIO_MODE not in MANUAL position.
       * Select AUTO mode bassed on RADIO_AUTO_MODE channel
       */
      return ModeUpdate(pprz_mode, (fbw_state->channels[RADIO_AUTO_MODE] > THRESHOLD2) ? PPRZ_MODE_AUTO2 : PPRZ_MODE_AUTO1);
    }
#endif // RADIO_AUTO_MODE
  } else {
    return false;
  }
}
#else // not RADIO_CONTROL
static inline uint8_t pprz_mode_update(void)
{
  return false;
}
#endif

static inline uint8_t mcu1_status_update(void)
{
  uint8_t new_status = fbw_state->status;
  if (mcu1_status != new_status) {
    bool changed = ((mcu1_status & MASK_FBW_CHANGED) != (new_status & MASK_FBW_CHANGED));
    mcu1_status = new_status;
    return changed;
  }
  return false;
}


/** Send back uncontrolled channels.
 */
static inline void copy_from_to_fbw(void)
{
#ifdef SetAutoCommandsFromRC
  SetAutoCommandsFromRC(ap_state->commands, fbw_state->channels);
#elif defined RADIO_YAW && defined COMMAND_YAW
  ap_state->commands[COMMAND_YAW] = fbw_state->channels[RADIO_YAW];
#endif
}

/** mode to enter when RC is lost in PPRZ_MODE_MANUAL or PPRZ_MODE_AUTO1 */
#ifndef RC_LOST_MODE
#define RC_LOST_MODE PPRZ_MODE_HOME
#endif

/**
 * Function to be called when a message from FBW is available
 */
static inline void telecommand_task(void)
{
  uint8_t mode_changed = false;
  copy_from_to_fbw();

  /* really_lost is true if we lost RC in MANUAL or AUTO1 */
  uint8_t really_lost = bit_is_set(fbw_state->status, STATUS_RADIO_REALLY_LOST) &&
    (pprz_mode == PPRZ_MODE_AUTO1 || pprz_mode == PPRZ_MODE_MANUAL);

  if (pprz_mode != PPRZ_MODE_HOME && pprz_mode != PPRZ_MODE_GPS_OUT_OF_ORDER && launch) {
    if (too_far_from_home) {
      pprz_mode = PPRZ_MODE_HOME;
      mode_changed = true;
    }
    if (really_lost) {
      pprz_mode = RC_LOST_MODE;
      mode_changed = true;
    }
  }
  if (bit_is_set(fbw_state->status, AVERAGED_CHANNELS_SENT)) {
    bool pprz_mode_changed = pprz_mode_update();
    mode_changed |= pprz_mode_changed;
#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS
    bool calib_mode_changed = RcSettingsModeUpdate(fbw_state->channels);
    rc_settings(calib_mode_changed || pprz_mode_changed);
    mode_changed |= calib_mode_changed;
#endif
  }
  mode_changed |= mcu1_status_update();
  if (mode_changed) { autopilot_send_mode(); }

#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
  /** In AUTO1 mode, compute roll setpoint and pitch setpoint from
   * \a RADIO_ROLL and \a RADIO_PITCH \n
   */
  if (pprz_mode == PPRZ_MODE_AUTO1) {
    /** Roll is bounded between [-AUTO1_MAX_ROLL;AUTO1_MAX_ROLL] */
    h_ctl_roll_setpoint = FLOAT_OF_PPRZ(fbw_state->channels[RADIO_ROLL], 0., AUTO1_MAX_ROLL);

    /** Pitch is bounded between [-AUTO1_MAX_PITCH;AUTO1_MAX_PITCH] */
    h_ctl_pitch_setpoint = FLOAT_OF_PPRZ(fbw_state->channels[RADIO_PITCH], 0., AUTO1_MAX_PITCH);
#if H_CTL_YAW_LOOP && defined RADIO_YAW
    /** Yaw is bounded between [-AUTO1_MAX_YAW_RATE;AUTO1_MAX_YAW_RATE] */
    h_ctl_yaw_rate_setpoint = FLOAT_OF_PPRZ(fbw_state->channels[RADIO_YAW], 0., AUTO1_MAX_YAW_RATE);
#endif
  } /** Else asynchronously set by \a h_ctl_course_loop() */

  /** In AUTO1, throttle comes from RADIO_THROTTLE
      In MANUAL, the value is copied to get it in the telemetry */
  if (pprz_mode == PPRZ_MODE_MANUAL || pprz_mode == PPRZ_MODE_AUTO1) {
    v_ctl_throttle_setpoint = fbw_state->channels[RADIO_THROTTLE];
  }
  /** else asynchronously set by v_ctl_climb_loop(); */

  mcu1_ppm_cpt = fbw_state->ppm_cpt;
#endif // RADIO_CONTROL


  vsupply = fbw_state->vsupply;
  current = fbw_state->current;
  energy = fbw_state->energy;

#ifdef RADIO_CONTROL
  /* the SITL check is a hack to prevent "automatic" launch in NPS */
#ifndef SITL
  if (!autopilot_flight_time) {
    if (pprz_mode == PPRZ_MODE_AUTO2 && fbw_state->channels[RADIO_THROTTLE] > THROTTLE_THRESHOLD_TAKEOFF) {
      launch = true;
    }
  }
#endif
#endif
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
    send_autopilot_version(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
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


#ifdef FAILSAFE_DELAY_WITHOUT_GPS
#define GpsTimeoutError (sys_time.nb_sec - gps.last_3dfix_time > FAILSAFE_DELAY_WITHOUT_GPS)
#endif

/**
 *  Compute desired_course
 */
void navigation_task(void)
{
#if defined FAILSAFE_DELAY_WITHOUT_GPS
  /** This section is used for the failsafe of GPS */
  static uint8_t last_pprz_mode;

  /** If aircraft is launched and is in autonomus mode, go into
      PPRZ_MODE_GPS_OUT_OF_ORDER mode (Failsafe) if we lost the GPS */
  if (launch) {
    if (GpsTimeoutError) {
      if (pprz_mode == PPRZ_MODE_AUTO2 || pprz_mode == PPRZ_MODE_HOME) {
        last_pprz_mode = pprz_mode;
        pprz_mode = PPRZ_MODE_GPS_OUT_OF_ORDER;
        autopilot_send_mode();
        gps_lost = true;
      }
    } else if (gps_lost) { /* GPS is ok */
      /** If aircraft was in failsafe mode, come back in previous mode */
      pprz_mode = last_pprz_mode;
      gps_lost = false;
      autopilot_send_mode();
    }
  }
#endif /* GPS && FAILSAFE_DELAY_WITHOUT_GPS */

  common_nav_periodic_task_4Hz();
  if (pprz_mode == PPRZ_MODE_HOME) {
    nav_home();
  } else if (pprz_mode == PPRZ_MODE_GPS_OUT_OF_ORDER) {
    nav_without_gps();
  } else {
    nav_periodic_task();
  }

#ifdef TCAS
  CallTCAS();
#endif

#if DOWNLINK && !defined PERIOD_NAVIGATION_Ap_0 // If not sent periodically (in default 0 mode)
  SEND_NAVIGATION(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif

  /* The nav task computes only nav_altitude. However, we are interested
     by desired_altitude (= nav_alt+alt_shift) in any case.
     So we always run the altitude control loop */
  if (v_ctl_mode == V_CTL_MODE_AUTO_ALT) {
    v_ctl_altitude_loop();
  }

  if (pprz_mode == PPRZ_MODE_AUTO2 || pprz_mode == PPRZ_MODE_HOME
      || pprz_mode == PPRZ_MODE_GPS_OUT_OF_ORDER) {
#ifdef H_CTL_RATE_LOOP
    /* Be sure to be in attitude mode, not roll */
    h_ctl_auto1_rate = false;
#endif
    if (lateral_mode >= LATERAL_MODE_COURSE) {
      h_ctl_course_loop();  /* aka compute nav_desired_roll */
    }

    // climb_loop(); //4Hz
  }
}


void attitude_loop(void)
{

  if (pprz_mode >= PPRZ_MODE_AUTO2) {
    if (v_ctl_mode == V_CTL_MODE_AUTO_THROTTLE) {
      v_ctl_throttle_setpoint = nav_throttle_setpoint;
      v_ctl_pitch_setpoint = nav_pitch;
    } else if (v_ctl_mode >= V_CTL_MODE_AUTO_CLIMB) {
      v_ctl_climb_loop();
    }

#if defined V_CTL_THROTTLE_IDLE
    Bound(v_ctl_throttle_setpoint, TRIM_PPRZ(V_CTL_THROTTLE_IDLE * MAX_PPRZ), MAX_PPRZ);
#endif

#ifdef V_CTL_POWER_CTL_BAT_NOMINAL
    if (vsupply > 0.) {
      v_ctl_throttle_setpoint *= 10. * V_CTL_POWER_CTL_BAT_NOMINAL / (float)vsupply;
      v_ctl_throttle_setpoint = TRIM_UPPRZ(v_ctl_throttle_setpoint);
    }
#endif

    // Copy the pitch setpoint from the guidance to the stabilization control
    h_ctl_pitch_setpoint = v_ctl_pitch_setpoint;
    Bound(h_ctl_pitch_setpoint, H_CTL_PITCH_MIN_SETPOINT, H_CTL_PITCH_MAX_SETPOINT);
    if (kill_throttle || (!autopilot_flight_time && !launch)) {
      v_ctl_throttle_setpoint = 0;
    }
  }

  h_ctl_attitude_loop(); /* Set  h_ctl_aileron_setpoint & h_ctl_elevator_setpoint */
  v_ctl_throttle_slew();
  ap_state->commands[COMMAND_THROTTLE] = v_ctl_throttle_slewed;
  ap_state->commands[COMMAND_ROLL] = -h_ctl_aileron_setpoint;
  ap_state->commands[COMMAND_PITCH] = h_ctl_elevator_setpoint;
#if H_CTL_YAW_LOOP && defined COMMAND_YAW
  ap_state->commands[COMMAND_YAW] = h_ctl_rudder_setpoint;
#endif
#if H_CTL_CL_LOOP && defined COMMAND_CL
  ap_state->commands[COMMAND_CL] = h_ctl_flaps_setpoint;
#endif

#if defined MCU_SPI_LINK || defined MCU_UART_LINK || defined MCU_CAN_LINK
  link_mcu_send();
#elif defined INTER_MCU && defined SINGLE_MCU
  /**Directly set the flag indicating to FBW that shared buffer is available*/
  inter_mcu_received_ap = true;
#endif

}


/** Run at PERIODIC_FREQUENCY (60Hz if not defined) */
void sensors_task(void)
{
#if USE_IMU
  imu_periodic();
#endif // USE_IMU

  //FIXME: this is just a kludge
#if USE_AHRS && defined SITL && !USE_NPS
  update_ahrs_from_sim();
#endif

#if USE_GPS
  gps_periodic_check();
#endif

  //FIXME: temporary hack, remove me
#ifdef InsPeriodic
  InsPeriodic();
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
  if (autopilot_flight_time) {
    autopilot_flight_time++;
  }
#if defined DATALINK || defined SITL
  datalink_time++;
#endif

  static uint8_t t = 0;
  if (vsupply < CATASTROPHIC_BAT_LEVEL * 10) {
    t++;
  } else {
    t = 0;
  }
  kill_throttle |= (t >= CATASTROPHIC_BAT_KILL_DELAY);
  kill_throttle |= launch && (dist2_to_home > Square(KILL_MODE_DISTANCE));

  if (!autopilot_flight_time &&
      stateGetHorizontalSpeedNorm_f() > MIN_SPEED_FOR_TAKEOFF) {
    autopilot_flight_time = 1;
    launch = true; /* Not set in non auto launch */
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

#if USE_IMU
  ImuEvent();
#endif

#ifdef InsEvent
  TODO("calling InsEvent, remove me..")
  InsEvent();
#endif

#if USE_GPS
  GpsEvent();
#endif /* USE_GPS */

#if USE_BARO_BOARD
  BaroEvent();
#endif

  DatalinkEvent();


#if defined MCU_SPI_LINK || defined MCU_UART_LINK
  link_mcu_event_task();
#endif

  if (inter_mcu_received_fbw) {
    /* receive radio control task from fbw */
    inter_mcu_received_fbw = false;
    telecommand_task();
  }

  modules_event_task();

#ifdef AHRS_TRIGGERED_ATTITUDE_LOOP
  if (new_ins_attitude > 0) {
    attitude_loop();
    new_ins_attitude = 0;
  }
#endif

} /* event_task_ap() */

