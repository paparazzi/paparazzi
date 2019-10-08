/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file firmwares/fixedwing/autopilot.c
 *
 * Fixedwing autopilot inititalization.
 *
 */

#include "autopilot.h"
#include "firmwares/fixedwing/autopilot_static.h"

#include "inter_mcu.h"
#include "link_mcu.h"
#include "state.h"
#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "generated/flight_plan.h"

/* Geofence exceptions */
#include "modules/nav/nav_geofence.h"

#if USE_GPS
#include "subsystems/gps.h"
#endif
static bool gps_lost;

#include "subsystems/datalink/downlink.h"

#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
static uint8_t  mcu1_ppm_cpt;
#endif

static inline uint8_t pprz_mode_update(void);
static inline uint8_t mcu1_status_update(void);
static inline void copy_from_to_fbw(void);

/** mode to enter when RC is lost in AP_MODE_MANUAL or AP_MODE_AUTO1 */
#ifndef RC_LOST_MODE
#define RC_LOST_MODE AP_MODE_HOME
#endif


/// @todo, properly implement or remove
#ifdef AHRS_TRIGGERED_ATTITUDE_LOOP
#include "subsystems/abi.h"
volatile uint8_t new_ins_attitude = 0;
static abi_event new_att_ev;
static void new_att_cb(uint8_t sender_id __attribute__((unused)),
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *gyro __attribute__((unused)))
{
  new_ins_attitude = 1;
}

/** use AP event in case of attitude control triggered by INS data
 */
void autopilot_event(void)
{
  if (new_ins_attitude > 0) {
    attitude_loop();
    new_ins_attitude = 0;
  }
}
#endif


/** Static autopilot API
 */

void autopilot_static_init(void)
{
  autopilot.mode = AP_MODE_AUTO2;

  lateral_mode = LATERAL_MODE_MANUAL;
  gps_lost = false;

  ///@todo: properly implement/fix a triggered attitude loop
#ifdef AHRS_TRIGGERED_ATTITUDE_LOOP
  AbiBindMsgIMU_GYRO_INT32(ABI_BROADCAST, &new_att_ev, &new_att_cb);
#endif

}

void autopilot_static_periodic(void)
{
  attitude_loop();
}

/**
 * Function to be called when a message from FBW is available
 */
void autopilot_static_on_rc_frame(void)
{
  uint8_t mode_changed = false;
  copy_from_to_fbw();

  /* rc_lost_while_in_use is true if we lost RC in MANUAL or AUTO1 */
  uint8_t rc_lost_while_in_use = bit_is_set(imcu_get_status(), STATUS_RADIO_REALLY_LOST) &&
                        (autopilot_get_mode() == AP_MODE_AUTO1 || autopilot_get_mode() == AP_MODE_MANUAL);

  /* RC_LOST_MODE defaults to AP_MODE_HOME, but it can also be set to NAV_MODE_NAV or MANUAL when the RC receiver is well configured to send failsafe commands */
  if (rc_lost_while_in_use) { // Always: no exceptions!
    mode_changed = autopilot_set_mode(RC_LOST_MODE);
  }

#ifdef RADIO_KILL_SWITCH
  if (imcu_get_radio(RADIO_KILL_SWITCH) < MIN_PPRZ / 2) {
    autopilot_set_kill_throttle(true);
  }
#endif

  /* If in-flight, with good GPS but too far, then activate HOME mode
   * In MANUAL with good RC, FBW will allow to override. */
  if (autopilot_get_mode() != AP_MODE_HOME && autopilot_get_mode() != AP_MODE_GPS_OUT_OF_ORDER && autopilot.launch) {
    if (too_far_from_home || datalink_lost() || higher_than_max_altitude()) {
      mode_changed = autopilot_set_mode(AP_MODE_HOME);
    }
  }
  if (bit_is_set(imcu_get_status(), AVERAGED_CHANNELS_SENT)) {
    bool pprz_mode_changed = pprz_mode_update();
    mode_changed |= pprz_mode_changed;
#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS
    PPRZ_MUTEX_LOCK(fbw_state_mtx);
    bool calib_mode_changed = RcSettingsModeUpdate(fbw_state->channels);
    PPRZ_MUTEX_UNLOCK(fbw_state_mtx);
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
  if (autopilot_get_mode() == AP_MODE_AUTO1) {
    /** Roll is bounded between [-AUTO1_MAX_ROLL;AUTO1_MAX_ROLL] */
    h_ctl_roll_setpoint = FLOAT_OF_PPRZ(imcu_get_radio(RADIO_ROLL), 0., AUTO1_MAX_ROLL);

    /** Pitch is bounded between [-AUTO1_MAX_PITCH;AUTO1_MAX_PITCH] */
    h_ctl_pitch_setpoint = FLOAT_OF_PPRZ(imcu_get_radio(RADIO_PITCH), 0., AUTO1_MAX_PITCH);
#if H_CTL_YAW_LOOP && defined RADIO_YAW
    /** Yaw is bounded between [-AUTO1_MAX_YAW_RATE;AUTO1_MAX_YAW_RATE] */
    h_ctl_yaw_rate_setpoint = FLOAT_OF_PPRZ(imcu_get_radio(RADIO_YAW), 0., AUTO1_MAX_YAW_RATE);
#endif
  } /** Else asynchronously set by \a h_ctl_course_loop() */

  /** In AUTO1, throttle comes from RADIO_THROTTLE
      In MANUAL, the value is copied to get it in the telemetry */
  if (autopilot_get_mode() == AP_MODE_MANUAL || autopilot_get_mode() == AP_MODE_AUTO1) {
    v_ctl_throttle_setpoint = imcu_get_radio(RADIO_THROTTLE);
  }
  /** else asynchronously set by v_ctl_climb_loop(); */

  mcu1_ppm_cpt = imcu_get_ppm_cpt();
#endif // RADIO_CONTROL

  // update electrical from FBW
  imcu_get_electrical(&ap_electrical);

#ifdef RADIO_CONTROL
  /* the SITL check is a hack to prevent "automatic" launch in NPS */
#ifndef SITL
  if (!autopilot.flight_time) {
    if (autopilot_get_mode() == AP_MODE_AUTO2 && imcu_get_radio(RADIO_THROTTLE) > THROTTLE_THRESHOLD_TAKEOFF) {
      autopilot.launch = true;
    }
  }
#endif
#endif
}

void autopilot_static_set_mode(uint8_t new_autopilot_mode)
{
  if (new_autopilot_mode != autopilot.mode) {
    autopilot.mode = new_autopilot_mode;
  }
}

void autopilot_static_SetModeHandler(float new_autopilot_mode)
{
  autopilot_static_set_mode(new_autopilot_mode);
}

void autopilot_static_set_motors_on(bool motors_on __attribute__((unused)))
{
  // Do nothing on fixedwing ?
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
      AP_MODE_GPS_OUT_OF_ORDER mode (Failsafe) if we lost the GPS */
  if (autopilot.launch) {
    if (GpsTimeoutError) {
      if (autopilot_get_mode() == AP_MODE_AUTO2 || autopilot_get_mode() == AP_MODE_HOME) {
        last_pprz_mode = autopilot_get_mode();
        autopilot_set_mode(AP_MODE_GPS_OUT_OF_ORDER);
        autopilot_send_mode();
        gps_lost = true;
      }
    } else if (gps_lost) { /* GPS is ok */
      /** If aircraft was in failsafe mode, come back in previous mode */
      autopilot_set_mode(last_pprz_mode);
      gps_lost = false;
      autopilot_send_mode();
    }
  }
#endif /* GPS && FAILSAFE_DELAY_WITHOUT_GPS */

  common_nav_periodic_task_4Hz();
  if (autopilot_get_mode() == AP_MODE_HOME) {
    nav_home();
  } else if (autopilot_get_mode() == AP_MODE_GPS_OUT_OF_ORDER) {
    nav_without_gps();
  } else {
    nav_periodic_task();
  }

#ifdef TCAS
  callTCAS();
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

  if (autopilot_get_mode() == AP_MODE_AUTO2 || autopilot_get_mode() == AP_MODE_HOME
      || autopilot_get_mode() == AP_MODE_GPS_OUT_OF_ORDER) {
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

  if (autopilot_get_mode() >= AP_MODE_AUTO2) {
#if CTRL_VERTICAL_LANDING
    if (v_ctl_mode == V_CTL_MODE_LANDING) {
      v_ctl_landing_loop();
    } else {
#endif
      if (v_ctl_mode == V_CTL_MODE_AUTO_THROTTLE) {
        v_ctl_throttle_setpoint = nav_throttle_setpoint;
        v_ctl_pitch_setpoint = nav_pitch;
      } else {
        if (v_ctl_mode >= V_CTL_MODE_AUTO_CLIMB) {
          v_ctl_climb_loop();
        } /* v_ctl_mode >= V_CTL_MODE_AUTO_CLIMB */
      } /* v_ctl_mode == V_CTL_MODE_AUTO_THROTTLE */
#if CTRL_VERTICAL_LANDING
    } /* v_ctl_mode == V_CTL_MODE_LANDING */
#endif

#if defined V_CTL_THROTTLE_IDLE
    Bound(v_ctl_throttle_setpoint, TRIM_PPRZ(V_CTL_THROTTLE_IDLE * MAX_PPRZ), MAX_PPRZ);
#endif

#ifdef V_CTL_POWER_CTL_BAT_NOMINAL
    if (ap_electrical.vsupply > 0.) {
      v_ctl_throttle_setpoint *= V_CTL_POWER_CTL_BAT_NOMINAL / ap_electrical.vsupply;
      v_ctl_throttle_setpoint = TRIM_UPPRZ(v_ctl_throttle_setpoint);
    }
#endif

    // Copy the pitch setpoint from the guidance to the stabilization control
    h_ctl_pitch_setpoint = v_ctl_pitch_setpoint;
    Bound(h_ctl_pitch_setpoint, H_CTL_PITCH_MIN_SETPOINT, H_CTL_PITCH_MAX_SETPOINT);
    if (autopilot.kill_throttle || (!autopilot.flight_time && !autopilot.launch)) {
      v_ctl_throttle_setpoint = 0;
    }
  }

  h_ctl_attitude_loop(); /* Set  h_ctl_aileron_setpoint & h_ctl_elevator_setpoint */
  v_ctl_throttle_slew();
  PPRZ_MUTEX_LOCK(ap_state_mtx);
  AP_COMMAND_SET_THROTTLE(v_ctl_throttle_slewed);
  AP_COMMAND_SET_ROLL(-h_ctl_aileron_setpoint);
  AP_COMMAND_SET_PITCH(h_ctl_elevator_setpoint);
  AP_COMMAND_SET_YAW(h_ctl_rudder_setpoint);
  AP_COMMAND_SET_CL(h_ctl_flaps_setpoint);
  PPRZ_MUTEX_UNLOCK(ap_state_mtx);

#if defined MCU_SPI_LINK || defined MCU_UART_LINK || defined MCU_CAN_LINK
  link_mcu_send();
#elif defined INTER_MCU && defined SINGLE_MCU
  /**Directly set the flag indicating to FBW that shared buffer is available*/
  inter_mcu_received_ap = true;
#endif

}

/******************** Interaction with FBW *****************************/

/** Update paparazzi mode.
 */
#if defined RADIO_CONTROL || defined RADIO_CONTROL_AUTO1
static inline uint8_t pprz_mode_update(void)
{
  if ((autopilot_get_mode() != AP_MODE_HOME &&
       autopilot_get_mode() != AP_MODE_GPS_OUT_OF_ORDER)
#ifdef UNLOCKED_HOME_MODE
      || true
#endif
     ) {
#ifndef RADIO_AUTO_MODE
    return autopilot_set_mode(AP_MODE_OF_PULSE(imcu_get_radio(RADIO_MODE)));
#else
    INFO("Using RADIO_AUTO_MODE to switch between AUTO1 and AUTO2.")
    /* If RADIO_AUTO_MODE is enabled mode swithing will be seperated between two switches/channels
     * RADIO_MODE will switch between AP_MODE_MANUAL and any AP_MODE_AUTO mode selected by RADIO_AUTO_MODE.
     *
     * This is mainly a cludge for entry level radios with no three-way switch but two available two-way switches which can be used.
     */
    if (AP_MODE_OF_PULSE(imcu_get_radio(RADIO_MODE)) == AP_MODE_MANUAL) {
      /* RADIO_MODE in MANUAL position */
      return autopilot_set_mode(AP_MODE_MANUAL);
    } else {
      /* RADIO_MODE not in MANUAL position.
       * Select AUTO mode bassed on RADIO_AUTO_MODE channel
       */
      return autopilot_set_mode((imcu_get_radio(RADIO_AUTO_MODE) > THRESHOLD2) ? AP_MODE_AUTO2 : AP_MODE_AUTO1);
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
  uint8_t new_status = imcu_get_status();
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
  PPRZ_MUTEX_LOCK(fbw_state_mtx);
  PPRZ_MUTEX_LOCK(ap_state_mtx);
#ifdef SetAutoCommandsFromRC
  SetAutoCommandsFromRC(ap_state->commands, fbw_state->channels);
#elif defined RADIO_YAW && defined COMMAND_YAW
  ap_state->commands[COMMAND_YAW] = fbw_state->channels[RADIO_YAW];
#endif
  PPRZ_MUTEX_UNLOCK(ap_state_mtx);
  PPRZ_MUTEX_UNLOCK(fbw_state_mtx);
}
