/*
 * Copyright (C) 2008-2012 The Paparazzi Team
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/rotorcraft/autopilot_static.c
 *
 * Static autopilot implementation.
 *
 */

#include "autopilot.h"
#include "autopilot_arming.h"

#include "subsystems/radio_control.h"
#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#include "subsystems/electrical.h"
#include "subsystems/settings.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_none.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#if USE_STABILIZATION_RATE
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#endif

#include "firmwares/rotorcraft/autopilot_guided.h"

#include "generated/settings.h"

#if USE_GPS
#include "subsystems/gps.h"
#else
#if NO_GPS_NEEDED_FOR_NAV
#define GpsIsLost() FALSE
#else
#define GpsIsLost() TRUE
#endif
#endif

/* Geofence exceptions */
#include "modules/nav/nav_geofence.h"

/** Mode that is set when the plane is really too far from home */
#ifndef FAILSAFE_MODE_TOO_FAR_FROM_HOME
#define FAILSAFE_MODE_TOO_FAR_FROM_HOME AP_MODE_FAILSAFE
#endif


#ifndef MODE_STARTUP
#define MODE_STARTUP AP_MODE_KILL
PRINT_CONFIG_MSG("Using default AP_MODE_KILL as MODE_STARTUP")
#endif

#ifndef UNLOCKED_HOME_MODE
#if MODE_AUTO1 == AP_MODE_HOME
#define UNLOCKED_HOME_MODE TRUE
PRINT_CONFIG_MSG("Enabled UNLOCKED_HOME_MODE since MODE_AUTO1 is AP_MODE_HOME")
#elif MODE_AUTO2 == AP_MODE_HOME
#define UNLOCKED_HOME_MODE TRUE
PRINT_CONFIG_MSG("Enabled UNLOCKED_HOME_MODE since MODE_AUTO2 is AP_MODE_HOME")
#else
#define UNLOCKED_HOME_MODE FALSE
#endif
#endif

#if MODE_MANUAL == AP_MODE_NAV
#error "MODE_MANUAL mustn't be AP_MODE_NAV"
#endif


void autopilot_static_init(void)
{
  /* Mode is finally set by autopilot_static_set_mode if MODE_STARTUP is not KILL.
   * For autopilot_static_set_mode to do anything, the requested mode needs to differ
   * from previous mode, so we set it to a safe KILL first.
   */
  autopilot.mode = AP_MODE_KILL;

  /* set startup mode, propagates through to guidance h/v */
  autopilot_static_set_mode(MODE_STARTUP);

  /* init arming */
  autopilot_arming_init();
}


#define NAV_PRESCALER (PERIODIC_FREQUENCY / NAV_FREQ)
void autopilot_static_periodic(void)
{

  RunOnceEvery(NAV_PRESCALER, compute_dist2_to_home());

  if (autopilot_in_flight() && autopilot.mode == AP_MODE_NAV) {
    if (too_far_from_home || datalink_lost() || higher_than_max_altitude()) {
      if (dist2_to_home > failsafe_mode_dist2) {
        autopilot_static_set_mode(FAILSAFE_MODE_TOO_FAR_FROM_HOME);
      } else {
        autopilot_static_set_mode(AP_MODE_HOME);
      }
    }
  }

  if (autopilot.mode == AP_MODE_HOME) {
    RunOnceEvery(NAV_PRESCALER, nav_home());
  } else {
    // otherwise always call nav_periodic_task so that carrot is always updated in GCS for other modes
    RunOnceEvery(NAV_PRESCALER, nav_periodic_task());
  }


  /* If in FAILSAFE mode and either already not in_flight anymore
   * or just "detected" ground, go to KILL mode.
   */
  if (autopilot.mode == AP_MODE_FAILSAFE) {
    if (!autopilot_in_flight()) {
      autopilot_static_set_mode(AP_MODE_KILL);
    }

#if FAILSAFE_GROUND_DETECT
    INFO("Using FAILSAFE_GROUND_DETECT: KILL")
    if (autopilot.ground_detected) {
      autopilot_static_set_mode(AP_MODE_KILL);
    }
#endif
  }

  /* Reset ground detection _after_ running flight plan
   */
  if (!autopilot_in_flight()) {
    autopilot.ground_detected = false;
    autopilot.detect_ground_once = false;
  }

  /* Set fixed "failsafe" commands from airframe file if in KILL mode.
   * If in FAILSAFE mode, run normal loops with failsafe attitude and
   * downwards velocity setpoints.
   */
  if (autopilot.mode == AP_MODE_KILL) {
    SetCommands(commands_failsafe);
  } else {
    guidance_v_run(autopilot_in_flight());
    guidance_h_run(autopilot_in_flight());
    SetRotorcraftCommands(stabilization_cmd, autopilot.in_flight, autopilot.motors_on);
  }
  autopilot.throttle = commands[COMMAND_THRUST];

}

/** AP mode setting handler
 *
 * Checks RC status before calling autopilot_static_set_mode function
 */
void autopilot_static_SetModeHandler(float mode)
{
  if (mode == AP_MODE_KILL || mode == AP_MODE_FAILSAFE || mode == AP_MODE_HOME) {
    // safety modes are always accessible via settings
    autopilot_static_set_mode(mode);
  } else {
    if (radio_control.status != RC_OK &&
        (mode == AP_MODE_NAV || mode == AP_MODE_GUIDED || mode == AP_MODE_FLIP || mode == AP_MODE_MODULE)) {
      // without RC, only nav-like modes are accessible
      autopilot_static_set_mode(mode);
    }
  }
  // with RC, other modes can only be changed from the RC
}


void autopilot_static_set_mode(uint8_t new_autopilot_mode)
{

  /* force startup mode (default is kill) as long as AHRS is not aligned */
  if (!ap_ahrs_is_aligned()) {
    new_autopilot_mode = MODE_STARTUP;
  }

  if (new_autopilot_mode != autopilot.mode) {
    /* horizontal mode */
    switch (new_autopilot_mode) {
      case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
        stabilization_attitude_set_failsafe_setpoint();
        guidance_h_mode_changed(GUIDANCE_H_MODE_ATTITUDE);
        break;
#endif
      case AP_MODE_KILL:
        autopilot_set_in_flight(false);
        guidance_h_mode_changed(GUIDANCE_H_MODE_KILL);
        break;
      case AP_MODE_RC_DIRECT:
        guidance_h_mode_changed(GUIDANCE_H_MODE_RC_DIRECT);
        break;
      case AP_MODE_RATE_RC_CLIMB:
      case AP_MODE_RATE_DIRECT:
      case AP_MODE_RATE_Z_HOLD:
#if USE_STABILIZATION_RATE
        guidance_h_mode_changed(GUIDANCE_H_MODE_RATE);
#else
        return;
#endif
        break;
      case AP_MODE_ATTITUDE_RC_CLIMB:
      case AP_MODE_ATTITUDE_DIRECT:
      case AP_MODE_ATTITUDE_CLIMB:
      case AP_MODE_ATTITUDE_Z_HOLD:
        guidance_h_mode_changed(GUIDANCE_H_MODE_ATTITUDE);
        break;
      case AP_MODE_FORWARD:
        guidance_h_mode_changed(GUIDANCE_H_MODE_FORWARD);
        break;
      case AP_MODE_CARE_FREE_DIRECT:
        guidance_h_mode_changed(GUIDANCE_H_MODE_CARE_FREE);
        break;
      case AP_MODE_HOVER_DIRECT:
      case AP_MODE_HOVER_CLIMB:
      case AP_MODE_HOVER_Z_HOLD:
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
        break;
      case AP_MODE_HOME:
      case AP_MODE_NAV:
        guidance_h_mode_changed(GUIDANCE_H_MODE_NAV);
        break;
      case AP_MODE_MODULE:
#ifdef GUIDANCE_H_MODE_MODULE_SETTING
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE_SETTING);
#endif
        break;
      case AP_MODE_FLIP:
        guidance_h_mode_changed(GUIDANCE_H_MODE_FLIP);
        break;
      case AP_MODE_GUIDED:
        guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
        break;
      default:
        break;
    }
    /* vertical mode */
    switch (new_autopilot_mode) {
      case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
        guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
        guidance_v_zd_sp = SPEED_BFP_OF_REAL(FAILSAFE_DESCENT_SPEED);
        break;
#endif
      case AP_MODE_KILL:
        autopilot_set_motors_on(FALSE);
        stabilization_cmd[COMMAND_THRUST] = 0;
        guidance_v_mode_changed(GUIDANCE_V_MODE_KILL);
        break;
      case AP_MODE_RC_DIRECT:
      case AP_MODE_RATE_DIRECT:
      case AP_MODE_ATTITUDE_DIRECT:
      case AP_MODE_HOVER_DIRECT:
      case AP_MODE_CARE_FREE_DIRECT:
      case AP_MODE_FORWARD:
        guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
        break;
      case AP_MODE_RATE_RC_CLIMB:
      case AP_MODE_ATTITUDE_RC_CLIMB:
        guidance_v_mode_changed(GUIDANCE_V_MODE_RC_CLIMB);
        break;
      case AP_MODE_ATTITUDE_CLIMB:
      case AP_MODE_HOVER_CLIMB:
        guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
        break;
      case AP_MODE_RATE_Z_HOLD:
      case AP_MODE_ATTITUDE_Z_HOLD:
      case AP_MODE_HOVER_Z_HOLD:
        guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
        break;
      case AP_MODE_HOME:
      case AP_MODE_NAV:
        guidance_v_mode_changed(GUIDANCE_V_MODE_NAV);
        break;
      case AP_MODE_MODULE:
#ifdef GUIDANCE_V_MODE_MODULE_SETTING
        guidance_v_mode_changed(GUIDANCE_V_MODE_MODULE_SETTING);
#endif
        break;
      case AP_MODE_FLIP:
        guidance_v_mode_changed(GUIDANCE_V_MODE_FLIP);
        break;
      case AP_MODE_GUIDED:
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        break;
      default:
        break;
    }
    //if switching to rate mode but rate mode is not defined, the function returned
    autopilot.mode = new_autopilot_mode;
  }
}


void autopilot_static_set_motors_on(bool motors_on)
{
  if (autopilot.mode != AP_MODE_KILL && ap_ahrs_is_aligned() && motors_on) {
    autopilot.motors_on = true;
  } else {
    autopilot.motors_on = false;
  }
  autopilot_arming_set(autopilot.motors_on);
}

void autopilot_static_on_rc_frame(void)
{

  if (kill_switch_is_on()) {
    autopilot_static_set_mode(AP_MODE_KILL);
  } else {
#ifdef RADIO_AUTO_MODE
    INFO("Using RADIO_AUTO_MODE to switch between AUTO1 and AUTO2.")
    uint8_t new_autopilot_mode = ap_mode_of_two_switches();
#else
#ifdef RADIO_MODE_2x3
    uint8_t new_autopilot_mode = ap_mode_of_3x2way_switch();
#else
    uint8_t new_autopilot_mode = ap_mode_of_3way_switch();
#endif
#endif

    /* don't enter NAV mode if GPS is lost (this also prevents mode oscillations) */
    if (!(new_autopilot_mode == AP_MODE_NAV && GpsIsLost())) {
      /* always allow to switch to manual */
      if (new_autopilot_mode == MODE_MANUAL) {
        autopilot_static_set_mode(new_autopilot_mode);
      }
      /* if in HOME mode, don't allow switching to non-manual modes */
      else if (((autopilot.mode != AP_MODE_HOME) && (autopilot.mode != AP_MODE_FAILSAFE))

#if UNLOCKED_HOME_MODE
               /* Allowed to leave home mode when UNLOCKED_HOME_MODE */
               || !too_far_from_home
#endif
              ) {
        autopilot_static_set_mode(new_autopilot_mode);
      }
    }
  }

  /* an arming sequence is used to start/stop motors.
   * only allow arming if ahrs is aligned
   */
  if (ap_ahrs_is_aligned()) {
    autopilot_arming_check_motors_on();
    autopilot.kill_throttle = ! autopilot.motors_on;
  } else {
    autopilot.arming_status = AP_ARMING_STATUS_AHRS_NOT_ALLIGNED;
  }

  /* if not in FAILSAFE or HOME mode, read RC and set commands accordingly */
  if (autopilot.mode != AP_MODE_FAILSAFE && autopilot.mode != AP_MODE_HOME) {

    /* if there are some commands that should always be set from RC, do it */
#ifdef SetAutoCommandsFromRC
    SetAutoCommandsFromRC(commands, radio_control.values);
#endif

    /* if not in NAV_MODE set commands from the rc */
#ifdef SetCommandsFromRC
    if (autopilot.mode != AP_MODE_NAV) {
      SetCommandsFromRC(commands, radio_control.values);
    }
#endif

    guidance_v_read_rc();
    guidance_h_read_rc(autopilot_in_flight());
  }

}

