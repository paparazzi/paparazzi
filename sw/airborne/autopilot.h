/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2016-2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file autopilot.h
 *
 * Core autopilot interface common to all firmwares.
 * Using either static or generated autopilot logic,
 * which depends on the firmware.
 *
 */

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"
#include "state.h"
#include "autopilot_utils.h"

// include firmware specific header
#include "autopilot_firmware.h"

// include static or generated autopilot
// static version by default
#ifndef USE_GENERATED_AUTOPILOT
#define USE_GENERATED_AUTOPILOT FALSE
#endif

#if USE_GENERATED_AUTOPILOT
#include "autopilot_generated.h"
#else
#include "autopilot_static.h"
#endif

/** PPRZ Autopilot structure definition
 */
struct pprz_autopilot {
  uint8_t mode;             ///< current autopilot mode
  uint8_t mode_auto2;       ///< FIXME hide this in a private part ?
  uint16_t flight_time;     ///< flight time in seconds
  pprz_t throttle;          ///< throttle level as will be displayed in GCS
  uint8_t arming_status;    ///< arming status
  bool motors_on;           ///< motor status
  bool kill_throttle;       ///< allow autopilot to use throttle
  bool in_flight;           ///< in flight status
  bool launch;              ///< request launch
  bool use_rc;              ///< enable/disable RC input
  bool power_switch;        ///< enable/disable power from power switch (if any)
  bool ground_detected;     ///< automatic detection of landing
  bool detect_ground_once;  ///< enable automatic detection of ground (one shot)
};


/** Global autopilot structure
 */
extern struct pprz_autopilot autopilot;

/** Autopilot initialization function
 */
extern void autopilot_init(void);

/** Autopilot periodic call at PERIODIC_FREQUENCY
 */
extern void autopilot_periodic(void);

/** Autopilot event check function
 */
extern void autopilot_event(void);

/** Autopilot RC input event hadler
 */
extern void autopilot_on_rc_frame(void);

/** Autopilot periodic failsafe checks
 */
extern void autopilot_failsafe_checks(void);

/** Set new autopilot mode
 *
 * @param[in] new_autopilot_mode new mode to set
 * @return true if mode has changed
 */
extern bool autopilot_set_mode(uint8_t new_autopilot_mode);

/** Handler for setter function with dl_setting
 */
extern void autopilot_SetModeHandler(float new_autopilot_mode);

/** Get autopilot mode
 *
 * @return current autopilot mode
 */
extern uint8_t autopilot_get_mode(void);

/** Reset flight time and launch status
 *  Also provide macro for dl_setting backward compatibility
 */
extern void autopilot_reset_flight_time(void);
#define autopilot_ResetFlightTimeAndLaunch(_) autopilot_reset_flight_time()

/** Start or stop motors
 *  May have no effect if motors has auto-start based on throttle setpoint
 *
 * @param[in] motors_on true to start motors, false to stop
 */
extern void autopilot_set_motors_on(bool motors_on);

/** Get motor status
 *
 * @return true if motors are running
 */
extern bool autopilot_get_motors_on(void);

/** Enable or disable motor control from autopilot
 *  Also provide macro for dl_setting backward compatibility
 *
 * @param[in] kill true to disable (kill), false to enable (un-kill)
 */
extern void autopilot_set_kill_throttle(bool kill);
#define autopilot_KillThrottle(_kill) autopilot_set_kill_throttle(_kill)

/** Get kill status
 *
 * @return true if motors are killed
 */
extern bool autopilot_throttle_killed(void);

/** Check if airframe is in flight
 *
 * @param[in] motors_on motors status
 */
extern void autopilot_check_in_flight(bool motors_on);

/** Set in flight status
 *
 * @param[in] in_flight in flight status
 */
extern void autopilot_set_in_flight(bool in_flight);

/** Get in flight status
 *
 * @return true if airframe in flight
 */
extern bool autopilot_in_flight(void);

/** reset in_flight counter
 *  actual implementation is firmware dependent
 */
extern void autopilot_reset_in_flight_counter(void);

/** Set power switch state
 *  This will actually enable the switch if POWER_SWITCH_GPIO is defined
 *  Also provide macro for dl_setting backward compatibility
 *
 * @param[in] power_switch true to enable, false to disable
 */
extern void autopilot_set_power_switch(bool power_switch);
#define autopilot_SetPowerSwitch(_ps) autopilot_set_power_switch(_ps)

/** Store marked settings in flash
 *  Try to make sure that we don't write to flash while flying
 *  Also provide macro for dl_setting backward compatibility
 */
extern void autopilot_store_settings(void);
#define autopilot_StoreSettings(_) autopilot_store_settings()

/** Clear marked settings in flash
 *  try to make sure that we don't write to flash while flying
 *  Also provide macro for dl_setting backward compatibility
 */
extern void autopilot_clear_settings(void);
#define autopilot_ClearSettings(_) autopilot_clear_settings()

/** Report autopilot version on default downlink channel
 */
extern void autopilot_send_version(void);

/** Report autopilot mode on default downlink channel
 */
extern void autopilot_send_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* AUTOPILOT_H */

