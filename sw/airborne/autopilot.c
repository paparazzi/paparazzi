/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 * @file autopilot.c
 *
 * Core autopilot interface common to all firmwares.
 * Using either static or generated autopilot logic,
 * which depends on the firmware.
 *
 */

#include "autopilot.h"

#include "generated/modules.h"

#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/gpio.h"
#include "modules/core/commands.h"
#include "modules/datalink/telemetry.h"
#include "modules/core/abi.h"

#include "modules/core/settings.h"
#include "generated/settings.h"

#include "pprz_version.h"

struct pprz_autopilot autopilot;

#ifndef AUTOPILOT_RC_ID
#define AUTOPILOT_RC_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(AUTOPILOT_RC_ID)
static abi_event rc_ev;

static void rc_cb(uint8_t __attribute__((unused)) sender_id,
                  struct RadioControl __attribute__((unused)) *rc)
{
  // TODO pass the RC struct to the on_rc_frame function
  autopilot_on_rc_frame();
}

static void send_autopilot_version(struct transport_tx *trans, struct link_device *dev)
{
  static uint32_t ap_version = PPRZ_VERSION_INT;
  static char *ver_desc = PPRZ_VERSION_DESC;
  pprz_msg_send_AUTOPILOT_VERSION(trans, dev, AC_ID, &ap_version, strlen(ver_desc), ver_desc);
}

static void send_alive(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ALIVE(trans, dev, AC_ID, 16, MD5SUM);
}

static void send_attitude(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  pprz_msg_send_ATTITUDE(trans, dev, AC_ID, &(att->phi), &(att->psi), &(att->theta));
};

static void send_dl_value(struct transport_tx *trans, struct link_device *dev)
{
  PeriodicSendDlValue(trans, dev);
}

static void send_minimal_com(struct transport_tx *trans, struct link_device *dev)
{
  float lat = DegOfRad(stateGetPositionLla_f()->lat);
  float lon = DegOfRad(stateGetPositionLla_f()->lon);
  float hmsl = stateGetPositionUtm_f()->alt;
  float gspeed = stateGetHorizontalSpeedNorm_f();
  float course = stateGetHorizontalSpeedDir_f();
  float climb = stateGetSpeedEnu_f()->z;
  uint8_t throttle = (uint8_t)(100 * autopilot.throttle / MAX_PPRZ);
#if USE_GPS
  uint8_t gps_fix = gps.fix;
#else
  uint8_t gps_fix = 0;
#endif
  pprz_msg_send_MINIMAL_COM(trans, dev, AC_ID,
                            &lat, &lon, &hmsl, &gspeed, &course, &climb,
                            &electrical.vsupply, &throttle, &autopilot.mode,
                            &nav_block, &gps_fix, &autopilot.flight_time);
}

void autopilot_init(void)
{
#ifdef MODE_AUTO2
  autopilot.mode_auto2 = MODE_AUTO2; // FIXME
#endif
  autopilot.flight_time = 0;
  autopilot.throttle = 0;
  autopilot.motors_on = false;
  autopilot.kill_throttle = true;
  autopilot.in_flight = false;
  autopilot.ground_detected = false;
  autopilot.detect_ground_once = false;
  autopilot.use_rc = true;

  // call firmware specific init
  autopilot_firmware_init();

  // call autopilot implementation init after guidance modules init
  // (should be guaranteed by modules dependencies)
  // it will set startup mode
#if USE_GENERATED_AUTOPILOT
  autopilot_generated_init();
#else
  autopilot_static_init();
#endif

  // bind ABI messages
  AbiBindMsgRADIO_CONTROL(AUTOPILOT_RC_ID, &rc_ev, rc_cb);

  // register messages
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AUTOPILOT_VERSION, send_autopilot_version);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ALIVE, send_alive);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ATTITUDE, send_attitude);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DL_VALUE, send_dl_value);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MINIMAL_COM, send_minimal_com);
}

/** AP periodic call
 */
void autopilot_periodic(void)
{
  // first check for failsafe case
  autopilot_failsafe_checks();

  // check if in flight
  autopilot_check_in_flight(autopilot_get_motors_on());

#if FIXEDWING_FIRMWARE
  if (autopilot.flight_time) {
#else
  if (autopilot_in_flight()) {
#endif
    // flight time is incremented every second
    // after takeoff for fixedwing
    // when in flight for other firmwares
    RunOnceEvery(PERIODIC_FREQUENCY, autopilot.flight_time++);
  }

#if USE_GENERATED_AUTOPILOT
  autopilot_generated_periodic();
#else
  autopilot_static_periodic();
#endif
}

/** AP event call
 */
void WEAK autopilot_event(void) {}

/** RC frame handler
 */
void autopilot_on_rc_frame(void)
{
#if USE_GENERATED_AUTOPILOT
  autopilot_generated_on_rc_frame();
#else
  autopilot_static_on_rc_frame();
#endif
}

/** Failsafe checks
 */
void WEAK autopilot_failsafe_checks(void) {}

/** set autopilot mode
 */
bool autopilot_set_mode(uint8_t new_autopilot_mode)
{
  uint8_t mode = autopilot.mode;
#if USE_GENERATED_AUTOPILOT
  autopilot_generated_set_mode(new_autopilot_mode);
#else
  autopilot_static_set_mode(new_autopilot_mode);
#endif
  return (autopilot.mode != mode);
}

/** AP mode setting handler
 */
void autopilot_SetModeHandler(float mode)
{
#if USE_GENERATED_AUTOPILOT
  autopilot_generated_SetModeHandler(mode);
#else
  autopilot_static_SetModeHandler(mode);
#endif
}

/** get autopilot mode
 */
uint8_t autopilot_get_mode(void)
{
  return autopilot.mode;
}

/** reset flight time and launch
 */
void autopilot_reset_flight_time(void)
{
  autopilot.flight_time = 0;
  autopilot.launch = false;
}

/** turn motors on/off, eventually depending of the current mode
 *  set kill_throttle accordingly FIXME is it true for FW firmware ?
 */
void autopilot_set_motors_on(bool motors_on)
{
#if USE_GENERATED_AUTOPILOT
  autopilot_generated_set_motors_on(motors_on);
#else
  autopilot_static_set_motors_on(motors_on);
#endif
  autopilot.kill_throttle = ! autopilot.motors_on;
}

/** get motors status
 */
bool autopilot_get_motors_on(void)
{
  return autopilot.motors_on;
}

/** set kill throttle
 */
void autopilot_set_kill_throttle(bool kill)
{
  if (kill) {
    autopilot_set_motors_on(false);
  } else {
    autopilot_set_motors_on(true);
  }
}

/** get kill status
 */
bool autopilot_throttle_killed(void)
{
  return autopilot.kill_throttle;
}

/** in flight check utility function
 *  actual implementation is firmware dependent
 */
void WEAK autopilot_check_in_flight(bool motors_on __attribute__((unused))) {}

/** reset in_flight counter
 *  actual implementation is firmware dependent
 */
void WEAK autopilot_reset_in_flight_counter(void) {}

/** set in_flight flag
 */
void autopilot_set_in_flight(bool in_flight)
{
  autopilot.in_flight = in_flight;
  if (!in_flight) {
    autopilot_reset_in_flight_counter();
  }
}

/** get in_flight flag
 */
bool autopilot_in_flight(void)
{
  return autopilot.in_flight;
}

/** store settings
 */
void autopilot_store_settings(void)
{
  if (autopilot.kill_throttle) {
    settings_store_flag = true;
    settings_store();
  }
}

/** clear settings
 */
void autopilot_clear_settings(void)
{
  if (autopilot.kill_throttle) {
    settings_clear_flag = true;
    settings_clear();
  }
}

/** send autopilot version
 */
void autopilot_send_version(void)
{
  send_autopilot_version(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
}

/** send autopilot mode
 *  actual implementation is firmware dependent
 */
void WEAK autopilot_send_mode(void) {}

