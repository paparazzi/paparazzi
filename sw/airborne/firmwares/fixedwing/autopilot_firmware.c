/*
 * Copyright (C) 2012-2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/fixedwing/autopilot_firmware.c
 *
 * Fixedwing specific autopilot interface
 * and initialization
 *
 */

#include "firmwares/fixedwing/autopilot_firmware.h"

#include "state.h"
#include "firmwares/fixedwing/nav.h"
#include <stdint.h>
#include "modules/energy/electrical.h"

PRINT_CONFIG_VAR(CONTROL_FREQUENCY)

uint8_t lateral_mode;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "generated/settings.h"

#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS
#include "modules/settings/rc_settings.h"
static void send_rc_settings(struct transport_tx *trans, struct link_device *dev)
{
  if (!RcSettingsOff()) {
    pprz_msg_send_SETTINGS(trans, dev, AC_ID, &slider_1_val, &slider_2_val);
  }
}
#else
uint8_t rc_settings_mode = 0;
#endif

static void send_mode(struct transport_tx *trans, struct link_device *dev)
{
  /** Old FBW status
   * - bit 0: RC OK
   * - bit 1: RC REALLY LOST
   * - bit 2: FBW AUTO
   * - bit 3: FBW FAILSAFE
   * - bit 4: RC averaged channels sent
   */
  uint8_t mcu1_status = 0;
#ifdef RADIO_CONTROL
  mcu1_status |= (radio_control.status == RC_OK ? (1<<0) : 0);
  mcu1_status |= (radio_control.status == RC_REALLY_LOST ? (1<<2) : 0);
  mcu1_status |= (radio_control.status == RC_OK ? (1<<4) : 0);
#endif
#ifdef AP_MODE_MANUAL
  mcu1_status |= (autopilot_get_mode() == AP_MODE_MANUAL ? 0 : (1<<2));
#endif

  pprz_msg_send_PPRZ_MODE(trans, dev, AC_ID,
                          &autopilot.mode, &v_ctl_mode, &lateral_mode, &horizontal_mode, &rc_settings_mode, &mcu1_status);
}

static void send_estimator(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ESTIMATOR(trans, dev, AC_ID,
                          &(stateGetPositionUtm_f()->alt), &(stateGetSpeedEnu_f()->z));
}

static void send_energy(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t throttle = 100 * autopilot.throttle / MAX_PPRZ;
  float power = electrical.vsupply * electrical.current;
  pprz_msg_send_ENERGY(trans, dev, AC_ID,
                       &throttle, &electrical.vsupply, &electrical.current, &power, &electrical.charge, &electrical.energy);
}

// FIXME not the best place
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include CTRL_TYPE_H
static void send_desired(struct transport_tx *trans, struct link_device *dev)
{
#ifndef USE_AIRSPEED
  float v_ctl_auto_airspeed_setpoint = NOMINAL_AIRSPEED;
#endif
  pprz_msg_send_DESIRED(trans, dev, AC_ID,
                        &h_ctl_roll_setpoint, &h_ctl_pitch_loop_setpoint, &h_ctl_course_setpoint,
                        &desired_x, &desired_y, &v_ctl_altitude_setpoint, &v_ctl_climb_setpoint,
                        &v_ctl_auto_airspeed_setpoint);
}

static void send_airspeed(struct transport_tx *trans __attribute__((unused)),
                          struct link_device *dev __attribute__((unused)))
{
  float airspeed = stateGetAirspeed_f();
#if USE_AIRSPEED
  pprz_msg_send_AIRSPEED(trans, dev, AC_ID,
                         &airspeed, &v_ctl_auto_airspeed_setpoint,
                         &v_ctl_auto_airspeed_controlled, &v_ctl_auto_groundspeed_setpoint);
#else
  float zero = 0;
  pprz_msg_send_AIRSPEED(trans, dev, AC_ID, &airspeed, &zero, &zero, &zero);
#endif
}

#if !INTERMCU_AP
#include "modules/radio_control/radio_control.h"
static void send_fbw_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t fbw_mode = 1; // old FBW_AUTO
#ifdef AP_MODE_MANUAL
  if (autopilot_get_mode() == AP_MODE_MANUAL) { fbw_mode = 0; } // FBW Manual
#endif
#ifdef RADIO_CONTROL
  uint8_t rc_status = radio_control.status;
  uint8_t rc_rate = radio_control.frame_rate;
#else
  uint8_t rc_status = 0;
  uint8_t rc_rate = 0;
#endif
  pprz_msg_send_FBW_STATUS(trans, dev, AC_ID,
                           &rc_status, &rc_rate, &fbw_mode,
                           &electrical.vsupply, &electrical.current);
}
#endif

#endif /* PERIODIC_TELEMETRY */

void autopilot_send_mode(void)
{
  // use default telemetry here
#if DOWNLINK
  send_mode(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif
}

void autopilot_firmware_init(void)
{
#if PERIODIC_TELEMETRY
  /* register some periodic message */
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PPRZ_MODE, send_mode);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ESTIMATOR, send_estimator);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED, send_airspeed);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ENERGY, send_energy);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DESIRED, send_desired);
#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RC_SETTINGS, send_rc_settings);
#endif
#if !INTERMCU_AP
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FBW_STATUS, send_fbw_status);
#endif
#endif
}


#include "generated/flight_plan.h"
#include "generated/airframe.h"

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
  static uint8_t t = 0;
  if (electrical.vsupply < CATASTROPHIC_BAT_LEVEL) {
    t++;
  } else {
    t = 0;
  }
#if !USE_GENERATED_AUTOPILOT
  // only check for static autopilot
  autopilot.kill_throttle |= (t >= CATASTROPHIC_BAT_KILL_DELAY);
  autopilot.kill_throttle |= autopilot.launch && (dist2_to_home > Square(KILL_MODE_DISTANCE));
#endif

  if (!autopilot.flight_time &&
      stateGetHorizontalSpeedNorm_f() > MIN_SPEED_FOR_TAKEOFF) {
    autopilot.flight_time = 1;
    autopilot.launch = true; /* Not set in non auto launch */
#if DOWNLINK
    uint16_t time_sec = sys_time.nb_sec;
    DOWNLINK_SEND_TAKEOFF(DefaultChannel, DefaultDevice, &time_sec);
#endif
  }

}


