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

// ap copy of fbw readings
struct Electrical ap_electrical;

uint8_t lateral_mode;
uint8_t  mcu1_status;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
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
  float power = ap_electrical.vsupply * ap_electrical.current;
  pprz_msg_send_ENERGY(trans, dev, AC_ID,
                       &throttle, &ap_electrical.vsupply, &ap_electrical.current, &power, &ap_electrical.charge, &ap_electrical.energy);
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
  ap_electrical.vsupply = 0.f;
  ap_electrical.current = 0.f;
  ap_electrical.charge  = 0.f;
  ap_electrical.energy  = 0.f;

  ap_electrical.bat_critical = false;
  ap_electrical.bat_low = false;

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
#endif
}

