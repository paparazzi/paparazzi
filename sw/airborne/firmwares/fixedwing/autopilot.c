/*
 * Copyright (C) 2012 Gautier Hattenberger
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
 * @file firmwares/fixedwing/autopilot.c
 *
 * Fixedwing autopilot inititalization.
 *
 */

#include <stdint.h>
#include "firmwares/fixedwing/autopilot.h"

#include "state.h"
#include "firmwares/fixedwing/nav.h"

#ifdef POWER_SWITCH_GPIO
#include "mcu_periph/gpio.h"
#endif

#include "pprz_version.h"

uint8_t pprz_mode;
bool_t kill_throttle;
uint8_t  mcu1_status;

bool_t launch;

/** flight time in seconds. */
uint16_t autopilot_flight_time;

uint8_t lateral_mode;

uint16_t vsupply;
int32_t current;
float energy;

bool_t gps_lost;

bool_t power_switch;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#include "generated/settings.h"

void send_autopilot_version(struct transport_tx *trans, struct link_device *dev)
{
  static uint32_t ap_version = PPRZ_VERSION_INT;
  static char *ver_desc = PPRZ_VERSION_DESC;
  pprz_msg_send_AUTOPILOT_VERSION(trans, dev, AC_ID, &ap_version, strlen(ver_desc), ver_desc);
}

static void send_alive(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ALIVE(trans, dev, AC_ID, 16, MD5SUM);
}

#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS
#include "rc_settings.h"
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
                          &pprz_mode, &v_ctl_mode, &lateral_mode, &horizontal_mode, &rc_settings_mode, &mcu1_status);
}

static void send_attitude(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  pprz_msg_send_ATTITUDE(trans, dev, AC_ID,
                         &(att->phi), &(att->psi), &(att->theta));
};

static void send_estimator(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ESTIMATOR(trans, dev, AC_ID,
                          &(stateGetPositionUtm_f()->alt), &(stateGetSpeedEnu_f()->z));
}

static void send_bat(struct transport_tx *trans, struct link_device *dev)
{
  int16_t amps = (int16_t)(current / 10);
  int16_t e = energy;
  // prevent overflow
  if (fabs(energy) >= INT16_MAX) {
    e = INT16_MAX;
  }
  pprz_msg_send_BAT(trans, dev, AC_ID,
                    &v_ctl_throttle_slewed, &vsupply, &amps,
                    &autopilot_flight_time, (uint8_t *)(&kill_throttle),
                    &block_time, &stage_time, &e);
}

static void send_energy(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t e = energy;
  if (fabs(energy) >= INT16_MAX) {
    e = INT16_MAX;
  }
  float vsup = ((float)vsupply) / 10.0f;
  float curs = ((float)current) / 1000.0f;
  float power = vsup * curs;
  pprz_msg_send_ENERGY(trans, dev, AC_ID, &vsup, &curs, &e, &power);
}

static void send_dl_value(struct transport_tx *trans, struct link_device *dev)
{
  PeriodicSendDlValue(trans, dev);
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

void autopilot_init(void)
{
  pprz_mode = PPRZ_MODE_AUTO2;
  kill_throttle = FALSE;
  launch = FALSE;
  autopilot_flight_time = 0;

  lateral_mode = LATERAL_MODE_MANUAL;

  gps_lost = FALSE;

  power_switch = FALSE;
#ifdef POWER_SWITCH_GPIO
  gpio_setup_output(POWER_SWITCH_GPIO);
  gpio_clear(POWER_SWITCH_GPIO);
#endif

#if PERIODIC_TELEMETRY
  /* register some periodic message */
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AUTOPILOT_VERSION, send_autopilot_version);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ALIVE, send_alive);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PPRZ_MODE, send_mode);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ATTITUDE, send_attitude);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ESTIMATOR, send_estimator);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED, send_airspeed);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_BAT, send_bat);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ENERGY, send_energy);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DL_VALUE, send_dl_value);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DESIRED, send_desired);
#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RC_SETTINGS, send_rc_settings);
#endif
#endif
}

