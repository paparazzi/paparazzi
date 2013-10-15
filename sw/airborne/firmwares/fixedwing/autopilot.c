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

#include "firmwares/fixedwing/autopilot.h"

#include "state.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/nav.h"
#include "generated/settings.h"

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

static void send_alive(void) {
  DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
}

#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS
#include "rc_settings.h"
static void send_rc_settings(void) {
  if (!RcSettingsOff())
    DOWNLINK_SEND_SETTINGS(DefaultChannel, DefaultDevice, &slider_1_val, &slider_2_val);
}
#else
uint8_t rc_settings_mode = 0;
#endif

void autopilot_send_mode(void) {
  DOWNLINK_SEND_PPRZ_MODE(DefaultChannel, DefaultDevice,
      &pprz_mode, &v_ctl_mode, &lateral_mode, &horizontal_mode, &rc_settings_mode, &mcu1_status);
}

static void send_attitude(void) {
  struct FloatEulers* att = stateGetNedToBodyEulers_f();
  DOWNLINK_SEND_ATTITUDE(DefaultChannel, DefaultDevice,
      &(att->phi), &(att->psi), &(att->theta));
};

static void send_estimator(void) {
  DOWNLINK_SEND_ESTIMATOR(DefaultChannel, DefaultDevice,
      &(stateGetPositionUtm_f()->alt), &(stateGetSpeedEnu_f()->z));
}

static void send_bat(void) {
  int16_t amps = (int16_t) (current/10);
  int16_t e = energy;
  DOWNLINK_SEND_BAT(DefaultChannel, DefaultDevice,
      &v_ctl_throttle_slewed, &vsupply, &amps,
      &autopilot_flight_time, &kill_throttle,
      &block_time, &stage_time, &e);
}

static void send_energy(void) {
  const int16_t e = energy;
  const float vsup = ((float)vsupply) / 10.0f;
  const float curs = ((float)current) / 1000.0f;
  const float power = vsup * curs;
  DOWNLINK_SEND_ENERGY(DefaultChannel, DefaultDevice, &vsup, &curs, &e, &power);
}

static void send_dl_value(void) {
  PeriodicSendDlValue(DefaultChannel, DefaultDevice);
}

// FIXME not the best place
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include CTRL_TYPE_H
static void send_desired(void) {
#ifndef USE_AIRSPEED
  float v_ctl_auto_airspeed_setpoint = NOMINAL_AIRSPEED;
#endif
  DOWNLINK_SEND_DESIRED(DefaultChannel, DefaultDevice,
      &h_ctl_roll_setpoint, &h_ctl_pitch_loop_setpoint, &h_ctl_course_setpoint,
      &desired_x, &desired_y, &v_ctl_altitude_setpoint, &v_ctl_climb_setpoint,
      &v_ctl_auto_airspeed_setpoint);
}

static void send_airspeed(void) {
#ifdef MEASURE_AIRSPEED
  float* airspeed = stateGetAirspeed_f();
  DOWNLINK_SEND_AIRSPEED(DefaultChannel, DefaultDevice, airspeed, airspeed, airspeed, airspeed);
#elif USE_AIRSPEED
  DOWNLINK_SEND_AIRSPEED(DefaultChannel, DefaultDevice,
      stateGetAirspeed_f(), &v_ctl_auto_airspeed_setpoint,
      &v_ctl_auto_airspeed_controlled, &v_ctl_auto_groundspeed_setpoint);
#endif
}

static void send_downlink(void) {
  static uint16_t last;
  uint16_t rate = (downlink_nb_bytes - last) / PERIOD_DOWNLINK_Ap_0;
  last = downlink_nb_bytes;
  DOWNLINK_SEND_DOWNLINK(DefaultChannel, DefaultDevice,
      &downlink_nb_ovrn, &rate, &downlink_nb_msgs);
}

void autopilot_init(void) {
  pprz_mode = PPRZ_MODE_AUTO2;
  kill_throttle = FALSE;
  launch = FALSE;
  autopilot_flight_time = 0;

  lateral_mode = LATERAL_MODE_MANUAL;

  gps_lost = FALSE;

  power_switch = FALSE;

  /* register some periodic message */
  register_periodic_telemetry(DefaultPeriodic, "ALIVE", send_alive);
  register_periodic_telemetry(DefaultPeriodic, "PPRZ_MODE", autopilot_send_mode);
  register_periodic_telemetry(DefaultPeriodic, "DOWNLINK", send_downlink);
  register_periodic_telemetry(DefaultPeriodic, "ATTITUDE", send_attitude);
  register_periodic_telemetry(DefaultPeriodic, "ESTIMATOR", send_estimator);
  register_periodic_telemetry(DefaultPeriodic, "AIRSPEED", send_airspeed);
  register_periodic_telemetry(DefaultPeriodic, "BAT", send_bat);
  register_periodic_telemetry(DefaultPeriodic, "ENERGY", send_energy);
  register_periodic_telemetry(DefaultPeriodic, "DL_VALUE", send_dl_value);
  register_periodic_telemetry(DefaultPeriodic, "DESIRED", send_desired);
#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS
  register_periodic_telemetry(DefaultPeriodic, "RC_SETTINGS", send_rc_settings);
#endif
}

