/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

#include "nps_autopilot.h"

#include "firmwares/rotorcraft/main.h"
#include "nps_sensors.h"
#include "nps_radio_control.h"
#include "nps_electrical.h"
#include "nps_fdm.h"

#include "subsystems/radio_control.h"
#include "subsystems/imu.h"
#include "mcu_periph/sys_time.h"
#include "state.h"
#include "subsystems/ahrs.h"
#include "subsystems/ins.h"
#include "math/pprz_algebra.h"

#ifndef NPS_NO_MOTOR_MIXING
#include "subsystems/actuators/motor_mixing.h"

#if NPS_COMMANDS_NB != MOTOR_MIXING_NB_MOTOR
#warning "NPS_COMMANDS_NB does not match MOTOR_MIXING_NB_MOTOR!"
#endif
#endif

#include "modules/core/abi.h"

#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

// for datalink_time hack
#include "subsystems/datalink/datalink.h"
#include "subsystems/actuators.h"

struct NpsAutopilot nps_autopilot;
bool nps_bypass_ahrs;
bool nps_bypass_ins;

#ifndef NPS_BYPASS_AHRS
#define NPS_BYPASS_AHRS FALSE
#endif

#ifndef NPS_BYPASS_INS
#define NPS_BYPASS_INS FALSE
#endif

#if INDI_RPM_FEEDBACK
#error "INDI_RPM_FEEDBACK can not be used in simulation!"
#endif

void nps_autopilot_init(enum NpsRadioControlType type_rc, int num_rc_script, char *rc_dev)
{
  nps_autopilot.launch = TRUE;

  nps_radio_control_init(type_rc, num_rc_script, rc_dev);
  nps_electrical_init();

  nps_bypass_ahrs = NPS_BYPASS_AHRS;
  nps_bypass_ins = NPS_BYPASS_INS;

  main_init();

}

void nps_autopilot_run_systime_step(void)
{
  sys_tick_handler();
}

#include <stdio.h>
#include "subsystems/gps.h"

void nps_autopilot_run_step(double time)
{

  nps_electrical_run_step(time);

#if RADIO_CONTROL && !RADIO_CONTROL_TYPE_DATALINK
  if (nps_radio_control_available(time)) {
    radio_control_feed();
    main_event();
  }
#endif

  if (nps_sensors_gyro_available()) {
    imu_feed_gyro_accel();
    main_event();
  }

  if (nps_sensors_mag_available()) {
    imu_feed_mag();
    main_event();
  }

  if (nps_sensors_baro_available()) {
    uint32_t now_ts = get_sys_time_usec();
    float pressure = (float) sensors.baro.value;
    AbiSendMsgBARO_ABS(BARO_SIM_SENDER_ID, now_ts, pressure);
    main_event();
  }

  if (nps_sensors_temperature_available()) {
    AbiSendMsgTEMPERATURE(BARO_SIM_SENDER_ID, (float)sensors.temp.value);
  }

#if USE_AIRSPEED
  if (nps_sensors_airspeed_available()) {
    stateSetAirspeed_f((float)sensors.airspeed.value);
    AbiSendMsgAIRSPEED(AIRSPEED_NPS_ID, (float)sensors.airspeed.value);
  }
#endif

#if USE_SONAR
  if (nps_sensors_sonar_available()) {
    uint32_t now_ts = get_sys_time_usec();
    float dist = (float) sensors.sonar.value;
    if (dist >= 0.0) {
      AbiSendMsgAGL(AGL_SONAR_NPS_ID, now_ts, dist);
    }

#ifdef SENSOR_SYNC_SEND_SONAR
    uint16_t foo = 0;
    DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &foo, &dist);
#endif

    main_event();
  }
#endif

#if USE_GPS
  if (nps_sensors_gps_available()) {
    gps_feed_value();
    main_event();
  }
#endif

  if (nps_bypass_ahrs) {
    sim_overwrite_ahrs();
  }

  if (nps_bypass_ins) {
    sim_overwrite_ins();
  }

  handle_periodic_tasks();

  /* scale final motor commands to 0-1 for feeding the fdm */
  for (uint8_t i = 0; i < NPS_COMMANDS_NB; i++) {
#if NPS_NO_MOTOR_MIXING
    actuators_pprz[i] = autopilot_get_motors_on() ? actuators_pprz[i] : 0;
    nps_autopilot.commands[i] = (double)actuators_pprz[i] / MAX_PPRZ;
#else
    nps_autopilot.commands[i] = (double)motor_mixing.commands[i] / MAX_PPRZ;
#endif
  }
}


void sim_overwrite_ahrs(void)
{

  struct FloatQuat quat_f;
  QUAT_COPY(quat_f, fdm.ltp_to_body_quat);
  stateSetNedToBodyQuat_f(&quat_f);

  struct FloatRates rates_f;
  RATES_COPY(rates_f, fdm.body_ecef_rotvel);
  stateSetBodyRates_f(&rates_f);

}

void sim_overwrite_ins(void)
{

  struct NedCoor_f ltp_pos;
  VECT3_COPY(ltp_pos, fdm.ltpprz_pos);
  stateSetPositionNed_f(&ltp_pos);

  struct NedCoor_f ltp_speed;
  VECT3_COPY(ltp_speed, fdm.ltpprz_ecef_vel);
  stateSetSpeedNed_f(&ltp_speed);

  struct NedCoor_f ltp_accel;
  VECT3_COPY(ltp_accel, fdm.ltpprz_ecef_accel);
  stateSetAccelNed_f(&ltp_accel);

}
