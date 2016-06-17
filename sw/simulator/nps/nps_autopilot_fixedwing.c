/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2013 The Paparazzi Team
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

#ifdef FBW
#include "firmwares/fixedwing/main_fbw.h"
#define Fbw(f) f ## _fbw()
#else
#define Fbw(f)
#endif

#ifdef AP
#include "firmwares/fixedwing/main_ap.h"
#define Ap(f) f ## _ap()
#else
#define Ap(f)
#endif

#include "nps_sensors.h"
#include "nps_radio_control.h"
#include "nps_electrical.h"
#include "nps_fdm.h"

#include "subsystems/radio_control.h"
#include "subsystems/imu.h"
#include "mcu_periph/sys_time.h"
#include "state.h"
#include "subsystems/commands.h"

#include "subsystems/abi.h"

// for launch
#include "firmwares/fixedwing/autopilot.h"

// for datalink_time hack
#include "subsystems/datalink/datalink.h"

struct NpsAutopilot autopilot;
bool nps_bypass_ahrs;
bool nps_bypass_ins;

#ifndef NPS_BYPASS_AHRS
#define NPS_BYPASS_AHRS FALSE
#endif

#ifndef NPS_BYPASS_INS
#define NPS_BYPASS_INS FALSE
#endif


#if !defined (FBW) || !defined (AP)
#error NPS does not currently support dual processor simulation for FBW and AP on fixedwing!
#endif

void nps_autopilot_init(enum NpsRadioControlType type_rc, int num_rc_script, char *rc_dev)
{

  autopilot.launch = FALSE;

  nps_radio_control_init(type_rc, num_rc_script, rc_dev);
  nps_electrical_init();

  nps_bypass_ahrs = NPS_BYPASS_AHRS;
  nps_bypass_ins = NPS_BYPASS_INS;

  Fbw(init);
  Ap(init);

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
    Fbw(event_task);
  }
#endif

  if (nps_sensors_gyro_available()) {
    imu_feed_gyro_accel();
    Fbw(event_task);
    Ap(event_task);
  }

  if (nps_sensors_mag_available()) {
    imu_feed_mag();
    Fbw(event_task);
    Ap(event_task);
  }

  if (nps_sensors_baro_available()) {
    float pressure = (float) sensors.baro.value;
    AbiSendMsgBARO_ABS(BARO_SIM_SENDER_ID, pressure);
    Fbw(event_task);
    Ap(event_task);
  }

  if (nps_sensors_temperature_available()) {
    AbiSendMsgTEMPERATURE(BARO_SIM_SENDER_ID, (float)sensors.temp.value);
  }

#if USE_AIRSPEED
  if (nps_sensors_airspeed_available()) {
    stateSetAirspeed_f((float)sensors.airspeed.value);
    Fbw(event_task);
    Ap(event_task);
  }
#endif

  if (nps_sensors_gps_available()) {
    gps_feed_value();
    Fbw(event_task);
    Ap(event_task);
  }

  if (nps_bypass_ahrs) {
    sim_overwrite_ahrs();
  }

  if (nps_bypass_ins) {
    sim_overwrite_ins();
  }

  Fbw(handle_periodic_tasks);
  Ap(handle_periodic_tasks);

  /* scale final motor commands to 0-1 for feeding the fdm */
#ifdef NPS_ACTUATOR_NAMES
  PRINT_CONFIG_MSG("actuators for JSBSim explicitly set.")
  PRINT_CONFIG_VAR(NPS_COMMANDS_NB)
  //PRINT_CONFIG_VAR(NPS_ACTUATOR_NAMES)

  for (uint8_t i = 0; i < NPS_COMMANDS_NB; i++) {
    autopilot.commands[i] = (double)commands[i] / MAX_PPRZ;
  }
  // hack: invert pitch to fit most JSBSim models
  autopilot.commands[COMMAND_PITCH] = -(double)commands[COMMAND_PITCH] / MAX_PPRZ;
#else
  PRINT_CONFIG_MSG("Using throttle, roll, pitch, yaw commands instead of explicit actuators.")
  PRINT_CONFIG_VAR(COMMAND_THROTTLE)
  PRINT_CONFIG_VAR(COMMAND_ROLL)
  PRINT_CONFIG_VAR(COMMAND_PITCH)

  autopilot.commands[COMMAND_THROTTLE] = (double)commands[COMMAND_THROTTLE] / MAX_PPRZ;
  autopilot.commands[COMMAND_ROLL] = (double)commands[COMMAND_ROLL] / MAX_PPRZ;
  // hack: invert pitch to fit most JSBSim models
  autopilot.commands[COMMAND_PITCH] = -(double)commands[COMMAND_PITCH] / MAX_PPRZ;
#ifdef COMMAND_YAW
  PRINT_CONFIG_VAR(COMMAND_YAW)
  autopilot.commands[COMMAND_YAW] = (double)commands[COMMAND_YAW] / MAX_PPRZ;
#else
  autopilot.commands[3] = 0.;
#endif
#endif

  // do the launch when clicking launch in GCS
  autopilot.launch = launch && !kill_throttle;
  if (!launch) {
    autopilot.commands[COMMAND_THROTTLE] = 0;
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

  if (state.ned_initialized_i || state.ned_initialized_f) {
    struct NedCoor_f ltp_pos;
    VECT3_COPY(ltp_pos, fdm.ltpprz_pos);
    stateSetPositionNed_f(&ltp_pos);
  }
  else if (state.utm_initialized_f) {
    struct LlaCoor_f lla;
    LLA_COPY(lla, fdm.lla_pos);
    struct UtmCoor_f utm;
    utm.zone = (lla.lon / 1e7 + 180) / 6 + 1;
    utm_of_lla_f(&utm, &lla);
    stateSetPositionUtm_f(&utm);
  }

  struct NedCoor_f ltp_speed;
  VECT3_COPY(ltp_speed, fdm.ltpprz_ecef_vel);
  stateSetSpeedNed_f(&ltp_speed);

  struct NedCoor_f ltp_accel;
  VECT3_COPY(ltp_accel, fdm.ltpprz_ecef_accel);
  stateSetAccelNed_f(&ltp_accel);

}
