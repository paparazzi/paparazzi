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
 * @file firmwares/rotorcraft/autopilot.c
 *
 * Autopilot.
 *
 */

#include <stdint.h>
#include "firmwares/rotorcraft/autopilot.h"

#include "generated/modules.h"

#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/radio_control.h"
#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#include "subsystems/electrical.h"
#include "subsystems/settings.h"
#include "subsystems/datalink/telemetry.h"

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

#ifdef POWER_SWITCH_GPIO
#include "mcu_periph/gpio.h"
#endif

#include "pprz_version.h"

uint8_t  autopilot_mode;
uint8_t  autopilot_mode_auto2;

bool   autopilot_in_flight;
uint32_t autopilot_in_flight_counter;
uint16_t autopilot_flight_time;

bool   autopilot_motors_on;
bool   kill_throttle;

bool   autopilot_rc;
bool   autopilot_power_switch;

bool   autopilot_ground_detected;
bool   autopilot_detect_ground_once;

/* Geofence exceptions */
#include "modules/nav/nav_geofence.h"

/** time steps for in_flight detection (at 20Hz, so 20=1second) */
#ifndef AUTOPILOT_IN_FLIGHT_TIME
#define AUTOPILOT_IN_FLIGHT_TIME    20
#endif

/** minimum vertical speed for in_flight condition in m/s */
#ifndef AUTOPILOT_IN_FLIGHT_MIN_SPEED
#define AUTOPILOT_IN_FLIGHT_MIN_SPEED 0.2
#endif

/** minimum vertical acceleration for in_flight condition in m/s^2 */
#ifndef AUTOPILOT_IN_FLIGHT_MIN_ACCEL
#define AUTOPILOT_IN_FLIGHT_MIN_ACCEL 2.0
#endif

/** minimum thrust for in_flight condition in pprz_t units (max = 9600) */
#ifndef AUTOPILOT_IN_FLIGHT_MIN_THRUST
#define AUTOPILOT_IN_FLIGHT_MIN_THRUST 500
#endif


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

static void send_attitude(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  pprz_msg_send_ATTITUDE(trans, dev, AC_ID, &(att->phi), &(att->psi), &(att->theta));
};

#if USE_MOTOR_MIXING
#include "subsystems/actuators/motor_mixing.h"
#endif

static void send_status(struct transport_tx *trans, struct link_device *dev)
{
  uint32_t imu_nb_err = 0;
#if USE_MOTOR_MIXING
  uint8_t _motor_nb_err = motor_mixing.nb_saturation + motor_mixing.nb_failure * 10;
#else
  uint8_t _motor_nb_err = 0;
#endif
#if USE_GPS
  uint8_t fix = gps.fix;
#else
  uint8_t fix = 0;
#endif
  uint8_t in_flight = autopilot_in_flight;
  uint8_t motors_on = autopilot_motors_on;
  uint16_t time_sec = sys_time.nb_sec;
  pprz_msg_send_ROTORCRAFT_STATUS(trans, dev, AC_ID,
                                  &imu_nb_err, &_motor_nb_err,
                                  &radio_control.status, &radio_control.frame_rate,
                                  &fix, &autopilot_mode, &in_flight, &motors_on,
                                  &guidance_h.mode, &guidance_v_mode,
                                  &electrical.vsupply, &time_sec);
}

static void send_energy(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t e = electrical.energy;
  if (fabs(electrical.energy) >= INT16_MAX) {
    e = INT16_MAX;
  }
  float vsup = ((float)electrical.vsupply) / 10.0f;
  float curs = ((float)electrical.current) / 1000.0f;
  float power = vsup * curs;
  pprz_msg_send_ENERGY(trans, dev, AC_ID, &vsup, &curs, &e, &power);
}

static void send_fp(struct transport_tx *trans, struct link_device *dev)
{
  int32_t carrot_up = -guidance_v_z_sp;
  pprz_msg_send_ROTORCRAFT_FP(trans, dev, AC_ID,
                              &(stateGetPositionEnu_i()->x),
                              &(stateGetPositionEnu_i()->y),
                              &(stateGetPositionEnu_i()->z),
                              &(stateGetSpeedEnu_i()->x),
                              &(stateGetSpeedEnu_i()->y),
                              &(stateGetSpeedEnu_i()->z),
                              &(stateGetNedToBodyEulers_i()->phi),
                              &(stateGetNedToBodyEulers_i()->theta),
                              &(stateGetNedToBodyEulers_i()->psi),
                              &guidance_h.sp.pos.y,
                              &guidance_h.sp.pos.x,
                              &carrot_up,
                              &guidance_h.sp.heading,
                              &stabilization_cmd[COMMAND_THRUST],
                              &autopilot_flight_time);
}

static void send_fp_min(struct transport_tx *trans, struct link_device *dev)
{
#if USE_GPS
  uint16_t gspeed = gps.gspeed;
#else
  // ground speed in cm/s
  uint16_t gspeed = stateGetHorizontalSpeedNorm_f() / 100;
#endif
  pprz_msg_send_ROTORCRAFT_FP_MIN(trans, dev, AC_ID,
                              &(stateGetPositionEnu_i()->x),
                              &(stateGetPositionEnu_i()->y),
                              &(stateGetPositionEnu_i()->z),
                              &gspeed);
}

#ifdef RADIO_CONTROL
static void send_rc(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_RC(trans, dev, AC_ID, RADIO_CONTROL_NB_CHANNEL, radio_control.values);
}

static void send_rotorcraft_rc(struct transport_tx *trans, struct link_device *dev)
{
#ifdef RADIO_KILL_SWITCH
  int16_t _kill_switch = radio_control.values[RADIO_KILL_SWITCH];
#else
  int16_t _kill_switch = 42;
#endif
  pprz_msg_send_ROTORCRAFT_RADIO_CONTROL(trans, dev, AC_ID,
                                         &radio_control.values[RADIO_ROLL],
                                         &radio_control.values[RADIO_PITCH],
                                         &radio_control.values[RADIO_YAW],
                                         &radio_control.values[RADIO_THROTTLE],
                                         &radio_control.values[RADIO_MODE],
                                         &_kill_switch,
                                         &radio_control.status);
}
#endif

#ifdef ACTUATORS
static void send_actuators(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ACTUATORS(trans, dev, AC_ID , ACTUATORS_NB, actuators);
}
#endif

static void send_dl_value(struct transport_tx *trans, struct link_device *dev)
{
  PeriodicSendDlValue(trans, dev);
}

static void send_rotorcraft_cmd(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROTORCRAFT_CMD(trans, dev, AC_ID,
                               &stabilization_cmd[COMMAND_ROLL],
                               &stabilization_cmd[COMMAND_PITCH],
                               &stabilization_cmd[COMMAND_YAW],
                               &stabilization_cmd[COMMAND_THRUST]);
}


void autopilot_init(void)
{
  autopilot_motors_on = false;
  kill_throttle = ! autopilot_motors_on;
  autopilot_in_flight = false;
  autopilot_in_flight_counter = 0;
  autopilot_mode_auto2 = MODE_AUTO2;
  autopilot_ground_detected = false;
  autopilot_detect_ground_once = false;
  autopilot_flight_time = 0;
  autopilot_rc = true;
  autopilot_power_switch = false;
#ifdef POWER_SWITCH_GPIO
  gpio_setup_output(POWER_SWITCH_GPIO);
  gpio_clear(POWER_SWITCH_GPIO); // POWER OFF
#endif

  // register messages
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AUTOPILOT_VERSION, send_autopilot_version);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ALIVE, send_alive);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_STATUS, send_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ATTITUDE, send_attitude);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ENERGY, send_energy);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_FP, send_fp);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_FP_MIN, send_fp_min);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_CMD, send_rotorcraft_cmd);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DL_VALUE, send_dl_value);
#ifdef ACTUATORS
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATORS, send_actuators);
#endif
#ifdef RADIO_CONTROL
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RC, send_rc);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_RADIO_CONTROL, send_rotorcraft_rc);
#endif
}

/** AP periodic call
 */
void autopilot_periodic(void)
{
#if USE_GENERATED_AUTOPILOT
  autopilot_generated_periodic();
#else
  autopilot_static_periodic();
#endif
}

/** set autopilot mode
 */
void autopilot_set_mode(uint8_t new_autopilot_mode)
{
#if USE_GENERATED_AUTOPILOT
  autopilot_generated_set_mode(new_autopilot_mode);
#else
  autopilot_static_set_mode(new_autopilot_mode);
#endif
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

/** turn motors on/off, eventually depending of the current mode
 *  set kill_throttle accordingly
 */
void autopilot_set_motors_on(bool motors_on)
{
#if USE_GENERATED_AUTOPILOT
  autopilot_generated_set_motors_on(motors_on);
#else
  autopilot_static_set_motors_on(motors_on);
#endif
  kill_throttle = ! autopilot_motors_on;
}

/** in flight check utility function
 */
void autopilot_check_in_flight(bool motors_on)
{
  if (autopilot_in_flight) {
    if (autopilot_in_flight_counter > 0) {
      /* probably in_flight if thrust, speed and accel above IN_FLIGHT_MIN thresholds */
      if ((stabilization_cmd[COMMAND_THRUST] <= AUTOPILOT_IN_FLIGHT_MIN_THRUST) &&
          (fabsf(stateGetSpeedNed_f()->z) < AUTOPILOT_IN_FLIGHT_MIN_SPEED) &&
          (fabsf(stateGetAccelNed_f()->z) < AUTOPILOT_IN_FLIGHT_MIN_ACCEL)) {
        autopilot_in_flight_counter--;
        if (autopilot_in_flight_counter == 0) {
          autopilot_in_flight = false;
        }
      } else { /* thrust, speed or accel not above min threshold, reset counter */
        autopilot_in_flight_counter = AUTOPILOT_IN_FLIGHT_TIME;
      }
    }
  } else { /* currently not in flight */
    if (autopilot_in_flight_counter < AUTOPILOT_IN_FLIGHT_TIME &&
        motors_on) {
      /* if thrust above min threshold, assume in_flight.
       * Don't check for velocity and acceleration above threshold here...
       */
      if (stabilization_cmd[COMMAND_THRUST] > AUTOPILOT_IN_FLIGHT_MIN_THRUST) {
        autopilot_in_flight_counter++;
        if (autopilot_in_flight_counter == AUTOPILOT_IN_FLIGHT_TIME) {
          autopilot_in_flight = true;
        }
      } else { /* currently not in_flight and thrust below threshold, reset counter */
        autopilot_in_flight_counter = 0;
      }
    }
  }
}

