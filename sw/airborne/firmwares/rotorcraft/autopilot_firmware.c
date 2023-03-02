/*
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
 * @file firmwares/rotorcraft/autopilot.c
 *
 * Autopilot.
 *
 */

#include "firmwares/rotorcraft/autopilot_firmware.h"

#include "generated/modules.h"

#include <stdint.h>
//#include "mcu_periph/sys_time.h"
#include "modules/energy/electrical.h"
#include "modules/datalink/telemetry.h"
#include "modules/radio_control/radio_control.h"

#if USE_GPS
#include "modules/gps/gps.h"
#else
#if NO_GPS_NEEDED_FOR_NAV
#define GpsIsLost() FALSE
#else
#define GpsIsLost() TRUE
#endif
#endif

uint8_t  autopilot_mode_auto2;
static uint32_t autopilot_in_flight_counter;

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

/** Z-acceleration threshold to detect ground in m/s^2 */
#ifndef THRESHOLD_GROUND_DETECT
#define THRESHOLD_GROUND_DETECT 25.0
#endif


#if USE_MOTOR_MIXING
#include "modules/actuators/motor_mixing.h"
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
  uint8_t in_flight = autopilot.in_flight;
  uint8_t motors_on = autopilot.motors_on;
  uint16_t time_sec = sys_time.nb_sec;
  pprz_msg_send_ROTORCRAFT_STATUS(trans, dev, AC_ID,
                                  &imu_nb_err, &_motor_nb_err,
                                  &radio_control.status, &radio_control.frame_rate,
                                  &fix, &autopilot.mode, &in_flight, &motors_on,
                                  &autopilot.arming_status, &guidance_h.mode, &guidance_v.mode,
                                  &time_sec, &electrical.vsupply);
}

static void send_energy(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t throttle = 100 * autopilot.throttle / MAX_PPRZ;
  float power = electrical.vsupply * electrical.current;
  pprz_msg_send_ENERGY(trans, dev, AC_ID,
                       &throttle, &electrical.vsupply, &electrical.current, &power, &electrical.charge, &electrical.energy);
}

static void send_fp(struct transport_tx *trans, struct link_device *dev)
{
  int32_t carrot_up = -guidance_v.z_sp;
  int32_t carrot_heading = ANGLE_BFP_OF_REAL(guidance_h.sp.heading);
  int32_t thrust = (int32_t)autopilot.throttle;
  struct EnuCoor_i *pos = stateGetPositionEnu_i();
#if GUIDANCE_INDI_HYBRID
  struct FloatEulers eulers_zxy;
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
  struct Int32Eulers att;
  EULERS_BFP_OF_REAL(att, eulers_zxy);
#else
  struct Int32Eulers att = *stateGetNedToBodyEulers_i();
#endif
  pprz_msg_send_ROTORCRAFT_FP(trans, dev, AC_ID,
                              &pos->x,
                              &pos->y,
                              &pos->z,
                              &(stateGetSpeedEnu_i()->x),
                              &(stateGetSpeedEnu_i()->y),
                              &(stateGetSpeedEnu_i()->z),
                              &att.phi,
                              &att.theta,
                              &att.psi,
                              &guidance_h.sp.pos.y,
                              &guidance_h.sp.pos.x,
                              &carrot_up,
                              &carrot_heading,
                              &thrust,
                              &autopilot.flight_time);
}

static void send_body_rates_accel(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_BODY_RATES_ACCEL(trans, dev, AC_ID,
                                  &(stateGetBodyRates_f()->p),
                                  &(stateGetBodyRates_f()->q),
                                  &(stateGetBodyRates_f()->r),
                                  &(stateGetAccelBody_i()->x),
                                  &(stateGetAccelBody_i()->y),
                                  &(stateGetAccelBody_i()->z));
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

static void send_rotorcraft_cmd(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROTORCRAFT_CMD(trans, dev, AC_ID,
                               &stabilization_cmd[COMMAND_ROLL],
                               &stabilization_cmd[COMMAND_PITCH],
                               &stabilization_cmd[COMMAND_YAW],
                               &stabilization_cmd[COMMAND_THRUST]);
}


void autopilot_firmware_init(void)
{
  autopilot_in_flight_counter = 0;
  autopilot_mode_auto2 = MODE_AUTO2;

  // register messages
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_STATUS, send_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ENERGY, send_energy);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_FP, send_fp);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_FP_MIN, send_fp_min);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_CMD, send_rotorcraft_cmd);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_BODY_RATES_ACCEL, send_body_rates_accel);
#ifdef RADIO_CONTROL
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_RADIO_CONTROL, send_rotorcraft_rc);
#endif
}

/** autopilot event function
 *
 * used for automatic ground detection
 */
void autopilot_event(void)
{
  if (autopilot.detect_ground_once
#ifdef AP_MODE_FAILSAFE
      || autopilot.mode == AP_MODE_FAILSAFE
#endif
     ) {
    struct NedCoor_f *accel = stateGetAccelNed_f();
    if (accel->z < -THRESHOLD_GROUND_DETECT ||
        accel->z > THRESHOLD_GROUND_DETECT) {
      autopilot.ground_detected = true;
      autopilot.detect_ground_once = false;
    }
  }
}

/** reset in_flight counter
 */
void autopilot_reset_in_flight_counter(void)
{
  autopilot_in_flight_counter = 0;
}

/** in flight check utility function
 */
void autopilot_check_in_flight(bool motors_on)
{
  if (autopilot.in_flight) {
    if (autopilot_in_flight_counter > 0) {
      /* probably in_flight if thrust, speed and accel above IN_FLIGHT_MIN thresholds */
      if ((stabilization_cmd[COMMAND_THRUST] <= AUTOPILOT_IN_FLIGHT_MIN_THRUST) &&
          (fabsf(stateGetSpeedNed_f()->z) < AUTOPILOT_IN_FLIGHT_MIN_SPEED) &&
          (fabsf(stateGetAccelNed_f()->z) < AUTOPILOT_IN_FLIGHT_MIN_ACCEL)) {
        autopilot_in_flight_counter--;
        if (autopilot_in_flight_counter == 0) {
          autopilot.in_flight = false;
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
          autopilot.in_flight = true;
        }
      } else { /* currently not in_flight and thrust below threshold, reset counter */
        autopilot_in_flight_counter = 0;
      }
    }
  }
}

