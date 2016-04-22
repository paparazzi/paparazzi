/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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

#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/radio_control.h"
#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#include "subsystems/electrical.h"
#include "subsystems/settings.h"
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_none.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#if USE_STABILIZATION_RATE
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#endif

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

#ifndef AUTOPILOT_DISABLE_AHRS_KILL
static inline int ahrs_is_aligned(void)
{
  return stateIsAttitudeValid();
}
#else
PRINT_CONFIG_MSG("Using AUTOPILOT_DISABLE_AHRS_KILL")
static inline int ahrs_is_aligned(void)
{
  return true;
}
#endif

/** Set descent speed in failsafe mode */
#ifndef FAILSAFE_DESCENT_SPEED
#define FAILSAFE_DESCENT_SPEED 1.5
PRINT_CONFIG_VAR(FAILSAFE_DESCENT_SPEED)
#endif

/** Mode that is set when the plane is really too far from home */
#ifndef FAILSAFE_MODE_TOO_FAR_FROM_HOME
#define FAILSAFE_MODE_TOO_FAR_FROM_HOME AP_MODE_FAILSAFE
#endif


#if USE_KILL_SWITCH_FOR_MOTOR_ARMING
#include "autopilot_arming_switch.h"
PRINT_CONFIG_MSG("Using kill switch for motor arming")
#elif USE_THROTTLE_FOR_MOTOR_ARMING
#include "autopilot_arming_throttle.h"
PRINT_CONFIG_MSG("Using throttle for motor arming")
#else
#include "autopilot_arming_yaw.h"
PRINT_CONFIG_MSG("Using 2 sec yaw for motor arming")
#endif

#ifndef MODE_STARTUP
#define MODE_STARTUP AP_MODE_KILL
PRINT_CONFIG_MSG("Using default AP_MODE_KILL as MODE_STARTUP")
#endif

#ifndef UNLOCKED_HOME_MODE
#if MODE_AUTO1 == AP_MODE_HOME
#define UNLOCKED_HOME_MODE TRUE
PRINT_CONFIG_MSG("Enabled UNLOCKED_HOME_MODE since MODE_AUTO1 is AP_MODE_HOME")
#elif MODE_AUTO2 == AP_MODE_HOME
#define UNLOCKED_HOME_MODE TRUE
PRINT_CONFIG_MSG("Enabled UNLOCKED_HOME_MODE since MODE_AUTO2 is AP_MODE_HOME")
#else
#define UNLOCKED_HOME_MODE FALSE
#endif
#endif

#if MODE_MANUAL == AP_MODE_NAV
#error "MODE_MANUAL mustn't be AP_MODE_NAV"
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
  /* mode is finally set at end of init if MODE_STARTUP is not KILL */
  autopilot_mode = AP_MODE_KILL;
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

  autopilot_arming_init();

  nav_init();
  guidance_h_init();
  guidance_v_init();

  stabilization_init();
  stabilization_none_init();
#if USE_STABILIZATION_RATE
  stabilization_rate_init();
#endif
  stabilization_attitude_init();

  /* set startup mode, propagates through to guidance h/v */
  autopilot_set_mode(MODE_STARTUP);

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AUTOPILOT_VERSION, send_autopilot_version);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ALIVE, send_alive);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_STATUS, send_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ATTITUDE, send_attitude);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ENERGY, send_energy);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_FP, send_fp);
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


#define NAV_PRESCALER (PERIODIC_FREQUENCY / NAV_FREQ)
void autopilot_periodic(void)
{

  RunOnceEvery(NAV_PRESCALER, compute_dist2_to_home());

  if (autopilot_in_flight && autopilot_mode == AP_MODE_NAV) {
    if (too_far_from_home) {
      if (dist2_to_home > failsafe_mode_dist2) {
        autopilot_set_mode(FAILSAFE_MODE_TOO_FAR_FROM_HOME);
      } else {
        autopilot_set_mode(AP_MODE_HOME);
      }
    }
  }

  if (autopilot_mode == AP_MODE_HOME) {
    RunOnceEvery(NAV_PRESCALER, nav_home());
  } else {
    // otherwise always call nav_periodic_task so that carrot is always updated in GCS for other modes
    RunOnceEvery(NAV_PRESCALER, nav_periodic_task());
  }


  /* If in FAILSAFE mode and either already not in_flight anymore
   * or just "detected" ground, go to KILL mode.
   */
  if (autopilot_mode == AP_MODE_FAILSAFE) {
    if (!autopilot_in_flight) {
      autopilot_set_mode(AP_MODE_KILL);
    }

#if FAILSAFE_GROUND_DETECT
    INFO("Using FAILSAFE_GROUND_DETECT: KILL")
    if (autopilot_ground_detected) {
      autopilot_set_mode(AP_MODE_KILL);
    }
#endif
  }

  /* Reset ground detection _after_ running flight plan
   */
  if (!autopilot_in_flight) {
    autopilot_ground_detected = false;
    autopilot_detect_ground_once = false;
  }

  /* Set fixed "failsafe" commands from airframe file if in KILL mode.
   * If in FAILSAFE mode, run normal loops with failsafe attitude and
   * downwards velocity setpoints.
   */
  if (autopilot_mode == AP_MODE_KILL) {
    SetCommands(commands_failsafe);
  } else {
    guidance_v_run(autopilot_in_flight);
    guidance_h_run(autopilot_in_flight);
    SetRotorcraftCommands(stabilization_cmd, autopilot_in_flight, autopilot_motors_on);
  }

}


void autopilot_set_mode(uint8_t new_autopilot_mode)
{

  /* force startup mode (default is kill) as long as AHRS is not aligned */
  if (!ahrs_is_aligned()) {
    new_autopilot_mode = MODE_STARTUP;
  }

  if (new_autopilot_mode != autopilot_mode) {
    /* horizontal mode */
    switch (new_autopilot_mode) {
      case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
        stabilization_attitude_set_failsafe_setpoint();
        guidance_h_mode_changed(GUIDANCE_H_MODE_ATTITUDE);
        break;
#endif
      case AP_MODE_KILL:
        autopilot_in_flight = false;
        autopilot_in_flight_counter = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_KILL);
        break;
      case AP_MODE_RC_DIRECT:
        guidance_h_mode_changed(GUIDANCE_H_MODE_RC_DIRECT);
        break;
      case AP_MODE_RATE_RC_CLIMB:
      case AP_MODE_RATE_DIRECT:
      case AP_MODE_RATE_Z_HOLD:
#if USE_STABILIZATION_RATE
        guidance_h_mode_changed(GUIDANCE_H_MODE_RATE);
#else
        return;
#endif
        break;
      case AP_MODE_ATTITUDE_RC_CLIMB:
      case AP_MODE_ATTITUDE_DIRECT:
      case AP_MODE_ATTITUDE_CLIMB:
      case AP_MODE_ATTITUDE_Z_HOLD:
        guidance_h_mode_changed(GUIDANCE_H_MODE_ATTITUDE);
        break;
      case AP_MODE_FORWARD:
        guidance_h_mode_changed(GUIDANCE_H_MODE_FORWARD);
        break;
      case AP_MODE_CARE_FREE_DIRECT:
        guidance_h_mode_changed(GUIDANCE_H_MODE_CARE_FREE);
        break;
      case AP_MODE_HOVER_DIRECT:
      case AP_MODE_HOVER_CLIMB:
      case AP_MODE_HOVER_Z_HOLD:
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
        break;
      case AP_MODE_HOME:
      case AP_MODE_NAV:
        guidance_h_mode_changed(GUIDANCE_H_MODE_NAV);
        break;
      case AP_MODE_MODULE:
#ifdef GUIDANCE_H_MODE_MODULE_SETTING
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE_SETTING);
#endif
        break;
      case AP_MODE_FLIP:
        guidance_h_mode_changed(GUIDANCE_H_MODE_FLIP);
        break;
      case AP_MODE_GUIDED:
        guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
        break;
      default:
        break;
    }
    /* vertical mode */
    switch (new_autopilot_mode) {
      case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
        guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
        guidance_v_zd_sp = SPEED_BFP_OF_REAL(FAILSAFE_DESCENT_SPEED);
        break;
#endif
      case AP_MODE_KILL:
        autopilot_set_motors_on(FALSE);
        stabilization_cmd[COMMAND_THRUST] = 0;
        guidance_v_mode_changed(GUIDANCE_V_MODE_KILL);
        break;
      case AP_MODE_RC_DIRECT:
      case AP_MODE_RATE_DIRECT:
      case AP_MODE_ATTITUDE_DIRECT:
      case AP_MODE_HOVER_DIRECT:
      case AP_MODE_CARE_FREE_DIRECT:
      case AP_MODE_FORWARD:
        guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
        break;
      case AP_MODE_RATE_RC_CLIMB:
      case AP_MODE_ATTITUDE_RC_CLIMB:
        guidance_v_mode_changed(GUIDANCE_V_MODE_RC_CLIMB);
        break;
      case AP_MODE_ATTITUDE_CLIMB:
      case AP_MODE_HOVER_CLIMB:
        guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
        break;
      case AP_MODE_RATE_Z_HOLD:
      case AP_MODE_ATTITUDE_Z_HOLD:
      case AP_MODE_HOVER_Z_HOLD:
        guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
        break;
      case AP_MODE_HOME:
      case AP_MODE_NAV:
        guidance_v_mode_changed(GUIDANCE_V_MODE_NAV);
        break;
      case AP_MODE_MODULE:
#ifdef GUIDANCE_V_MODE_MODULE_SETTING
        guidance_v_mode_changed(GUIDANCE_V_MODE_MODULE_SETTING);
#endif
        break;
      case AP_MODE_FLIP:
        guidance_v_mode_changed(GUIDANCE_V_MODE_FLIP);
        break;
      case AP_MODE_GUIDED:
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        break;
      default:
        break;
    }
    //if switching to rate mode but rate mode is not defined, the function returned
    autopilot_mode = new_autopilot_mode;
  }

}

bool autopilot_guided_goto_ned(float x, float y, float z, float heading)
{
  if (autopilot_mode == AP_MODE_GUIDED) {
    guidance_h_set_guided_pos(x, y);
    guidance_h_set_guided_heading(heading);
    guidance_v_set_guided_z(z);
    return true;
  }
  return false;
}

bool autopilot_guided_goto_ned_relative(float dx, float dy, float dz, float dyaw)
{
  if (autopilot_mode == AP_MODE_GUIDED && stateIsLocalCoordinateValid()) {
    float x = stateGetPositionNed_f()->x + dx;
    float y = stateGetPositionNed_f()->y + dy;
    float z = stateGetPositionNed_f()->z + dz;
    float heading = stateGetNedToBodyEulers_f()->psi + dyaw;
    return autopilot_guided_goto_ned(x, y, z, heading);
  }
  return false;
}

bool autopilot_guided_goto_body_relative(float dx, float dy, float dz, float dyaw)
{
  if (autopilot_mode == AP_MODE_GUIDED && stateIsLocalCoordinateValid()) {
    float psi = stateGetNedToBodyEulers_f()->psi;
    float x = stateGetPositionNed_f()->x + cosf(-psi) * dx + sinf(-psi) * dy;
    float y = stateGetPositionNed_f()->y - sinf(-psi) * dx + cosf(-psi) * dy;
    float z = stateGetPositionNed_f()->z + dz;
    float heading = psi + dyaw;
    return autopilot_guided_goto_ned(x, y, z, heading);
  }
  return false;
}

bool autopilot_guided_move_ned(float vx, float vy, float vz, float heading)
{
  if (autopilot_mode == AP_MODE_GUIDED) {
    guidance_h_set_guided_vel(vx, vy);
    guidance_h_set_guided_heading(heading);
    guidance_v_set_guided_vz(vz);
    return true;
  }
  return false;
}

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


void autopilot_set_motors_on(bool motors_on)
{
  if (autopilot_mode != AP_MODE_KILL && ahrs_is_aligned() && motors_on) {
    autopilot_motors_on = true;
  } else {
    autopilot_motors_on = false;
  }
  kill_throttle = ! autopilot_motors_on;
  autopilot_arming_set(autopilot_motors_on);
}


#define THRESHOLD_1_PPRZ (MIN_PPRZ / 2)
#define THRESHOLD_2_PPRZ (MAX_PPRZ / 2)

/** get autopilot mode as set by RADIO_MODE 3-way switch */
static uint8_t ap_mode_of_3way_switch(void)
{
  if (radio_control.values[RADIO_MODE] > THRESHOLD_2_PPRZ) {
    return autopilot_mode_auto2;
  }
  else if (radio_control.values[RADIO_MODE] > THRESHOLD_1_PPRZ) {
    return MODE_AUTO1;
  }
  else {
    return MODE_MANUAL;
  }
}

/**
 * Get autopilot mode from two 2way switches.
 * RADIO_MODE switch just selectes between MANUAL and AUTO.
 * If not MANUAL, the RADIO_AUTO_MODE switch selects between AUTO1 and AUTO2.
 *
 * This is mainly a cludge for entry level radios with no three-way switch,
 * but two available two-way switches which can be used.
 */
#if defined RADIO_AUTO_MODE || defined(__DOXYGEN__)
static uint8_t ap_mode_of_two_switches(void)
{
  if (radio_control.values[RADIO_MODE] < THRESHOLD_1_PPRZ) {
    /* RADIO_MODE in MANUAL position */
    return MODE_MANUAL;
  }
  else {
    /* RADIO_MODE not in MANUAL position.
     * Select AUTO mode bassed on RADIO_AUTO_MODE channel
     */
    if (radio_control.values[RADIO_AUTO_MODE] > THRESHOLD_2_PPRZ) {
      return autopilot_mode_auto2;
    }
    else
      return MODE_AUTO1;
  }
}
#endif

void autopilot_on_rc_frame(void)
{

  if (kill_switch_is_on()) {
    autopilot_set_mode(AP_MODE_KILL);
  } else {
#ifdef RADIO_AUTO_MODE
    INFO("Using RADIO_AUTO_MODE to switch between AUTO1 and AUTO2.")
    uint8_t new_autopilot_mode = ap_mode_of_two_switches();
#else
    uint8_t new_autopilot_mode = ap_mode_of_3way_switch();
#endif

    /* don't enter NAV mode if GPS is lost (this also prevents mode oscillations) */
    if (!(new_autopilot_mode == AP_MODE_NAV && GpsIsLost())) {
      /* always allow to switch to manual */
      if (new_autopilot_mode == MODE_MANUAL) {
        autopilot_set_mode(new_autopilot_mode);
      }
      /* if in HOME mode, don't allow switching to non-manual modes */
      else if ((autopilot_mode != AP_MODE_HOME)
#if UNLOCKED_HOME_MODE
               /* Allowed to leave home mode when UNLOCKED_HOME_MODE */
               || !too_far_from_home
#endif
              ) {
        autopilot_set_mode(new_autopilot_mode);
      }
    }
  }

  /* an arming sequence is used to start/stop motors.
   * only allow arming if ahrs is aligned
   */
  if (ahrs_is_aligned()) {
    autopilot_arming_check_motors_on();
    kill_throttle = ! autopilot_motors_on;
  }

  /* if not in FAILSAFE or HOME mode, read RC and set commands accordingly */
  if (autopilot_mode != AP_MODE_FAILSAFE && autopilot_mode != AP_MODE_HOME) {

    /* if there are some commands that should always be set from RC, do it */
#ifdef SetAutoCommandsFromRC
    SetAutoCommandsFromRC(commands, radio_control.values);
#endif

    /* if not in NAV_MODE set commands from the rc */
#ifdef SetCommandsFromRC
    if (autopilot_mode != AP_MODE_NAV) {
      SetCommandsFromRC(commands, radio_control.values);
    }
#endif

    guidance_v_read_rc();
    guidance_h_read_rc(autopilot_in_flight);
  }

}
