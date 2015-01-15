/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * @file firmwares/rotorcraft/autopilot.h
 *
 * Autopilot modes.
 *
 */

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "std.h"
#include "generated/airframe.h"
#include "state.h"

#define AP_MODE_KILL              0
#define AP_MODE_FAILSAFE          1
#define AP_MODE_HOME              2
#define AP_MODE_RATE_DIRECT       3
#define AP_MODE_ATTITUDE_DIRECT   4
#define AP_MODE_RATE_RC_CLIMB     5
#define AP_MODE_ATTITUDE_RC_CLIMB 6
#define AP_MODE_ATTITUDE_CLIMB    7
#define AP_MODE_RATE_Z_HOLD       8
#define AP_MODE_ATTITUDE_Z_HOLD   9
#define AP_MODE_HOVER_DIRECT      10
#define AP_MODE_HOVER_CLIMB       11
#define AP_MODE_HOVER_Z_HOLD      12
#define AP_MODE_NAV               13
#define AP_MODE_RC_DIRECT         14  // Safety Pilot Direct Commands for helicopter direct control
#define AP_MODE_CARE_FREE_DIRECT  15
#define AP_MODE_FORWARD           16
#define AP_MODE_MODULE            17

extern uint8_t autopilot_mode;
extern uint8_t autopilot_mode_auto2;
extern bool_t autopilot_motors_on;
extern bool_t autopilot_in_flight;
extern bool_t kill_throttle;
extern bool_t autopilot_rc;

extern bool_t autopilot_power_switch;

extern void autopilot_init(void);
extern void autopilot_periodic(void);
extern void autopilot_on_rc_frame(void);
extern void autopilot_set_mode(uint8_t new_autopilot_mode);
extern void autopilot_set_motors_on(bool_t motors_on);
extern void autopilot_check_in_flight(bool_t motors_on);

extern bool_t autopilot_ground_detected;
extern bool_t autopilot_detect_ground_once;

extern uint16_t autopilot_flight_time;

/** Default RC mode.
 */
#ifndef MODE_MANUAL
#define MODE_MANUAL AP_MODE_ATTITUDE_DIRECT
#endif
#ifndef MODE_AUTO1
#define MODE_AUTO1 AP_MODE_HOVER_Z_HOLD
#endif
#ifndef MODE_AUTO2
#define MODE_AUTO2 AP_MODE_NAV
#endif


#define THRESHOLD_1_PPRZ (MIN_PPRZ / 2)
#define THRESHOLD_2_PPRZ (MAX_PPRZ / 2)

#define AP_MODE_OF_PPRZ(_rc, _mode) {    \
    if      (_rc > THRESHOLD_2_PPRZ)     \
      _mode = autopilot_mode_auto2;      \
    else if (_rc > THRESHOLD_1_PPRZ)     \
      _mode = MODE_AUTO1;                \
    else                                 \
      _mode = MODE_MANUAL;               \
  }

#define autopilot_KillThrottle(_kill) { \
    if (_kill)                          \
      autopilot_set_motors_on(FALSE);   \
    else                                \
      autopilot_set_motors_on(TRUE);    \
  }

#ifdef POWER_SWITCH_GPIO
#include "mcu_periph/gpio.h"
#define autopilot_SetPowerSwitch(_v) {                          \
    autopilot_power_switch = _v;                                \
    if (_v) { gpio_set(POWER_SWITCH_GPIO); }  \
    else { gpio_clear(POWER_SWITCH_GPIO); }   \
  }
#else
#define autopilot_SetPowerSwitch(_v) {  \
    autopilot_power_switch = _v;        \
  }
#endif

/** Set Rotorcraft commands.
 *  Limit thrust and/or yaw depending of the in_flight
 *  and motors_on flag status
 */
#ifndef ROTORCRAFT_COMMANDS_YAW_ALWAYS_ENABLED
#define SetRotorcraftCommands(_cmd, _in_flight,  _motor_on) { \
    if (!(_in_flight)) { _cmd[COMMAND_YAW] = 0; }               \
    if (!(_motor_on)) { _cmd[COMMAND_THRUST] = 0; }             \
    commands[COMMAND_ROLL] = _cmd[COMMAND_ROLL];                \
    commands[COMMAND_PITCH] = _cmd[COMMAND_PITCH];              \
    commands[COMMAND_YAW] = _cmd[COMMAND_YAW];                  \
    commands[COMMAND_THRUST] = _cmd[COMMAND_THRUST];            \
  }
#else
#define SetRotorcraftCommands(_cmd, _in_flight,  _motor_on) { \
    if (!(_motor_on)) { _cmd[COMMAND_THRUST] = 0; }             \
    commands[COMMAND_ROLL] = _cmd[COMMAND_ROLL];                \
    commands[COMMAND_PITCH] = _cmd[COMMAND_PITCH];              \
    commands[COMMAND_YAW] = _cmd[COMMAND_YAW];                  \
    commands[COMMAND_THRUST] = _cmd[COMMAND_THRUST];            \
  }
#endif

/** Z-acceleration threshold to detect ground in m/s^2 */
#ifndef THRESHOLD_GROUND_DETECT
#define THRESHOLD_GROUND_DETECT 25.0
#endif
/** Ground detection based on vertical acceleration.
 */
static inline void DetectGroundEvent(void)
{
  if (autopilot_mode == AP_MODE_FAILSAFE || autopilot_detect_ground_once) {
    struct NedCoor_f *accel = stateGetAccelNed_f();
    if (accel->z < -THRESHOLD_GROUND_DETECT ||
        accel->z > THRESHOLD_GROUND_DETECT) {
      autopilot_ground_detected = TRUE;
      autopilot_detect_ground_once = FALSE;
    }
  }
}

#include "subsystems/settings.h"

static inline void autopilot_StoreSettings(float store)
{
  if (kill_throttle && store) {
    settings_store_flag = store;
    settings_store();
  }
}

#if DOWNLINK
#include "subsystems/datalink/transport.h"
extern void send_autopilot_version(struct transport_tx *trans, struct link_device *dev);
#endif

#endif /* AUTOPILOT_H */
