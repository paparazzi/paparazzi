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
#define AP_MODE_FLIP              18
#define AP_MODE_GUIDED            19

extern uint8_t autopilot_mode;
extern uint8_t autopilot_mode_auto2;
extern bool autopilot_motors_on;
extern bool autopilot_in_flight;
extern bool kill_throttle;
extern bool autopilot_rc;

extern bool autopilot_power_switch;

extern void autopilot_init(void);
extern void autopilot_periodic(void);
extern void autopilot_on_rc_frame(void);
extern void autopilot_set_mode(uint8_t new_autopilot_mode);
extern void autopilot_set_motors_on(bool motors_on);
extern void autopilot_check_in_flight(bool motors_on);

extern bool autopilot_ground_detected;
extern bool autopilot_detect_ground_once;

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
#ifdef ROTORCRAFT_IS_HELI
#define SetRotorcraftCommands(_cmd, _in_flight,  _motor_on) { \
    commands[COMMAND_ROLL] = _cmd[COMMAND_ROLL];                \
    commands[COMMAND_PITCH] = _cmd[COMMAND_PITCH];              \
    commands[COMMAND_YAW] = _cmd[COMMAND_YAW];                  \
    commands[COMMAND_THRUST] = _cmd[COMMAND_THRUST];            \
  }
#else

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
      autopilot_ground_detected = true;
      autopilot_detect_ground_once = false;
    }
  }
}

#include "subsystems/settings.h"

/* try to make sure that we don't write to flash while flying */
static inline void autopilot_StoreSettings(float store)
{
  if (kill_throttle && store) {
    settings_store_flag = store;
    settings_store();
  }
}

static inline void autopilot_ClearSettings(float clear)
{
  if (kill_throttle && clear) {
    settings_clear_flag = clear;
    settings_clear();
  }
}

#if DOWNLINK
#include "pprzlink/pprzlink_transport.h"
extern void send_autopilot_version(struct transport_tx *trans, struct link_device *dev);
#endif

/** Set position and heading setpoints in GUIDED mode.
 * @param x North position (local NED frame) in meters.
 * @param y East position (local NED frame) in meters.
 * @param z Down position (local NED frame) in meters.
 * @param heading Setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern bool autopilot_guided_goto_ned(float x, float y, float z, float heading);

/** Set position and heading setpoints wrt. current position in GUIDED mode.
 * @param dx Offset relative to current north position (local NED frame) in meters.
 * @param dy Offset relative to current east position (local NED frame) in meters.
 * @param dz Offset relative to current down position (local NED frame) in meters.
 * @param dyaw Offset relative to current heading setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern bool autopilot_guided_goto_ned_relative(float dx, float dy, float dz, float dyaw);

/** Set position and heading setpoints wrt. current position AND heading in GUIDED mode.
 * @param dx relative position (body frame, forward) in meters.
 * @param dy relative position (body frame, right) in meters.
 * @param dz relative position (body frame, down) in meters.
 * @param dyaw Offset relative to current heading setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern bool autopilot_guided_goto_body_relative(float dx, float dy, float dz, float dyaw);

/** Set velocity and heading setpoints in GUIDED mode.
 * @param vx North velocity (local NED frame) in meters/sec.
 * @param vy East velocity (local NED frame) in meters/sec.
 * @param vz Down velocity (local NED frame) in meters/sec.
 * @param heading Setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern bool autopilot_guided_move_ned(float vx, float vy, float vz, float heading);

/** Set guided setpoints using flag mask in GUIDED mode.
 * @param flags Bits 0-3 are used to determine the axis system to be used.
 * If bits 0 and 1 are clear then the coordinates are set in absolute NE coordinates.
 * If bit 1 is set bit 0 is ignored.
 * Bits 5-7 define whether the setpoints should be used as position or velocity.
 * Bit flags are defined as follows:
      bit 0: x,y as offset coordinates
      bit 1: x,y in body coordinates
      bit 2: z as offset coordinates
      bit 3: yaw as offset coordinates
      bit 4: free
      bit 5: x,y as vel
      bit 6: z as vel
      bit 7: yaw as rate
 * @param x North position/velocity in meters or meters/sec.
 * @param y East position/velocity in meters or meters/sec.
 * @param z Down position/velocity in meters or meters/sec.
 * @param yaw Heading or heading rate setpoint in radians or radians/sec.
 */
extern void autopilot_guided_update(uint8_t flags, float x, float y, float z, float yaw);

#endif /* AUTOPILOT_H */
