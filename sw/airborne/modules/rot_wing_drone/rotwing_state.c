/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 *
 * This file is part of paparazzi
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

/** @file "modules/rotwing/rotwing_state.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * This module keeps track of the current state of a rotating wing drone and desired state set by the RC or flightplan. Paramters are being scheduled in each change of a current state and desired state. Functions are defined in this module to call the actual state and desired state and set a desired state.
 */

#include "modules/rot_wing_drone/rotwing_state.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "modules/nav/nav_rotorcraft_hybrid.h"
#include "firmwares/rotorcraft/autopilot_firmware.h"

#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"

// Quad state identification
#ifndef ROTWING_MIN_SKEW_ANGLE_DEG_QUAD
#define ROTWING_MIN_SKEW_ANGLE_DEG_QUAD 10.0
#endif
#ifndef ROTWING_MIN_SKEW_ANGLE_COUNTER
#define ROTWING_MIN_SKEW_ANGLE_COUNTER 10         // Minimum number of loops the skew angle is below ROTWING_MIN_SKEW_ANGLE_COUNTER
#endif

// Skewing state identification
#ifndef ROTWING_SKEWING_COUNTER
#define ROTWING_SKEWING_COUNTER 10                // Minimum number of loops the skew angle is in between QUAD and FW
#endif

// maximum quad airspeed to force quad state
#ifndef ROTWING_MAX_QUAD_AIRSPEED
#define ROTWING_MAX_QUAD_AIRSPEED 20.0
#endif

// Half skew state identification
#ifndef ROTWING_HALF_SKEW_ANGLE_DEG
#define ROTWING_HALF_SKEW_ANGLE_DEG 55.0
#endif
#ifndef ROTWING_HALF_SKEW_ANGLE_RANG
#define ROTWING_HALF_SKEW_ANGLE_HALF_RANGE 10.0
#endif
#ifndef ROTWING_HALF_SKEW_COUNTER
#define ROTWING_HALF_SKEW_COUNTER 10              // Minimum number of loops the skew angle is at HALF_SKEW_ANGLE_DEG +/- ROTWING_HALF_SKEW_ANGLE_HALF_RANGE to trigger ROTWING_HALF_SKEW_ANGLE state
#endif

// FW state identification
#ifndef ROTWING_MIN_FW_SKEW_ANGLE_DEG
#define ROTWING_MIN_FW_SKEW_ANGLE_DEG 80.0        // Minimum wing angle to fly in fixed wing state 
#endif
#ifndef ROTWING_MIN_FW_COUNTER
#define ROTWING_MIN_FW_COUNTER 10                 // Minimum number of loops the skew angle is above the MIN_FW_SKEW_ANGLE
#endif

// FW idle state identification
#ifndef ROTWING_MIN_THRUST_IDLE
#define ROTWING_MIN_THRUST_IDLE 100
#endif
#ifndef ROTWING_MIN_THRUST_IDLE_COUNTER
#define ROTWING_MIN_THRUST_IDLE_COUNTER 10
#endif

// FW hov mot off state identification
#ifndef ROTWING_HOV_MOT_OFF_RPM_TH
#define ROTWING_HOV_MOT_OFF_RPM_TH 50
#endif
#ifndef ROTWING_HOV_MOT_OFF_COUNTER
#define ROTWING_HOV_MOT_OFF_COUNTER 10
#endif

#ifndef ROTWING_STATE_USE_ROTATION_REF_MODEL
#define ROTWING_STATE_USE_ROTATION_REF_MODEL FALSE
#endif


// Hover preferred pitch (deg)
#ifndef ROTWING_STATE_HOVER_PREF_PITCH
#define ROTWING_STATE_HOVER_PREF_PITCH 0.0
#endif

// Transition preffered pitch (deg)
#ifndef ROTWING_STATE_TRANSITION_PREF_PITCH
#define ROTWING_STATE_TRANSITION_PREF_PITCH 3.0
#endif

// Forward preffered pitch (deg)
#ifndef ROTWING_STATE_FW_PREF_PITCH
#define ROTWING_STATE_FW_PREF_PITCH 8.0
#endif

/** ABI binding feedback data.
 */
#ifndef ROTWING_STATE_ACT_FEEDBACK_ID
#define ROTWING_STATE_ACT_FEEDBACK_ID ABI_BROADCAST
#endif
abi_event rotwing_state_feedback_ev;
static void rotwing_state_feedback_cb(uint8_t sender_id, struct act_feedback_t *feedback_msg, uint8_t num_act);
#define ROTWING_STATE_NUM_HOVER_RPM  4
int32_t rotwing_state_hover_rpm[ROTWING_STATE_NUM_HOVER_RPM] = {0, 0, 0, 0};

struct RotwingState rotwing_state;
struct RotWingStateSettings rotwing_state_settings;
struct RotWingStateSkewing  rotwing_state_skewing;

uint8_t rotwing_state_hover_counter = 0;
uint8_t rotwing_state_skewing_counter = 0;
uint8_t rotwing_state_fw_counter = 0;
uint8_t rotwing_state_fw_idle_counter = 0;
uint8_t rotwing_state_fw_m_off_counter = 0;

float rotwing_state_max_hover_speed = 7;

bool hover_motors_active = true;
bool bool_disable_hover_motors = false;


inline void rotwing_check_set_current_state(void);
inline void rotwing_switch_state(void);

inline void rotwing_state_set_hover_settings(void);
inline void rotwing_state_set_skewing_settings(void);
inline void rotwing_state_set_fw_settings(void);
inline void rotwing_state_set_fw_hov_mot_idle_settings(void);
inline void rotwing_state_set_fw_hov_mot_off_settings(void);

inline void rotwing_state_set_state_settings(void);
inline void rotwing_state_skewer(void);

inline void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_rotating_wing_state(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t adc_dummy = 0;
  pprz_msg_send_ROTATING_WING_STATE(trans, dev, AC_ID,
                                    &rotwing_state.current_state,
                                    &rotwing_state.desired_state,
                                    &rotwing_state_skewing.wing_angle_deg,
                                    &rotwing_state_skewing.wing_angle_deg_sp,
                                    &adc_dummy,
                                    &rotwing_state_skewing.servo_pprz_cmd);
}
#endif // PERIODIC_TELEMETRY

void init_rotwing_state(void)
{
  // Bind ABI messages
  AbiBindMsgACT_FEEDBACK(ROTWING_STATE_ACT_FEEDBACK_ID, &rotwing_state_feedback_ev, rotwing_state_feedback_cb);

  // Start the drone in a desired hover state
  rotwing_state.current_state = ROTWING_STATE_HOVER;
  rotwing_state.desired_state = ROTWING_STATE_HOVER;

  rotwing_state_skewing.wing_angle_deg_sp     = 0;
  rotwing_state_skewing.wing_angle_deg        = 0;
  rotwing_state_skewing.servo_pprz_cmd        = -MAX_PPRZ;
  rotwing_state_skewing.airspeed_scheduling   = false;
  rotwing_state_skewing.force_rotation_angle  = false;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTATING_WING_STATE, send_rotating_wing_state);
#endif
}

void periodic_rotwing_state(void)
{
  // Check and set the current state
  rotwing_check_set_current_state();

  // Check and update desired state
  if (guidance_h.mode == GUIDANCE_H_MODE_NAV) {
    rotwing_switch_state();
  } else if (guidance_h.mode == GUIDANCE_H_MODE_ATTITUDE) {
    rotwing_state_set_hover_settings();
  } else if (guidance_h.mode == GUIDANCE_H_MODE_FORWARD) {
    rotwing_state_set_fw_settings();
  }

  // Run the wing skewer
  rotwing_state_skewer();

  //TODO: incorparate motor active / disbaling depending on called flight state
  // Switch on motors if flight mode is attitude
  if (guidance_h.mode == GUIDANCE_H_MODE_ATTITUDE) {
    bool_disable_hover_motors = false;
  } else if (guidance_h.mode == GUIDANCE_H_MODE_FORWARD) {
    bool_disable_hover_motors = false;
  }
}

// Function to request a state
void request_rotwing_state(uint8_t state)
{
  if (state <= ROTWING_STATE_FW_HOV_MOT_OFF) {
    rotwing_state.desired_state = state;
  }
}

// Function to prefer configuration
void rotwing_request_configuration(uint8_t configuration)
{
  switch (configuration) {
    case ROTWING_CONFIGURATION_HOVER:
      request_rotwing_state(ROTWING_STATE_HOVER);
      break;
    case ROTWING_CONFIGURATION_HYBRID:
      request_rotwing_state(ROTWING_STATE_SKEWING);
      break;
    case ROTWING_CONFIGURATION_EFFICIENT:
      request_rotwing_state(ROTWING_STATE_FW_HOV_MOT_OFF);
      break;
  }
}

void rotwing_check_set_current_state(void)
{
  // if !in_flight, set state to hover
  if (!autopilot.in_flight) {
    rotwing_state_hover_counter = 0;
    rotwing_state_skewing_counter = 0;
    rotwing_state_fw_counter = 0;
    rotwing_state_fw_idle_counter = 0;
    rotwing_state_fw_m_off_counter = 0;
    rotwing_state.current_state = ROTWING_STATE_HOVER;
    return;
  }

  // States can be checked according to wing angle sensor, setpoints .....
  uint8_t prev_state = rotwing_state.current_state;
  switch (prev_state) {
    case ROTWING_STATE_HOVER:
      // Check if state needs to be set to skewing
      if (rotwing_state_skewing.wing_angle_deg > ROTWING_MIN_SKEW_ANGLE_DEG_QUAD) {
        rotwing_state_skewing_counter++;
      } else {
        rotwing_state_skewing_counter = 0;
      }

      // Switch state if necessary
      if (rotwing_state_skewing_counter > ROTWING_MIN_SKEW_ANGLE_COUNTER) {
        rotwing_state.current_state = ROTWING_STATE_SKEWING;
        rotwing_state_skewing_counter = 0;
      }
      break;

    case ROTWING_STATE_SKEWING:
      // Check if state needs to be set to hover
      if (rotwing_state_skewing.wing_angle_deg < ROTWING_MIN_SKEW_ANGLE_DEG_QUAD) {
        rotwing_state_hover_counter++;
      } else {
        rotwing_state_hover_counter = 0;
      }

      // Check if state needs to be set to fixed wing
      if (rotwing_state_skewing.wing_angle_deg > ROTWING_MIN_FW_SKEW_ANGLE_DEG) {
        rotwing_state_fw_counter++;
      } else {
        rotwing_state_fw_counter = 0;
      }

      // Switch state if necessary
      if (rotwing_state_hover_counter > ROTWING_MIN_SKEW_ANGLE_COUNTER) {
        rotwing_state.current_state = ROTWING_STATE_HOVER;
        rotwing_state_hover_counter = 0;
      }

      if (rotwing_state_fw_counter > ROTWING_MIN_FW_COUNTER) {
        rotwing_state.current_state = ROTWING_STATE_FW;
        rotwing_state_fw_counter = 0;
      }
      break;

    case ROTWING_STATE_FW:
      // Check if state needs to be set to skewing
      if (rotwing_state_skewing.wing_angle_deg < ROTWING_MIN_FW_SKEW_ANGLE_DEG) {
        rotwing_state_skewing_counter++;
      } else {
        rotwing_state_skewing_counter = 0;
      }

      // Check if state needs to be set to fixed wing with hover motors idle (If hover thrust below threshold)
      if (stabilization_cmd[COMMAND_THRUST] < ROTWING_MIN_THRUST_IDLE && rotwing_state.desired_state > ROTWING_STATE_FW) {
        rotwing_state_fw_idle_counter++;
      } else {
        rotwing_state_fw_idle_counter = 0;
      }

      // Switch state if necessary
      if (rotwing_state_skewing_counter > ROTWING_MIN_FW_COUNTER) {
        rotwing_state.current_state = ROTWING_STATE_SKEWING;
        rotwing_state_skewing_counter = 0;
        rotwing_state_fw_idle_counter = 0;
      } else if (rotwing_state_fw_idle_counter > ROTWING_MIN_THRUST_IDLE_COUNTER
                 && rotwing_state_skewing_counter == 0) {
        rotwing_state.current_state = ROTWING_STATE_FW_HOV_MOT_IDLE;
        rotwing_state_skewing_counter = 0;
        rotwing_state_fw_idle_counter = 0;
      }
      break;

    case ROTWING_STATE_FW_HOV_MOT_IDLE:
      // Check if state needs to be set to fixed wing with hover motors activated
      if (stabilization_cmd[COMMAND_THRUST] > ROTWING_MIN_THRUST_IDLE
          || rotwing_state.desired_state < ROTWING_STATE_FW_HOV_MOT_IDLE) {
        rotwing_state_fw_counter++;
      } else {
        rotwing_state_fw_counter = 0;
      }

      // Check if state needs to be set to fixed wing with hover motors off (if zero rpm on hover motors)
      if (rotwing_state_hover_rpm[0] == 0
          && rotwing_state_hover_rpm[1] == 0
          && rotwing_state_hover_rpm[2] == 0
          && rotwing_state_hover_rpm[3] == 0) {
#if !USE_NPS
        rotwing_state_fw_m_off_counter++;
#endif
      } else {
        rotwing_state_fw_m_off_counter = 0;
      }

      // Switch state if necessary
      if (rotwing_state_fw_counter > ROTWING_MIN_THRUST_IDLE_COUNTER) {
        rotwing_state.current_state = ROTWING_STATE_FW;
        rotwing_state_fw_counter = 0;
        rotwing_state_fw_m_off_counter = 0;
      } else if (rotwing_state_fw_m_off_counter > ROTWING_HOV_MOT_OFF_COUNTER
                 && rotwing_state_fw_counter == 0) {
        rotwing_state.current_state = ROTWING_STATE_FW_HOV_MOT_OFF;
        rotwing_state_fw_counter = 0;
        rotwing_state_fw_m_off_counter = 0;
      }
      break;

    case ROTWING_STATE_FW_HOV_MOT_OFF:
      // Check if state needs to be set to fixed wing with hover motors idle (if rpm on hover motors)
      if (rotwing_state_hover_rpm[0] > ROTWING_HOV_MOT_OFF_RPM_TH
          && rotwing_state_hover_rpm[1] > ROTWING_HOV_MOT_OFF_RPM_TH
          && rotwing_state_hover_rpm[2] > ROTWING_HOV_MOT_OFF_RPM_TH
          && rotwing_state_hover_rpm[3] > ROTWING_HOV_MOT_OFF_RPM_TH) {
        rotwing_state_fw_idle_counter++;
      } else {
        rotwing_state_fw_idle_counter = 0;
      }

      // Switch state if necessary
      if (rotwing_state_fw_idle_counter > ROTWING_MIN_THRUST_IDLE_COUNTER) {
        rotwing_state.current_state = ROTWING_STATE_FW_HOV_MOT_IDLE;
        rotwing_state_fw_idle_counter = 0;
      }
      break;

    default:
      break;
  }
}

// Function that handles settings for switching state(s)
void rotwing_switch_state(void)
{
  switch (rotwing_state.current_state) {
    case ROTWING_STATE_HOVER:
      if (rotwing_state.desired_state > ROTWING_STATE_HOVER) {
        rotwing_state_set_skewing_settings();
      } else {
        rotwing_state_set_hover_settings();
      }
      break;

    case ROTWING_STATE_SKEWING:
      if (rotwing_state.desired_state < ROTWING_STATE_SKEWING && stateGetAirspeed_f() < ROTWING_MAX_QUAD_AIRSPEED) {
        rotwing_state_set_hover_settings();
      } else {
        rotwing_state_set_skewing_settings();
      }
      break;

    case ROTWING_STATE_FW:
      if (rotwing_state.desired_state > ROTWING_STATE_FW) {
        rotwing_state_set_fw_hov_mot_idle_settings();
      } else if (rotwing_state.desired_state < ROTWING_STATE_FW) {
        rotwing_state_set_skewing_settings();
      } else {
        rotwing_state_set_fw_settings();
      }
      break;

    case ROTWING_STATE_FW_HOV_MOT_IDLE:
      if (rotwing_state.desired_state > ROTWING_STATE_FW_HOV_MOT_IDLE) {
        rotwing_state_set_fw_hov_mot_off_settings();
      } else if (rotwing_state.desired_state < ROTWING_STATE_FW_HOV_MOT_IDLE) {
        rotwing_state_set_fw_settings();
      } else {
        rotwing_state_set_fw_hov_mot_idle_settings();
      }
      break;

    case ROTWING_STATE_FW_HOV_MOT_OFF:
      if (rotwing_state.desired_state < ROTWING_STATE_FW_HOV_MOT_OFF) {
        rotwing_state_set_fw_hov_mot_idle_settings();
      } else {
        rotwing_state_set_fw_hov_mot_off_settings();
      }
      break;
  }
}

void rotwing_state_set_hover_settings(void)
{
  rotwing_state_settings.wing_scheduler       = ROTWING_STATE_WING_QUAD_SETTING;
  rotwing_state_settings.hover_motors_active  = true;
  rotwing_state_settings.hover_motors_disable = false;
  rotwing_state_settings.force_forward        = false;
  rotwing_state_settings.preferred_pitch      = ROTWING_STATE_PITCH_QUAD_SETTING;
  rotwing_state_settings.stall_protection     = false;
  rotwing_state_settings.max_v_climb          = 2.0;
  rotwing_state_settings.max_v_descend        = 1.0;
  rotwing_state_settings.nav_max_speed        = rotwing_state_max_hover_speed; // Using setting
  rotwing_state_set_state_settings();
}

void rotwing_state_set_skewing_settings(void)
{
  // Wing may be skewed to quad when desired state is hover and skewing settings set by state machine
  if (rotwing_state.desired_state == ROTWING_STATE_HOVER) {
    rotwing_state_settings.wing_scheduler     = ROTWING_STATE_WING_QUAD_SETTING;
  } else {
    rotwing_state_settings.wing_scheduler     = ROTWING_STATE_WING_SCHEDULING_SETTING;
  }
  rotwing_state_settings.hover_motors_active  = true;
  rotwing_state_settings.hover_motors_disable = false;
  rotwing_state_settings.force_forward        = false;
  rotwing_state_settings.preferred_pitch      = ROTWING_STATE_PITCH_TRANSITION_SETTING;
  rotwing_state_settings.stall_protection     = false;
  rotwing_state_settings.max_v_climb          = 2.0;
  rotwing_state_settings.max_v_descend        = 1.0;
  rotwing_state_settings.nav_max_speed        = 100; // Big as we use airspeed guidance
  rotwing_state_set_state_settings();
}

void rotwing_state_set_fw_settings(void)
{
  rotwing_state_settings.wing_scheduler       = ROTWING_STATE_WING_FW_SETTING;
  rotwing_state_settings.hover_motors_active  = true;
  rotwing_state_settings.hover_motors_disable = false;
  rotwing_state_settings.force_forward        = true;
  rotwing_state_settings.preferred_pitch      = ROTWING_STATE_PITCH_FW_SETTING;
  rotwing_state_settings.stall_protection     = false;
  rotwing_state_settings.max_v_climb          = 4.0;
  rotwing_state_settings.max_v_descend        = 4.0;
  rotwing_state_settings.nav_max_speed        = 100; // Big as we use airspeed guidance
  rotwing_state_set_state_settings();
}

void rotwing_state_set_fw_hov_mot_idle_settings(void)
{
  rotwing_state_settings.wing_scheduler       = ROTWING_STATE_WING_FW_SETTING;
  rotwing_state_settings.hover_motors_active  = false;
  rotwing_state_settings.hover_motors_disable = false;
  rotwing_state_settings.force_forward        = true;
  rotwing_state_settings.preferred_pitch      = ROTWING_STATE_PITCH_FW_SETTING;
  rotwing_state_settings.stall_protection     = true;
  rotwing_state_settings.max_v_climb          = 4.0;
  rotwing_state_settings.max_v_descend        = 4.0;
  rotwing_state_settings.nav_max_speed        = 100; // Big as we use airspeed guidance
  rotwing_state_set_state_settings();
}

void rotwing_state_set_fw_hov_mot_off_settings(void)
{
  rotwing_state_settings.wing_scheduler       = ROTWING_STATE_WING_FW_SETTING;
  rotwing_state_settings.hover_motors_active  = false;
  rotwing_state_settings.hover_motors_disable = true;
  rotwing_state_settings.force_forward        = true;
  rotwing_state_settings.preferred_pitch      = ROTWING_STATE_PITCH_FW_SETTING;
  rotwing_state_settings.stall_protection     = true;
  rotwing_state_settings.max_v_climb          = 4.0;
  rotwing_state_settings.max_v_descend        = 4.0;
  rotwing_state_settings.nav_max_speed        = 100; // Big as we use airspeed guidance
  rotwing_state_set_state_settings();
}

void rotwing_state_set_state_settings(void)
{

  if (!rotwing_state_skewing.force_rotation_angle) {
    switch (rotwing_state_settings.wing_scheduler) {
      case ROTWING_STATE_WING_QUAD_SETTING:
        rotwing_state_skewing.airspeed_scheduling = false;
        rotwing_state_skewing.wing_angle_deg_sp = 0;
        break;
      case ROTWING_STATE_WING_SCHEDULING_SETTING:
        rotwing_state_skewing.airspeed_scheduling = true;
        break;
      case ROTWING_STATE_WING_FW_SETTING:
        rotwing_state_skewing.airspeed_scheduling = true;
        break;
    }
  } else {
    rotwing_state_skewing.airspeed_scheduling = false;
  }

  hover_motors_active = rotwing_state_settings.hover_motors_active;

  bool_disable_hover_motors = rotwing_state_settings.hover_motors_disable;

  force_forward = rotwing_state_settings.force_forward;

  nav_max_speed = rotwing_state_settings.nav_max_speed;

  // TO DO: pitch angle now hard coded scheduled by wing angle

  // Stall protection handled by hover_motors_active boolean

  // TO DO: Climb and descend speeds now handled by guidance airspeed
}

void rotwing_state_skewer(void)
{
  if (rotwing_state_skewing.airspeed_scheduling) {
    float wing_angle_scheduled_sp_deg = 0;
    float airspeed = stateGetAirspeed_f();
    if (airspeed < 8) {
      wing_angle_scheduled_sp_deg = 0;
    } else if (airspeed < 10 && (rotwing_state.desired_state > ROTWING_STATE_HOVER)) {
      wing_angle_scheduled_sp_deg = 55;
    } else if (airspeed > 10) {
      wing_angle_scheduled_sp_deg = ((airspeed - 10.)) / 4. * 35. + 55.;
    } else {
      wing_angle_scheduled_sp_deg = 0;
    }

    Bound(wing_angle_scheduled_sp_deg, 0., 90.)
    rotwing_state_skewing.wing_angle_deg_sp = wing_angle_scheduled_sp_deg;
  }
}

void rotwing_state_skew_actuator_periodic(void)
{

  // calc rotation percentage of setpoint (0 deg = -1, 45 deg = 0, 90 deg = 1)
  float wing_rotation_percentage = (rotwing_state_skewing.wing_angle_deg_sp - 45.) / 45.;
  Bound(wing_rotation_percentage, -1., 1.);

  float servo_pprz_cmd = MAX_PPRZ * wing_rotation_percentage;
  // Calulcate rotation_cmd
  Bound(servo_pprz_cmd, -MAX_PPRZ, MAX_PPRZ);

#if ROTWING_STATE_USE_ROTATION_REF_MODEL
  // Rotate with second order filter
  static float rotwing_state_skew_p_cmd = -MAX_PPRZ;
  static float rotwing_state_skew_d_cmd = 0;
  float speed_sp  = 0.001 * (servo_pprz_cmd - rotwing_state_skew_p_cmd);
  rotwing_state_skew_d_cmd += 0.003 * (speed_sp - rotwing_state_skew_d_cmd);
  rotwing_state_skew_p_cmd += rotwing_state_skew_d_cmd;
  Bound(rotwing_state_skew_p_cmd, -MAX_PPRZ, MAX_PPRZ);
  rotwing_state_skewing.servo_pprz_cmd = rotwing_state_skew_p_cmd;
#else
  // Directly controlling the wing rotation
  rotwing_state_skewing.servo_pprz_cmd = servo_pprz_cmd;
#endif

#if USE_NPS
  // Export to the index of the SKEW in the NPS_ACTUATOR_NAMES array
  actuators_pprz[INDI_NUM_ACT] = (rotwing_state_skewing.servo_pprz_cmd + MAX_PPRZ) / 2.; // Scale to simulation command

  // Simulate wing angle from command
  rotwing_state_skewing.wing_angle_deg = (float) rotwing_state_skewing.servo_pprz_cmd / MAX_PPRZ * 45. + 45.;

  // SEND ABI Message to ctr_eff_sched and other modules that want Actuator position feedback
  struct act_feedback_t feedback;
  feedback.idx =  SERVO_ROTATION_MECH_IDX;
  feedback.position = 0.5 * M_PI - RadOfDeg(rotwing_state_skewing.wing_angle_deg);
  feedback.set.position = true;

  // Send ABI message
  AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_UAVCAN_ID, &feedback, 1);
#endif
}

static void rotwing_state_feedback_cb(uint8_t __attribute__((unused)) sender_id,
                                      struct act_feedback_t UNUSED *feedback_msg, uint8_t UNUSED num_act_message)
{
  for (int i = 0; i < num_act_message; i++) {

    for (int i = 0; i < num_act_message; i++) {
      // Check for wing rotation feedback
      if ((feedback_msg[i].set.position) && (feedback_msg[i].idx == SERVO_ROTATION_MECH_IDX)) {
        // Get wing rotation angle from sensor
        float wing_angle_rad = 0.5 * M_PI - feedback_msg[i].position;
        rotwing_state_skewing.wing_angle_deg = DegOfRad(wing_angle_rad);

        // Bound wing rotation angle
        Bound(rotwing_state_skewing.wing_angle_deg, 0, 90.);
      }
    }

    // Sanity check that index is valid
    int idx = feedback_msg[i].idx;
    if (feedback_msg[i].set.rpm) {
      if ((idx == SERVO_MOTOR_FRONT_IDX) || (idx == SERVO_BMOTOR_FRONT_IDX)) {
        rotwing_state_hover_rpm[0] = feedback_msg->rpm;
      } else if ((idx == SERVO_MOTOR_RIGHT_IDX) || (idx == SERVO_BMOTOR_RIGHT_IDX)) {
        rotwing_state_hover_rpm[1] = feedback_msg->rpm;
      } else if ((idx == SERVO_MOTOR_BACK_IDX) || (idx == SERVO_BMOTOR_BACK_IDX)) {
        rotwing_state_hover_rpm[2] = feedback_msg->rpm;
      } else if ((idx == SERVO_MOTOR_LEFT_IDX) || (idx == SERVO_BMOTOR_LEFT_IDX)) {
        rotwing_state_hover_rpm[3] = feedback_msg->rpm;
      }
    }
  }
}

void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle)
{
  float pitch_priority_factor = 11.;
  float roll_priority_factor = 10.;
  float thrust_priority_factor = 7.;
  float pusher_priority_factor = 30.;

  float horizontal_accel_weight = 10.;
  float vertical_accel_weight = 10.;

  // Set weights
  Wu_gih[0] = roll_priority_factor * 10.414;
  Wu_gih[1] = pitch_priority_factor * 27.53;
  Wu_gih[2] = thrust_priority_factor * 0.626;
  Wu_gih[3] = pusher_priority_factor * 1.0;

  // adjust weights
  float thrust_command = (actuator_state_filt_vect[0] + actuator_state_filt_vect[1] + actuator_state_filt_vect[2] +
                          actuator_state_filt_vect[3]) / 4;
  Bound(thrust_command, 0, MAX_PPRZ);
  float fixed_wing_percentage = !hover_motors_active; // TODO: when hover props go below 40%, ...
  Bound(fixed_wing_percentage, 0, 1);
#define AIRSPEED_IMPORTANCE_IN_FORWARD_WEIGHT 16

  Wv_gih[0] = horizontal_accel_weight * (1.0f + fixed_wing_percentage *
                                         AIRSPEED_IMPORTANCE_IN_FORWARD_WEIGHT); // stall n low hover motor_off (weight 16x more important than vertical weight)
  Wv_gih[1] = horizontal_accel_weight;
  Wv_gih[2] = vertical_accel_weight;

  struct FloatEulers eulers_zxy;
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  // Evaluate motors_on boolean
  if (!hover_motors_active) {
    if (stateGetAirspeed_f() < 15.) {
      hover_motors_active = true;
      bool_disable_hover_motors = false;
    } else if (eulers_zxy.theta > RadOfDeg(15.0)) {
      hover_motors_active = true;
      bool_disable_hover_motors = false;
    }
  } else {
    bool_disable_hover_motors = false;
  }

  float du_min_thrust_z = ((MAX_PPRZ - actuator_state_filt_vect[0]) * g1g2[3][0] + (MAX_PPRZ -
                           actuator_state_filt_vect[1]) * g1g2[3][1] + (MAX_PPRZ - actuator_state_filt_vect[2]) * g1g2[3][2] +
                           (MAX_PPRZ - actuator_state_filt_vect[3]) * g1g2[3][3]) * hover_motors_active;
  Bound(du_min_thrust_z, -50., 0.);
  float du_max_thrust_z = -(actuator_state_filt_vect[0] * g1g2[3][0] + actuator_state_filt_vect[1] * g1g2[3][1] +
                            actuator_state_filt_vect[2] * g1g2[3][2] + actuator_state_filt_vect[3] * g1g2[3][3]);
  Bound(du_max_thrust_z, 0., 50.);

  float roll_limit_rad = 2.0; // big roll limit hacked in to overcome wls problems at roll limit
  float max_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MAX_PITCH);
  float min_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MIN_PITCH);

  float scheduled_pitch_angle = 0;
  float pitch_angle_range = 3.;
  if (rotwing_state_skewing.wing_angle_deg < 55) {
    scheduled_pitch_angle = 0;
  } else {
    float pitch_progression = (rotwing_state_skewing.wing_angle_deg - 55) / 35.;
    scheduled_pitch_angle = pitch_angle_range * pitch_progression;
  }
  if (!hover_motors_active) {
    scheduled_pitch_angle = 8.;
  }
  Bound(scheduled_pitch_angle, -5., 8.);
  guidance_indi_pitch_pref_deg = scheduled_pitch_angle;

  float pitch_pref_rad = RadOfDeg(guidance_indi_pitch_pref_deg);

  // Set lower limits
  du_min_gih[0] = -roll_limit_rad - roll_angle; //roll
  du_min_gih[1] = min_pitch_limit_rad - pitch_angle; // pitch
  du_min_gih[2] = du_min_thrust_z;
  du_min_gih[3] = (-actuator_state_filt_vect[8] * g1g2[4][8]);

  // Set upper limits limits
  du_max_gih[0] = roll_limit_rad - roll_angle; //roll
  du_max_gih[1] = max_pitch_limit_rad - pitch_angle; // pitch
  du_max_gih[2] = du_max_thrust_z;
  du_max_gih[3] = 9.0; // Hacky value to prevent drone from pitching down in transition

  // Set prefered states
  du_pref_gih[0] = 0; // prefered delta roll angle
  du_pref_gih[1] = -pitch_angle + pitch_pref_rad;// prefered delta pitch angle
  du_pref_gih[2] = du_max_gih[2]; // Low thrust better for efficiency
  du_pref_gih[3] = body_v[0]; // solve the body acceleration
}
