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

#include "modules/rotwing_drone/rotwing_state.h"
#include "firmwares/rotorcraft/autopilot_firmware.h"
#include "modules/core/commands.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"


/* Minimum measured RPM to consider the hover motors running (RPM) */
#ifndef ROTWING_QUAD_MIN_RPM
#define ROTWING_QUAD_MIN_RPM 400
#endif

/* Minimum measured RPM to consider the pusher motor running (RPM) */
#ifndef ROTWING_PUSH_MIN_RPM
#define ROTWING_PUSH_MIN_RPM 300
#endif

/* Timeout for the RPM feedback (seconds) */
#ifndef ROTWING_RPM_TIMEOUT
#define ROTWING_RPM_TIMEOUT 1.0
#endif

/* Minimum thrust to consider the hover motors idling (PPRZ) */
#ifndef ROTWING_QUAD_IDLE_MIN_THRUST
#define ROTWING_QUAD_IDLE_MIN_THRUST 100
#endif

/* Minimum time for the hover motors to be considered idling (seconds) */
#ifndef ROTWING_QUAD_IDLE_TIMEOUT
#define ROTWING_QUAD_IDLE_TIMEOUT 0.4
#endif

/* Minimum measured skew angle to consider the rotating wing in fixedwing (deg) */
#ifndef ROTWING_FW_SKEW_ANGLE
#define ROTWING_FW_SKEW_ANGLE 85.0
#endif

/* TODO: Give a name.... */
#ifndef ROTWING_SKEW_BACK_MARGIN
#define ROTWING_SKEW_BACK_MARGIN 5.0
#endif

/* Maximum angle difference between the measured skew and modelled skew for skew failure detection */
#ifndef ROTWING_SKEW_REF_MODEL_MAX_DIFF
#define ROTWING_SKEW_REF_MODEL_MAX_DIFF 5.f
#endif

/* Skew angle at which the mininum airspeed starts its linear portion */
#ifndef ROTWING_MIN_AIRSPEED_SLOPE_START_ANGLE
#define ROTWING_MIN_AIRSPEED_SLOPE_START_ANGLE 30.0
#endif

/* Amount of time the airspeed needs to be below the FW_MIN_AIRSPEED */
#ifndef ROTWING_FW_STALL_TIMEOUT
#define ROTWING_FW_STALL_TIMEOUT 0.5
#endif

/* Fix for not having double can busses */
#ifndef SERVO_BMOTOR_PUSH_IDX
#define SERVO_BMOTOR_PUSH_IDX SERVO_MOTOR_PUSH_IDX
#endif

/* Fix for not having double can busses */
#ifndef SERVO_BROTATION_MECH_IDX
#define SERVO_BROTATION_MECH_IDX SERVO_ROTATION_MECH_IDX
#endif

/** ABI binding feedback data */
#ifndef ROTWING_STATE_ACT_FEEDBACK_ID
#define ROTWING_STATE_ACT_FEEDBACK_ID ABI_BROADCAST
#endif
abi_event rotwing_state_feedback_ev;
static void rotwing_state_feedback_cb(uint8_t sender_id, struct act_feedback_t *feedback_msg, uint8_t num_act);
struct rotwing_state_t rotwing_state;
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_rotating_wing_state(struct transport_tx *trans, struct link_device *dev)
{
  // Set the status
  union rotwing_bitmask_t status;
  status.value = 0;
  status.skew_angle_valid = rotwing_state_skew_angle_valid();
  status.hover_motors_enabled = rotwing_state.hover_motors_enabled;
  status.hover_motors_idle = rotwing_state_hover_motors_idling();
  status.hover_motors_running = rotwing_state_hover_motors_running();
  status.pusher_motor_running = rotwing_state_pusher_motor_running();
  status.skew_forced = rotwing_state.force_skew;

  // Send the message
  uint8_t state = rotwing_state.state;
  uint8_t nav_state = rotwing_state.nav_state;
  pprz_msg_send_ROTATING_WING_STATE(trans, dev, AC_ID,
                                    &state,
                                    &nav_state,
                                    &status.value,
                                    &rotwing_state.meas_skew_angle_deg,
                                    &rotwing_state.sp_skew_angle_deg,
                                    &gi_unbounded_airspeed_sp,
                                    &rotwing_state.min_airspeed,
                                    &rotwing_state.max_airspeed);
}
#endif // PERIODIC_TELEMETRY

void rotwing_state_init(void)
{
  // Initialize rotwing state
  rotwing_state.state = ROTWING_STATE_FORCE_HOVER; // For takeoff
  rotwing_state.nav_state = ROTWING_STATE_FORCE_HOVER;
  rotwing_state.hover_motors_enabled = true;
  rotwing_state.fw_min_airspeed = ROTWING_FW_MIN_AIRSPEED;
  rotwing_state.cruise_airspeed = ROTWING_FW_CRUISE_AIRSPEED;
  rotwing_state.sp_skew_angle_deg = 0;
  rotwing_state.meas_skew_angle_deg = 0;
  rotwing_state.meas_skew_angle_time = 0;
  rotwing_state.skew_cmd = 0;
  rotwing_state.force_skew = false;
  for (int i = 0; i < 5; i++) {
    rotwing_state.meas_rpm[i] = 0;
    rotwing_state.meas_rpm_time[i] = 0;
  }
  rotwing_state.fail_skew_angle = false;
  rotwing_state.fail_hover_motor = false;
  rotwing_state.fail_pusher_motor = false;
  rotwing_state.ref_model_skew_angle_deg = 0;
  // Bind ABI messages
  AbiBindMsgACT_FEEDBACK(ROTWING_STATE_ACT_FEEDBACK_ID, &rotwing_state_feedback_ev, rotwing_state_feedback_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTATING_WING_STATE, send_rotating_wing_state);
#endif
}

/**
 * @brief Check if hover motors are idling (COMMAND_THRUST < ROTWING_QUAD_IDLE_MIN_THRUST) for ROTWING_QUAD_IDLE_TIMEOUT time
 * @return true if hover motors are idling, false otherwise
 */
bool rotwing_state_hover_motors_idling(void) {
  static float last_idle_time = 0;
  // Check if hover motors are idling and reset timer
  if(stabilization.cmd[COMMAND_THRUST] > ROTWING_QUAD_IDLE_MIN_THRUST) {
    last_idle_time = get_sys_time_float();
  }

  // Check if we exceeded the idle timeout
  if(get_sys_time_float() - last_idle_time > ROTWING_QUAD_IDLE_TIMEOUT) {
    return true;
  }
  return false;
}

void rotwing_state_periodic(void)
{
  /* Get some genericly used variables */
  static float last_stall_time = 0;
  float current_time = get_sys_time_float();
  float meas_airspeed = stateGetAirspeed_f();
  float meas_skew_angle = rotwing_state.meas_skew_angle_deg;
  Bound(meas_skew_angle, 0, 90); // Bound to prevent errors
  
  if(meas_airspeed > rotwing_state.fw_min_airspeed) {
    last_stall_time = current_time;
  }

  /* Override modes if flying with RC */
  rotwing_state.state = rotwing_state.nav_state;
  if(guidance_h.mode == GUIDANCE_H_MODE_NONE) {
    // Kill mode
    if(stabilization.mode == STABILIZATION_MODE_NONE) {
      rotwing_state.state = ROTWING_STATE_FORCE_HOVER;
    }
    // ATT and ATT_FWD
    else if(stabilization.mode == STABILIZATION_MODE_ATTITUDE) {
      if (stabilization.att_submode == STABILIZATION_ATT_SUBMODE_FORWARD) {
        rotwing_state.state = ROTWING_STATE_FORCE_FW;
      } else {
        rotwing_state.state = ROTWING_STATE_FORCE_HOVER;
      }
    }
  }

  /* Handle the quad motors on/off state */
  if(rotwing_state.state == ROTWING_STATE_FORCE_HOVER || rotwing_state.state == ROTWING_STATE_REQUEST_HOVER) {
    rotwing_state.hover_motors_enabled = true;
  }
  else if((current_time - last_stall_time) < ROTWING_FW_STALL_TIMEOUT && rotwing_state_hover_motors_idling() && rotwing_state_pusher_motor_running() && meas_skew_angle >= ROTWING_FW_SKEW_ANGLE 
        && (gi_unbounded_airspeed_sp >= rotwing_state.fw_min_airspeed || rotwing_state.state != ROTWING_STATE_FREE)) {
    rotwing_state.hover_motors_enabled = false;
  }
  else {
    rotwing_state.hover_motors_enabled = true;
  }


  /* Calculate min/max airspeed bounds based on skew angle */
  float skew_min_airspeed = ROTWING_FW_QUAD_MIN_AIRSPEED * (meas_skew_angle - ROTWING_MIN_AIRSPEED_SLOPE_START_ANGLE) / (90.f - ROTWING_MIN_AIRSPEED_SLOPE_START_ANGLE);
  float skew_max_airspeed = ROTWING_QUAD_MAX_AIRSPEED + (ROTWING_FW_MAX_AIRSPEED - ROTWING_QUAD_MAX_AIRSPEED) * meas_skew_angle / ROTWING_FW_SKEW_ANGLE;
  Bound(skew_min_airspeed, 0, rotwing_state.fw_min_airspeed);
  Bound(skew_max_airspeed, ROTWING_QUAD_MAX_AIRSPEED, ROTWING_FW_MAX_AIRSPEED);

  if(!rotwing_state_hover_motors_running() || !rotwing_state.hover_motors_enabled) {
    skew_min_airspeed = rotwing_state.fw_min_airspeed;
    skew_max_airspeed = ROTWING_FW_MAX_AIRSPEED;
  }


  /* Handle the skew angle setpoint */
  if (rotwing_state.force_skew) {
    // Do nothing
  }
  else if(rotwing_state.state == ROTWING_STATE_FORCE_HOVER) {
    rotwing_state.sp_skew_angle_deg = 0.f;
  }
  else if(!rotwing_state_hover_motors_running() || !rotwing_state.hover_motors_enabled || rotwing_state.state == ROTWING_STATE_FORCE_FW) {
    rotwing_state.sp_skew_angle_deg = 90.f;
  }
  else if(!rotwing_state_pusher_motor_running()) {
    rotwing_state.sp_skew_angle_deg = 0.f;
  }
  else if(rotwing_state.state == ROTWING_STATE_REQUEST_HOVER && meas_skew_angle <= (ROTWING_SKEW_ANGLE_STEP + ROTWING_SKEW_BACK_MARGIN)) {
    rotwing_state.sp_skew_angle_deg = 0.f;
  }
  else {
    // SKEWING function based on Vair
    if (meas_airspeed < ROTWING_SKEW_DOWN_AIRSPEED) {
      rotwing_state.sp_skew_angle_deg = 0.f;
    } else if (meas_airspeed < ROTWING_SKEW_UP_AIRSPEED) {
      // Hysteresis do nothing
    } else if (meas_airspeed < ROTWING_QUAD_MAX_AIRSPEED) {
      rotwing_state.sp_skew_angle_deg = ROTWING_SKEW_ANGLE_STEP;
    } else {
      rotwing_state.sp_skew_angle_deg = ((meas_airspeed - ROTWING_QUAD_MAX_AIRSPEED)) / (rotwing_state.fw_min_airspeed - ROTWING_QUAD_MAX_AIRSPEED) * (90.f - ROTWING_SKEW_ANGLE_STEP) + ROTWING_SKEW_ANGLE_STEP;
    }
  }
  Bound(rotwing_state.sp_skew_angle_deg, 0.f, 90.f);


  /* Handle the airspeed bounding */
  Bound(rotwing_state.cruise_airspeed, rotwing_state.fw_min_airspeed, ROTWING_FW_MAX_AIRSPEED);
  if((!rotwing_state_hover_motors_running() && rotwing_state.state != ROTWING_STATE_FORCE_HOVER) || rotwing_state.state == ROTWING_STATE_FORCE_FW) {
    rotwing_state.min_airspeed = rotwing_state.cruise_airspeed;
    rotwing_state.max_airspeed = rotwing_state.cruise_airspeed;
  }
  else if(!rotwing_state_pusher_motor_running()) {
    rotwing_state.min_airspeed = 0;
    rotwing_state.max_airspeed = ROTWING_QUAD_NOPUSH_AIRSPEED;
  }
  else if(rotwing_state.state == ROTWING_STATE_FORCE_HOVER || rotwing_state.state == ROTWING_STATE_REQUEST_HOVER) {
    rotwing_state.min_airspeed = skew_min_airspeed;
    rotwing_state.max_airspeed = fmax(ROTWING_QUAD_MAX_AIRSPEED, skew_min_airspeed);
  }
  else if(rotwing_state.state == ROTWING_STATE_REQUEST_FW) {
    rotwing_state.min_airspeed = fmin(rotwing_state.cruise_airspeed, skew_max_airspeed);
    rotwing_state.max_airspeed = fmin(rotwing_state.cruise_airspeed, skew_max_airspeed);
  }
  else{
    rotwing_state.min_airspeed = skew_min_airspeed;
    rotwing_state.max_airspeed = skew_max_airspeed;
  }

  // Override failing skewing while fwd
  /*if(meas_skew_angle > 70 && rotwing_state_.skewing_failing) {
    rotwing_state.min_airspeed = 0; // Vstall + margin
    rotwing_state.max_airspeed = 0; // Max airspeed FW
  }*/

  guidance_set_min_max_airspeed(rotwing_state.min_airspeed, rotwing_state.max_airspeed);
  /* Set navigation/guidance settings */
  nav_max_deceleration_sp = ROTWING_FW_MAX_DECELERATION * meas_skew_angle / 90.f + ROTWING_QUAD_MAX_DECELERATION * (90.f - meas_skew_angle) / 90.f; //TODO: Do we really want to based this on the skew?


  /* Calculate the skew command */
  float servo_pprz_cmd = MAX_PPRZ * (rotwing_state.sp_skew_angle_deg - 45.f) / 45.f;
  BoundAbs(servo_pprz_cmd, MAX_PPRZ);

  // Rotate with second order filter
  static float rotwing_state_skew_p_cmd = -MAX_PPRZ;
  static float rotwing_state_skew_d_cmd = 0;

  float speed_sp = ROTWING_SKEW_REF_MODEL_P_GAIN * (servo_pprz_cmd - rotwing_state_skew_p_cmd);
  BoundAbs(speed_sp, ROTWING_SKEW_REF_MODEL_MAX_SPEED);
  rotwing_state_skew_d_cmd += ROTWING_SKEW_REF_MODEL_D_GAIN * (speed_sp - rotwing_state_skew_d_cmd);
  rotwing_state_skew_p_cmd += rotwing_state_skew_d_cmd;
  BoundAbs(rotwing_state_skew_p_cmd, MAX_PPRZ);
  rotwing_state.ref_model_skew_angle_deg = 45.0 / MAX_PPRZ * rotwing_state_skew_p_cmd + 45.0;

#if (ROTWING_SKEW_REF_MODEL || USE_NPS)
  rotwing_state.skew_cmd  = rotwing_state_skew_p_cmd;
#else
  rotwing_state.skew_cmd = servo_pprz_cmd;
#endif
#ifdef COMMAND_ROT_MECH
  commands[COMMAND_ROT_MECH] = rotwing_state.skew_cmd;
#endif

  /* Add simulation feedback for the skewing and RPM */
#if USE_NPS
  // Export to the index of the SKEW in the NPS_ACTUATOR_NAMES array
#ifdef COMMAND_ROT_MECH
  commands[COMMAND_ROT_MECH] = (rotwing_state.skew_cmd + MAX_PPRZ) / 2.f; // Scale to simulation command
#else
  actuators_pprz[INDI_NUM_ACT] = (rotwing_state.skew_cmd + MAX_PPRZ) / 2.f; // Scale to simulation command
#endif
  // SEND ABI Message to ctr_eff_sched, ourself and other modules that want Actuator position feedback
  struct act_feedback_t feedback;
  feedback.idx =  SERVO_ROTATION_MECH_IDX;
  feedback.position = 0.5f * M_PI - RadOfDeg((float) rotwing_state.skew_cmd / MAX_PPRZ * 45.f + 45.f);
  feedback.set.position = true;

  // Send ABI message (or simulate failure)
  if(!rotwing_state.fail_skew_angle) {
    AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_BOARD_ID, &feedback, 1);
  }

  // Simulate to always have RPM if on and active feedback
  rotwing_state.meas_rpm[0] = (actuators[SERVO_MOTOR_FRONT_IDX].pprz_val >= 0)? (ROTWING_QUAD_MIN_RPM + 100) : 0;
  rotwing_state.meas_rpm[1] = (actuators[SERVO_MOTOR_RIGHT_IDX].pprz_val >= 0)? (ROTWING_QUAD_MIN_RPM + 100) : 0;
  rotwing_state.meas_rpm[2] = (actuators[SERVO_MOTOR_BACK_IDX].pprz_val >= 0)? (ROTWING_QUAD_MIN_RPM + 100) : 0;
  rotwing_state.meas_rpm[3] = (actuators[SERVO_MOTOR_LEFT_IDX].pprz_val >= 0)? (ROTWING_QUAD_MIN_RPM + 100) : 0;
  rotwing_state.meas_rpm[4] = (actuators[SERVO_MOTOR_PUSH_IDX].pprz_val >= 0)? (ROTWING_PUSH_MIN_RPM + 100) : 0;
  for(uint8_t i = 0; i < 5; i++) {
    rotwing_state.meas_rpm_time[i] = current_time;
  }

#ifdef SITL
  if(rotwing_state.fail_hover_motor) {
    rotwing_state.meas_rpm[0] =  0;
    rotwing_state.meas_rpm[1] =  0;
    rotwing_state.meas_rpm[2] =  0;
    rotwing_state.meas_rpm[3] =  0;
  }

  if(rotwing_state.fail_pusher_motor) {
    rotwing_state.meas_rpm[4] =  0;
  }
#endif

#endif
}

static void rotwing_state_feedback_cb(uint8_t __attribute__((unused)) sender_id,
                                      struct act_feedback_t UNUSED *feedback_msg, uint8_t UNUSED num_act_message)
{
  float current_time = get_sys_time_float();
  for (int i = 0; i < num_act_message; i++) {
    int idx = feedback_msg[i].idx;

    // Check for wing rotation feedback
    if ((feedback_msg[i].set.position) && (idx == SERVO_ROTATION_MECH_IDX || idx == SERVO_BROTATION_MECH_IDX)) {
      // Get wing rotation angle from sensor
      float skew_angle_rad = 0.5 * M_PI - feedback_msg[i].position;
      rotwing_state.meas_skew_angle_deg = DegOfRad(skew_angle_rad);
      rotwing_state.meas_skew_angle_time = current_time;
    }

    // Get the RPM feedbacks of the motors
    if (feedback_msg[i].set.rpm) {
      if ((idx == SERVO_MOTOR_FRONT_IDX) || (idx == SERVO_BMOTOR_FRONT_IDX)) {
        rotwing_state.meas_rpm[0] = feedback_msg->rpm;
        rotwing_state.meas_rpm_time[0] = current_time;
      } else if ((idx == SERVO_MOTOR_RIGHT_IDX) || (idx == SERVO_BMOTOR_RIGHT_IDX)) {
        rotwing_state.meas_rpm[1] = feedback_msg->rpm;
        rotwing_state.meas_rpm_time[1] = current_time;
      } else if ((idx == SERVO_MOTOR_BACK_IDX) || (idx == SERVO_BMOTOR_BACK_IDX)) {
        rotwing_state.meas_rpm[2] = feedback_msg->rpm;
        rotwing_state.meas_rpm_time[2] = current_time;
      } else if ((idx == SERVO_MOTOR_LEFT_IDX) || (idx == SERVO_BMOTOR_LEFT_IDX)) {
        rotwing_state.meas_rpm[3] = feedback_msg->rpm;
        rotwing_state.meas_rpm_time[3] = current_time;
      } else if ((idx == SERVO_MOTOR_PUSH_IDX) || (idx == SERVO_BMOTOR_PUSH_IDX)) {
        rotwing_state.meas_rpm[4] = feedback_msg->rpm;
        rotwing_state.meas_rpm_time[4] = current_time;
      }
    }
  }
}

bool rotwing_state_hover_motors_running(void) {
  float current_time = get_sys_time_float();
  // Check if hover motors are running
  if (rotwing_state.meas_rpm[0] > ROTWING_QUAD_MIN_RPM
      && rotwing_state.meas_rpm[1] > ROTWING_QUAD_MIN_RPM
      && rotwing_state.meas_rpm[2] > ROTWING_QUAD_MIN_RPM
      && rotwing_state.meas_rpm[3] > ROTWING_QUAD_MIN_RPM
      && (current_time - rotwing_state.meas_rpm_time[0]) < ROTWING_RPM_TIMEOUT
      && (current_time - rotwing_state.meas_rpm_time[1]) < ROTWING_RPM_TIMEOUT
      && (current_time - rotwing_state.meas_rpm_time[2]) < ROTWING_RPM_TIMEOUT
      && (current_time - rotwing_state.meas_rpm_time[3]) < ROTWING_RPM_TIMEOUT) {
    return true;
  } else {
    return false;
  }
}

bool rotwing_state_pusher_motor_running(void) {
  // Check if pusher motor is running
  if (rotwing_state.meas_rpm[4] > ROTWING_PUSH_MIN_RPM && (get_sys_time_float() - rotwing_state.meas_rpm_time[4]) < ROTWING_RPM_TIMEOUT) {
    return true;
  } else {
    return false;
  }
}

bool rotwing_state_skew_angle_valid(void) {
  // Check if the measured skew angle is timed out and the reference model is matching
  bool skew_timeout = (get_sys_time_float() - rotwing_state.meas_skew_angle_time) > ROTWING_RPM_TIMEOUT;
  bool skew_angle_match = fabs(rotwing_state.meas_skew_angle_deg - rotwing_state.ref_model_skew_angle_deg) < ROTWING_SKEW_REF_MODEL_MAX_DIFF;
  
  return (!skew_timeout && skew_angle_match);
}

void rotwing_state_set(enum rotwing_states_t state) {
  rotwing_state.nav_state = state;
}


/* TODO move these rotwing navigation helper functions to an appropriate module/file */
bool rotwing_state_choose_circle_direction(uint8_t wp_id) {
  // Get circle waypoint coordinates in NED
  struct FloatVect3 circ_ned = {.x = waypoints[wp_id].enu_f.y,
                                .y = waypoints[wp_id].enu_f.x,
                                .z = -waypoints[wp_id].enu_f.z};

  // Get drone position coordinates in NED
  struct FloatVect3 pos_ned = {.x = stateGetPositionNed_f()->x,
                               .y = stateGetPositionNed_f()->y,
                               .z = stateGetPositionNed_f()->z};

  // Get vector pointing from drone to waypoint
  struct FloatVect3 vect_pos_circ;
  VECT3_DIFF(vect_pos_circ, circ_ned, pos_ned);

  static struct FloatVect3 x_axis = {.x = 1, .y = 0, .z = 0};
  static struct FloatVect3 z_axis = {.x = 0, .y = 0, .z = 1};
  struct FloatVect3 att_NED;
  struct FloatVect3 cross_att_circ;

  float_rmat_transp_vmult(&att_NED, stateGetNedToBodyRMat_f(), &x_axis);

  VECT3_CROSS_PRODUCT(cross_att_circ, vect_pos_circ, att_NED);
  float y = VECT3_DOT_PRODUCT(cross_att_circ, z_axis);
  float x = VECT3_DOT_PRODUCT(vect_pos_circ, att_NED);
  
  float body_to_wp_angle_rad = atan2f(y, x);

  if (body_to_wp_angle_rad >= 0.f) {
    return true; // CCW circle
  } else {
    return false; // CW circle
  }
}

void rotwing_state_set_transition_wp(uint8_t wp_id) {

  // Get drone position coordinates in NED
  struct EnuCoor_f target_enu = {.x = stateGetPositionNed_f()->y,
                                  .y = stateGetPositionNed_f()->x,
                                  .z = -stateGetPositionNed_f()->z};

  static struct FloatVect3 x_axis = {.x = 1, .y = 0, .z = 0};
  struct FloatVect3 x_att_NED;

  float_rmat_transp_vmult(&x_att_NED, stateGetNedToBodyRMat_f(), &x_axis);

  // set the new transition WP coordinates 100m ahead of the drone
  target_enu.x = 100.f * x_att_NED.y + target_enu.x;
  target_enu.y = 100.f * x_att_NED.x + target_enu.y;

  waypoint_set_enu(wp_id, &target_enu);

  // Send waypoint update every half second
  RunOnceEvery(100 / 2, {
    // Send to the GCS that the waypoint has been moved
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                               &waypoints[wp_id].enu_i.x,
                               &waypoints[wp_id].enu_i.y,
                               &waypoints[wp_id].enu_i.z);
  });

}

void rotwing_state_update_WP_height(uint8_t wp_id, float height) {
  struct EnuCoor_f target_enu = {.x = waypoints[wp_id].enu_f.x,
                                 .y = waypoints[wp_id].enu_f.y,
                                 .z = height};
  
  waypoint_set_enu(wp_id, &target_enu);

  DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                               &waypoints[wp_id].enu_i.x,
                               &waypoints[wp_id].enu_i.y,
                               &waypoints[wp_id].enu_i.z);
}
