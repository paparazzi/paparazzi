/*
 *
 * Copyright (C) 2008-2011 Joby Energy Inc
 *
 */

/** \file quat_setpoint.c
 *  \brief Quaternion setpoint generation
 *
 */

#include "subsystems/ahrs.h"

#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "stabilization/quat_setpoint.h"
#include "stabilization.h"

#include "messages.h"
#include "mcu_periph/uart.h"
#include "downlink.h"

#define QUAT_SETPOINT_HOVER_PITCH RadOfDeg(90)

// reset to "hover" setpoint
static void reset_sp_quat(int32_t _psi, int32_t _theta, struct Int32Quat *initial)
{
  int32_t pitch_rotation_angle;
  struct Int32Quat pitch_axis_quat;

  struct Int32Quat pitch_rotated_quat, pitch_rotated_quat2;

  struct Int32Vect3 y_axis = { 0, 1, 0 };

  struct Int32Eulers rotated_eulers;

  // compose rotation about Y axis (pitch axis) from hover
  pitch_rotation_angle = ANGLE_BFP_OF_REAL(-QUAT_SETPOINT_HOVER_PITCH);
  INT32_QUAT_OF_AXIS_ANGLE(pitch_axis_quat, y_axis, pitch_rotation_angle);
  INT32_QUAT_COMP_NORM_SHORTEST(pitch_rotated_quat, *initial, pitch_axis_quat);

  INT32_EULERS_OF_QUAT(rotated_eulers, pitch_rotated_quat);

  // reset euler angles
  rotated_eulers.theta = _theta;
  rotated_eulers.phi = _psi;

  INT32_QUAT_OF_EULERS(pitch_rotated_quat, rotated_eulers);

  // compose rotation about Y axis (pitch axis) to hover
  pitch_rotation_angle = ANGLE_BFP_OF_REAL(QUAT_SETPOINT_HOVER_PITCH);
  INT32_QUAT_OF_AXIS_ANGLE(pitch_axis_quat, y_axis, pitch_rotation_angle);
  INT32_QUAT_COMP_NORM_SHORTEST(pitch_rotated_quat2, pitch_rotated_quat, pitch_axis_quat);

  // store result into setpoint
  QUAT_COPY(stab_att_sp_quat, pitch_rotated_quat2);
}

static void update_sp_quat_from_eulers(void) {
    struct Int32RMat sp_rmat;

#ifdef STICKS_RMAT312
    INT32_RMAT_OF_EULERS_312(sp_rmat, stab_att_sp_euler);
#else
    INT32_RMAT_OF_EULERS_321(sp_rmat, stab_att_sp_euler);
#endif
    INT32_QUAT_OF_RMAT(stab_att_sp_quat, sp_rmat);
    INT32_QUAT_WRAP_SHORTEST(stab_att_sp_quat);
}

/*
void stabilization_attitude_read_rc_incremental(bool_t enable_alpha_vane, bool_t enable_beta_vane)
{
  pprz_t roll = radio_control.values[RADIO_ROLL];
  pprz_t pitch = radio_control.values[RADIO_PITCH];
  pprz_t yaw = radio_control.values[RADIO_YAW];
  struct Int32Quat prev_sp_quat, q_e2s, temp_quat;

  struct Int32RMat R_e2s;
  struct Int32Rates sticks_w_bn_e, sticks_w_bn_s;

  QUAT_COPY(prev_sp_quat, stab_att_sp_quat);

  sticks_w_bn_e.p = APPLY_DEADBAND(roll, STABILIZATION_ATTITUDE_DEADBAND_A) * ROLL_COEF_H;
  sticks_w_bn_e.q = APPLY_DEADBAND(pitch, STABILIZATION_ATTITUDE_DEADBAND_E) * PITCH_COEF_RATE;
  sticks_w_bn_e.r = APPLY_DEADBAND(yaw, STABILIZATION_ATTITUDE_DEADBAND_R) * YAW_COEF_RATE;

  // Don't let the sticks command a theta rotation in vane mode
  if (enable_alpha_vane) {
    sticks_w_bn_e.q = 0;
  }

  if (enable_beta_vane) {
    sticks_w_bn_e.r = 0;
  }
  // q_e2s = q_sp (comp) q_b^(-1)
  INT_QUAT_INV_COMP_NORM_SHORTEST(q_e2s, ahrs.ltp_to_body_quat, stab_att_sp_quat);

  INT_RMAT_OF_QUAT(R_e2s, q_e2s);

  INT_RMAT_RATEMULT(sticks_w_bn_s, R_e2s, sticks_w_bn_e);

  INT_QUAT_DIFFERENTIAL(temp_quat, sticks_w_bn_s, 1/RC_UPDATE_FREQ);

  INT32_QUAT_COMP_NORM_SHORTEST(stab_att_sp_quat, prev_sp_quat, temp_quat);

  // update euler setpoints for telemetry
  INT32_EULERS_OF_QUAT(stab_att_sp_euler, stab_att_sp_quat);
}
*/

void stabilization_attitude_read_rc_absolute(struct Int32Eulers sp, bool_t in_flight) {

#ifdef AIRPLANE_STICKS
  pprz_t roll = radio_control.values[RADIO_ROLL];
  pprz_t pitch = radio_control.values[RADIO_PITCH];
  pprz_t yaw = radio_control.values[RADIO_YAW];
#else // QUAD STICKS
  pprz_t roll = radio_control.values[RADIO_YAW];
  pprz_t pitch = radio_control.values[RADIO_PITCH];
  pprz_t yaw = -radio_control.values[RADIO_ROLL];
#endif
  struct Int32Eulers sticks_eulers;
  struct Int32Quat sticks_quat, prev_sp_quat;

  // heading hold?
  if (in_flight) {
    // compose setpoint based on previous setpoint + pitch/roll sticks
    reset_sp_quat(RATE_BFP_OF_REAL(yaw * YAW_COEF), RATE_BFP_OF_REAL(pitch * PITCH_COEF), &stab_att_sp_quat);

    // get commanded yaw rate from sticks
    sticks_eulers.phi = RATE_BFP_OF_REAL(APPLY_DEADBAND(roll, STABILIZATION_ATTITUDE_DEADBAND_A) * ROLL_COEF / RC_UPDATE_FREQ);
    sticks_eulers.theta = 0;
    sticks_eulers.psi = 0;

    // convert yaw rate * dt into quaternion
    INT32_QUAT_OF_EULERS(sticks_quat, sticks_eulers);
    QUAT_COPY(prev_sp_quat, stab_att_sp_quat)

    // update setpoint by rotating by incremental yaw command
    INT32_QUAT_COMP_NORM_SHORTEST(stab_att_sp_quat, prev_sp_quat, sticks_quat);
  } else { /* if not flying, use current body position + pitch/yaw from sticks to compose setpoint */
    reset_sp_quat(RATE_BFP_OF_REAL(yaw * YAW_COEF), RATE_BFP_OF_REAL(pitch * PITCH_COEF), &ahrs.ltp_to_body_quat);
  }

  // update euler setpoints for telemetry
  INT32_EULERS_OF_QUAT(stab_att_sp_euler, stab_att_sp_quat);
}

void stabilization_attitude_sp_enter()
{
  quat_setpoint_enter_absolute();
}


void quat_setpoint_enter_absolute()
{
  // reset setpoint to "hover"
  reset_sp_quat(0., 0., &ahrs.ltp_to_body_quat);
}
