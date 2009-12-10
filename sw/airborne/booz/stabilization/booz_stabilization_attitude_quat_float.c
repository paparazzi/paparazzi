/*
 * $Id: booz_stabilization_attitude_euler.c 3787 2009-07-24 15:33:54Z poine $
 *  
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

#include "booz_stabilization.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "booz_ahrs.h"
#include "booz_radio_control.h"

#include "airframe.h"


struct FloatVect3  booz_stabilization_pgain;
struct FloatVect3  booz_stabilization_dgain;
struct FloatVect3  booz_stabilization_ddgain;
struct FloatVect3  booz_stabilization_igain;
struct FloatQuat   booz_stabilization_sum_err;
struct FloatEulers booz_stabilization_att_sum_err;

float booz_stabilization_att_fb_cmd[COMMANDS_NB];
float booz_stabilization_att_ff_cmd[COMMANDS_NB];


void booz_stabilization_attitude_init(void) {

  booz_stabilization_attitude_ref_init();

  VECT3_ASSIGN(booz_stabilization_pgain,
	       BOOZ_STABILIZATION_ATTITUDE_PHI_PGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_THETA_PGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_PSI_PGAIN);
  
  VECT3_ASSIGN(booz_stabilization_dgain,
	       BOOZ_STABILIZATION_ATTITUDE_PHI_DGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_THETA_DGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_PSI_DGAIN);
  
  VECT3_ASSIGN(booz_stabilization_igain,
	       BOOZ_STABILIZATION_ATTITUDE_PHI_IGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_THETA_IGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_PSI_IGAIN);

  VECT3_ASSIGN(booz_stabilization_ddgain,
	       BOOZ_STABILIZATION_ATTITUDE_PHI_DDGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_THETA_DDGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_PSI_DDGAIN);

  FLOAT_QUAT_ZERO( booz_stabilization_sum_err );
  FLOAT_EULERS_ZERO( booz_stabilization_att_sum_err );

}

static void reset_psi_ref_from_body(void) {
    booz_stab_att_sp_euler.psi = booz_ahrs_float.ltp_to_body_euler.psi;
    booz_stab_att_ref_euler.psi = booz_ahrs_float.ltp_to_body_euler.psi;
    booz_stab_att_ref_rate.r = 0;
    booz_stab_att_ref_accel.r = 0;
}

// Returns rotation about Y axis from dcm from -2 * pi to 2 * pi
static float get_pitch_rotation_angle(struct FloatRMat *rmat)
{
  float dcm02 = rmat->m[2];
  float dcm22 = rmat->m[8];

  return -atan2f(dcm02, dcm22);
}

// complicated function to reset setpoint quaternion to pitch theta, roll phi using provided
// quaternion initial rotating first about pitch, then roll (and not yaw)
static void reset_sp_quat(float phi, float theta, struct FloatQuat *initial)
{
  float pitch_rotation_angle, roll_rotation_angle;
  struct FloatQuat pitch_axis_quat, roll_axis_quat;

  struct FloatRMat ltp_to_body_rmat;

  struct FloatQuat pitch_rotated_quat, roll_rotated_quat;

  struct FloatVect3 x_axis = { 1, 0, 0 };
  struct FloatVect3 y_axis = { 0, 1, 0 };

  struct FloatEulers rotated_euler;
  
  // Convert body orientation to rotation matrix
  FLOAT_RMAT_OF_QUAT(ltp_to_body_rmat, *initial);

  // compose rotation about Y axis (pitch axis) to theta
  pitch_rotation_angle = theta - get_pitch_rotation_angle(&ltp_to_body_rmat);
  FLOAT_QUAT_OF_AXIS_ANGLE(pitch_axis_quat, y_axis, pitch_rotation_angle);
  FLOAT_QUAT_COMP(pitch_rotated_quat, *initial, pitch_axis_quat);

  // compose rotation about X axis (roll axis) to phi
  FLOAT_EULERS_OF_QUAT(rotated_euler, pitch_rotated_quat);
  roll_rotation_angle = phi - rotated_euler.phi;
  FLOAT_QUAT_OF_AXIS_ANGLE(roll_axis_quat, x_axis, roll_rotation_angle);
  FLOAT_QUAT_COMP(roll_rotated_quat, pitch_rotated_quat, roll_axis_quat);

  // store result into setpoint
  FLOAT_QUAT_COPY(booz_stab_att_sp_quat, roll_rotated_quat);
}

static void update_sp_quat_from_eulers(void) {
    struct FloatRMat sp_rmat;

#ifdef STICKS_RMAT312
    FLOAT_RMAT_OF_EULERS_312(sp_rmat, booz_stab_att_sp_euler);
#else
    FLOAT_RMAT_OF_EULERS_321(sp_rmat, booz_stab_att_sp_euler);
#endif
    FLOAT_QUAT_OF_RMAT(booz_stab_att_sp_quat, sp_rmat);
}

static void update_ref_quat_from_eulers(void) {
    struct FloatRMat ref_rmat;

#ifdef STICKS_RMAT312
    FLOAT_RMAT_OF_EULERS_312(ref_rmat, booz_stab_att_ref_euler);
#else
    FLOAT_RMAT_OF_EULERS_321(ref_rmat, booz_stab_att_ref_euler);
#endif
    FLOAT_QUAT_OF_RMAT(booz_stab_att_ref_quat, ref_rmat);
}

#ifdef USE_VANE
void booz_stabilization_attitude_read_beta_vane(float beta)
{
  struct FloatEulers sticks_eulers;
  struct FloatQuat sticks_quat, prev_sp_quat;

  sticks_eulers.phi = BOOZ_STABILIZATION_ATTITUDE_BETA_VANE_T * beta / RC_UPDATE_FREQ;
  sticks_eulers.theta = 0;
  sticks_eulers.psi = 0;

  // convert eulers to quaternion
  FLOAT_QUAT_OF_EULERS(sticks_quat, sticks_eulers);
  FLOAT_QUAT_COPY(prev_sp_quat, booz_stab_att_sp_quat)
  
  // rotate previous setpoint by commanded rotation
  FLOAT_QUAT_COMP(booz_stab_att_sp_quat, prev_sp_quat, sticks_quat);
}

void booz_stabilization_attitude_read_alpha_vane(float alpha)
{
  struct FloatEulers sticks_eulers;
  struct FloatQuat sticks_quat, prev_sp_quat;
  sticks_eulers.phi = 0;
  sticks_eulers.theta = BOOZ_STABILIZATION_ATTITUDE_ALPHA_VANE_T * alpha / RC_UPDATE_FREQ;
  sticks_eulers.psi = 0;

  // convert eulers to quaternion
  FLOAT_QUAT_OF_EULERS(sticks_quat, sticks_eulers);
  FLOAT_QUAT_COPY(prev_sp_quat, booz_stab_att_sp_quat)
  
  // rotate previous setpoint by commanded rotation
  FLOAT_QUAT_COMP(booz_stab_att_sp_quat, prev_sp_quat, sticks_quat);
}
#endif

void booz_stabilization_attitude_read_rc(bool_t in_flight) {

  uint32_t rate_stick_mode = radio_control.values[RADIO_CONTROL_MODE] < -150;
  static uint32_t last_rate_stick_mode;
  pprz_t roll = radio_control.values[RADIO_CONTROL_ROLL];
  pprz_t pitch = radio_control.values[RADIO_CONTROL_PITCH];
  pprz_t yaw = radio_control.values[RADIO_CONTROL_YAW];
  struct FloatEulers sticks_eulers;
  struct FloatQuat sticks_quat, prev_sp_quat;

  // RC stick commands rate or position?
  if (rate_stick_mode) {
    // convert sticks to commanded rates
    sticks_eulers.phi = APPLY_DEADBAND(roll, BOOZ_STABILIZATION_ATTITUDE_DEADBAND_A) * ROLL_COEF_RATE / RC_UPDATE_FREQ;
    sticks_eulers.theta = APPLY_DEADBAND(pitch, BOOZ_STABILIZATION_ATTITUDE_DEADBAND_E) * PITCH_COEF_RATE / RC_UPDATE_FREQ;
    sticks_eulers.psi = APPLY_DEADBAND(yaw, BOOZ_STABILIZATION_ATTITUDE_DEADBAND_R) * YAW_COEF / RC_UPDATE_FREQ;

    // convert eulers to quaternion
    FLOAT_QUAT_OF_EULERS(sticks_quat, sticks_eulers);
    FLOAT_QUAT_COPY(prev_sp_quat, booz_stab_att_sp_quat)
  
    // rotate previous setpoint by commanded rotation
    FLOAT_QUAT_COMP(booz_stab_att_sp_quat, prev_sp_quat, sticks_quat);
  } else {
    // First time switching from rate to position reset the setpoint based on the body
    if (last_rate_stick_mode) {
      reset_sp_quat(roll * ROLL_COEF, pitch * PITCH_COEF, &booz_ahrs_float.ltp_to_body_quat);
    }

    // heading hold?
    if (in_flight) {
      // compose setpoint based on previous setpoint + pitch/roll sticks
      reset_sp_quat(roll * ROLL_COEF, pitch * PITCH_COEF, &booz_stab_att_sp_quat);

      // get commanded yaw rate from sticks
      sticks_eulers.phi = 0;
      sticks_eulers.theta = 0;
      sticks_eulers.psi = APPLY_DEADBAND(yaw, BOOZ_STABILIZATION_ATTITUDE_DEADBAND_R) * YAW_COEF / RC_UPDATE_FREQ;

      // convert yaw rate * dt into quaternion
      FLOAT_QUAT_OF_EULERS(sticks_quat, sticks_eulers);
      FLOAT_QUAT_COPY(prev_sp_quat, booz_stab_att_sp_quat)
  
      // update setpoint by rotating by yaw command
      FLOAT_QUAT_COMP(booz_stab_att_sp_quat, prev_sp_quat, sticks_quat);
    } else { /* if not flying, use current body position + pitch/roll from sticks to compose setpoint */
      reset_sp_quat(roll * ROLL_COEF, pitch * PITCH_COEF, &booz_ahrs_float.ltp_to_body_quat);
    }
  }
  // update euler setpoints for telemetry
  FLOAT_EULERS_OF_QUAT(booz_stab_att_sp_euler, booz_stab_att_sp_quat);
  last_rate_stick_mode = rate_stick_mode;
}



void booz_stabilization_attitude_enter(void) {

  reset_psi_ref_from_body();
  update_sp_quat_from_eulers();
  update_ref_quat_from_eulers();
  
  FLOAT_EULERS_ZERO( booz_stabilization_att_sum_err );
  FLOAT_QUAT_ZERO( booz_stabilization_sum_err );
  
}


#define IERROR_SCALE 1024

#define MAX_SUM_ERR RadOfDeg(56000)
#include <stdio.h>
void booz_stabilization_attitude_run(bool_t  in_flight) {
#ifdef BOOZ_AHRS_FIXED_POINT
  BOOZ_AHRS_FLOAT_OF_INT32();
#endif 

  
  /* 
   * Update reference
   */
  booz_stabilization_attitude_ref_update();

  /* 
   * Compute feedforward
   */
  /* ref accel bfp is 2^12 and int controller offsets by 5 */
  booz_stabilization_att_ff_cmd[COMMAND_ROLL] = 
    booz_stabilization_ddgain.x * BFP_OF_REAL(booz_stab_att_ref_accel.p, (12-5));
  booz_stabilization_att_ff_cmd[COMMAND_PITCH] = 
    booz_stabilization_ddgain.y * BFP_OF_REAL(booz_stab_att_ref_accel.q, (12-5));
  booz_stabilization_att_ff_cmd[COMMAND_YAW] = 
    booz_stabilization_ddgain.z * BFP_OF_REAL(booz_stab_att_ref_accel.r, (12-5));

  /*
   * Compute feedback                        
   */

  /* attitude error                          */
  struct FloatQuat att_err; 
  FLOAT_QUAT_INV_COMP(att_err, booz_ahrs_float.ltp_to_body_quat, booz_stab_att_ref_quat);
  /* wrap it in the shortest direction       */
  FLOAT_QUAT_WRAP_SHORTEST(att_err);  

  if (in_flight) {
    struct FloatQuat new_sum_err, scaled_att_err;
    /* update accumulator */
    FLOAT_QUAT_COPY(scaled_att_err, att_err);
    scaled_att_err.qx /= IERROR_SCALE;
    scaled_att_err.qy /= IERROR_SCALE;
    scaled_att_err.qz /= IERROR_SCALE;
    FLOAT_QUAT_COMP_INV(new_sum_err, booz_stabilization_sum_err, scaled_att_err);
    FLOAT_QUAT_NORMALISE(new_sum_err);
    FLOAT_QUAT_COPY(booz_stabilization_sum_err, new_sum_err);
  }
  else {
    /* reset accumulator */
    FLOAT_EULERS_ZERO(booz_stabilization_att_sum_err);
    FLOAT_QUAT_ZERO( booz_stabilization_sum_err );
  }
  
  /*  rate error                */
  struct FloatRates rate_err;
  RATES_DIFF(rate_err, booz_ahrs_float.body_rate, booz_stab_att_ref_rate);

  /*  PID                  */
  booz_stabilization_att_fb_cmd[COMMAND_ROLL] = 
    -2. * booz_stabilization_pgain.x  * QUAT1_BFP_OF_REAL(att_err.qx)+
    booz_stabilization_dgain.x  * RATE_BFP_OF_REAL(rate_err.p) +
    booz_stabilization_igain.x  * QUAT1_BFP_OF_REAL(booz_stabilization_sum_err.qx);

  booz_stabilization_att_fb_cmd[COMMAND_PITCH] = 
    -2. * booz_stabilization_pgain.y  * QUAT1_BFP_OF_REAL(att_err.qy)+
    booz_stabilization_dgain.y  * RATE_BFP_OF_REAL(rate_err.q) +
    booz_stabilization_igain.y  * QUAT1_BFP_OF_REAL(booz_stabilization_sum_err.qy);
  
  booz_stabilization_att_fb_cmd[COMMAND_YAW] = 
    -2. * booz_stabilization_pgain.z  * QUAT1_BFP_OF_REAL(att_err.qz)+
    booz_stabilization_dgain.z  * RATE_BFP_OF_REAL(rate_err.r) +
    booz_stabilization_igain.z  * QUAT1_BFP_OF_REAL(booz_stabilization_sum_err.qz);


  booz_stabilization_cmd[COMMAND_ROLL] = 
    ((booz_stabilization_att_fb_cmd[COMMAND_ROLL]+booz_stabilization_att_ff_cmd[COMMAND_ROLL]))/(1<<16);
  booz_stabilization_cmd[COMMAND_PITCH] = 
    ((booz_stabilization_att_fb_cmd[COMMAND_PITCH]+booz_stabilization_att_ff_cmd[COMMAND_PITCH]))/(1<<16);
  booz_stabilization_cmd[COMMAND_YAW] = 
    ((booz_stabilization_att_fb_cmd[COMMAND_YAW]+booz_stabilization_att_ff_cmd[COMMAND_YAW]))/(1<<16);
    
}



