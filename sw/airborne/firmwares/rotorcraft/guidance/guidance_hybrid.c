/*
 * Copyright (C) 2014 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 * This is code for guidance of hybrid UAVs. It needs a simple velocity
 * model to control the ground velocity of the UAV while estimating the
 * wind velocity.
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

/** @file firmwares/rotorcraft/guidance/guidance_hybrid.c
 *  Guidance controllers (horizontal and vertical) for Hybrid UAV configurations.
 *
 * Functionality:
 * 1) hover with (helicopter) thrust vectoring and align the heading with the wind vector.
 * 2) Forward flight with using pitch and a bit of thrust to control altitude and
 *    heading to control the velocity vector
 * 3) Transition between the two, with the possibility to fly at any airspeed
 */

#include "firmwares/rotorcraft/guidance/guidance_hybrid.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "modules/radio_control/radio_control.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

/* for guidance_v_thrust_coeff */
#include "firmwares/rotorcraft/guidance/guidance_v.h"

// max airspeed for quadshot guidance
#define MAX_AIRSPEED 15
// high res frac for integration of angles
#define INT32_ANGLE_HIGH_RES_FRAC 18

// Variables used for settings
int32_t guidance_hybrid_norm_ref_airspeed;
float alt_pitch_gain = 0.3;
int32_t max_airspeed = MAX_AIRSPEED;
int32_t wind_gain;
int32_t horizontal_speed_gain;
float max_turn_bank;
float turn_bank_gain;

// Private variables
static struct Int32Eulers guidance_hybrid_ypr_sp;
static struct Int32Vect2 guidance_hybrid_airspeed_sp;
static struct Int32Vect2 guidance_h_pos_err;
static struct Int32Vect2 guidance_hybrid_airspeed_ref;
static struct Int32Vect2 wind_estimate;
static struct Int32Vect2 wind_estimate_high_res;
static struct Int32Vect2 guidance_hybrid_ref_airspeed;

static int32_t norm_sp_airspeed_disp;
static int32_t heading_diff_disp;
static int32_t omega_disp;
static int32_t high_res_psi;
static int32_t airspeed_sp_heading_disp;
static bool guidance_hovering;
bool force_forward_flight;
static int32_t v_control_pitch;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_hybrid_guidance(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i *pos = stateGetPositionNed_i();
  struct NedCoor_i *speed = stateGetSpeedNed_i();
  pprz_msg_send_HYBRID_GUIDANCE(trans, dev, AC_ID,
                                &(pos->x), &(pos->y),
                                &(speed->x), &(speed->y),
                                &wind_estimate.x, &wind_estimate.y,
                                &guidance_h_pos_err.x,
                                &guidance_h_pos_err.y,
                                &guidance_hybrid_airspeed_sp.x,
                                &guidance_hybrid_airspeed_sp.y,
                                &guidance_hybrid_norm_ref_airspeed,
                                &heading_diff_disp,
                                &guidance_hybrid_ypr_sp.phi,
                                &guidance_hybrid_ypr_sp.theta,
                                &guidance_hybrid_ypr_sp.psi);
}

#endif

void guidance_hybrid_init(void)
{

  INT_EULERS_ZERO(guidance_hybrid_ypr_sp);
  INT_VECT2_ZERO(guidance_hybrid_airspeed_sp);
  INT_VECT2_ZERO(guidance_hybrid_airspeed_ref);

  high_res_psi = 0;
  guidance_hovering = true;
  horizontal_speed_gain = 8;
  guidance_hybrid_norm_ref_airspeed = 0;
  max_turn_bank = 40.0;
  turn_bank_gain = 0.8;
  wind_gain = 64;
  force_forward_flight = 0;
  INT_VECT2_ZERO(wind_estimate);
  INT_VECT2_ZERO(guidance_hybrid_ref_airspeed);
  INT_VECT2_ZERO(wind_estimate_high_res);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_HYBRID_GUIDANCE, send_hybrid_guidance);
#endif

}

#define INT32_ANGLE_HIGH_RES_NORMALIZE(_a) {             \
    while ((_a) > (INT32_ANGLE_PI << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC)))  (_a) -= (INT32_ANGLE_2_PI << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC));    \
    while ((_a) < -(INT32_ANGLE_PI << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC))) (_a) += (INT32_ANGLE_2_PI << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC));    \
  }

void guidance_hybrid_run(void)
{
  guidance_hybrid_determine_wind_estimate();
  guidance_hybrid_position_to_airspeed();
  guidance_hybrid_airspeed_to_attitude(&guidance_hybrid_ypr_sp);
  guidance_hybrid_set_cmd_i(&guidance_hybrid_ypr_sp);
}

void guidance_hybrid_reset_heading(struct Int32Eulers *sp_cmd)
{
  guidance_hybrid_ypr_sp.psi = sp_cmd->psi;
  high_res_psi = sp_cmd->psi << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC);
  stabilization_attitude_set_rpy_setpoint_i(sp_cmd);
}

/// Convert a required airspeed to a certain attitude for the Quadshot
void guidance_hybrid_airspeed_to_attitude(struct Int32Eulers *ypr_sp)
{

  //notes:
  //in forward flight, it is preferred to first get to min(airspeed_sp, airspeed_ref) and then change heading and then get to airspeed_sp
  //in hover, just a gradual change is needed, or maybe not even needed
  //changes between flight regimes should be handled

  //determine the heading of the airspeed_sp vector
  int32_t omega = 0;
  float airspeed_sp_heading = atan2f((float) POS_FLOAT_OF_BFP(guidance_hybrid_airspeed_sp.y),
                                     (float) POS_FLOAT_OF_BFP(guidance_hybrid_airspeed_sp.x));
  //only for debugging
  airspeed_sp_heading_disp = (int32_t)(DegOfRad(airspeed_sp_heading));

  //The difference of the current heading with the required heading.
  float heading_diff = airspeed_sp_heading - ANGLE_FLOAT_OF_BFP(ypr_sp->psi);
  FLOAT_ANGLE_NORMALIZE(heading_diff);

  //only for debugging
  heading_diff_disp = (int32_t)(heading_diff / 3.14 * 180.0);

  //calculate the norm of the airspeed setpoint
  int32_t norm_sp_airspeed;
  norm_sp_airspeed = int32_vect2_norm(&guidance_hybrid_airspeed_sp);

  //reference goes with a steady pace towards the setpoint airspeed
  //hold ref norm below 4 m/s until heading is aligned
  if (!((norm_sp_airspeed > (4 << 8)) && (guidance_hybrid_norm_ref_airspeed < (4 << 8))
        && (guidance_hybrid_norm_ref_airspeed > ((4 << 8) - 10)) && (fabs(heading_diff) > (5.0 / 180.0 * 3.14)))) {
    guidance_hybrid_norm_ref_airspeed = guidance_hybrid_norm_ref_airspeed + ((int32_t)(norm_sp_airspeed >
                                        guidance_hybrid_norm_ref_airspeed) * 2 - 1) * 3 / 2;
  }

  norm_sp_airspeed_disp = norm_sp_airspeed;

  int32_t psi = ypr_sp->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);

  guidance_hybrid_ref_airspeed.x = (guidance_hybrid_norm_ref_airspeed * c_psi) >> INT32_TRIG_FRAC;
  guidance_hybrid_ref_airspeed.y = (guidance_hybrid_norm_ref_airspeed * s_psi) >> INT32_TRIG_FRAC;

  if (guidance_hybrid_norm_ref_airspeed < (4 << 8)) {
    /// if required speed is lower than 4 m/s act like a rotorcraft
    // translate speed_sp into bank angle and heading

    // change heading to direction of airspeed, faster if the airspeed is higher
    if (heading_diff > 0.0) {
      omega = (norm_sp_airspeed << (INT32_ANGLE_FRAC - INT32_POS_FRAC)) / 6;
    } else if (heading_diff < 0.0) {
      omega = (norm_sp_airspeed << (INT32_ANGLE_FRAC - INT32_POS_FRAC)) / -6;
    }

    if (omega > ANGLE_BFP_OF_REAL(0.8)) { omega = ANGLE_BFP_OF_REAL(0.8); }
    if (omega < ANGLE_BFP_OF_REAL(-0.8)) { omega = ANGLE_BFP_OF_REAL(-0.8); }

    // 2) calculate roll/pitch commands
    struct Int32Vect2 hover_sp;
    //if the setpoint is beyond 4m/s but the ref is not, the norm of the hover sp will stay at 4m/s
    if (norm_sp_airspeed > (4 << 8)) {
      hover_sp.x = (guidance_hybrid_airspeed_sp.x << 8) / norm_sp_airspeed * 4;
      hover_sp.y = (guidance_hybrid_airspeed_sp.y << 8) / norm_sp_airspeed * 4;
    } else {
      hover_sp.x = guidance_hybrid_airspeed_sp.x;
      hover_sp.y = guidance_hybrid_airspeed_sp.y;
    }

    // gain of 10 means that for 4 m/s an angle of 40 degrees is needed
    ypr_sp->theta = (((- (c_psi * hover_sp.x + s_psi * hover_sp.y)) >> INT32_TRIG_FRAC) * 10 * INT32_ANGLE_PI / 180) >> 8;
    ypr_sp->phi = ((((- s_psi * hover_sp.x + c_psi * hover_sp.y)) >> INT32_TRIG_FRAC) * 10 * INT32_ANGLE_PI / 180) >>  8;
  } else {
    /// if required speed is higher than 4 m/s act like a fixedwing
    // translate speed_sp into theta + thrust
    // coordinated turns to change heading

    // calculate required pitch angle from airspeed_sp magnitude
    if (guidance_hybrid_norm_ref_airspeed > (15 << 8)) {
      ypr_sp->theta = -ANGLE_BFP_OF_REAL(RadOfDeg(78.0));
    } else if (guidance_hybrid_norm_ref_airspeed > (8 << 8)) {
      ypr_sp->theta = -(((guidance_hybrid_norm_ref_airspeed - (8 << 8)) * 2 * INT32_ANGLE_PI / 180) >> 8) - ANGLE_BFP_OF_REAL(
                        RadOfDeg(68.0));
    } else {
      ypr_sp->theta = -(((guidance_hybrid_norm_ref_airspeed - (4 << 8)) * 7 * INT32_ANGLE_PI / 180) >> 8) - ANGLE_BFP_OF_REAL(
                        RadOfDeg(40.0));
    }

    // if the sp_airspeed is within hovering range, don't start a coordinated turn
    if (norm_sp_airspeed < (4 << 8)) {
      omega = 0;
      ypr_sp->phi = 0;
    } else { // coordinated turn
      ypr_sp->phi = ANGLE_BFP_OF_REAL(heading_diff * turn_bank_gain);
      if (ypr_sp->phi > ANGLE_BFP_OF_REAL(max_turn_bank / 180.0 * M_PI)) { ypr_sp->phi = ANGLE_BFP_OF_REAL(max_turn_bank / 180.0 * M_PI); }
      if (ypr_sp->phi < ANGLE_BFP_OF_REAL(-max_turn_bank / 180.0 * M_PI)) { ypr_sp->phi = ANGLE_BFP_OF_REAL(-max_turn_bank / 180.0 * M_PI); }

      //feedforward estimate angular rotation omega = g*tan(phi)/v
      omega = ANGLE_BFP_OF_REAL(9.81 / POS_FLOAT_OF_BFP(guidance_hybrid_norm_ref_airspeed) * tanf(ANGLE_FLOAT_OF_BFP(
                                  ypr_sp->phi)));

      if (omega > ANGLE_BFP_OF_REAL(0.7)) { omega = ANGLE_BFP_OF_REAL(0.7); }
      if (omega < ANGLE_BFP_OF_REAL(-0.7)) { omega = ANGLE_BFP_OF_REAL(-0.7); }
    }
  }

  //only for debugging purposes
  omega_disp = omega;

  //go to higher resolution because else the increment is too small to be added
  high_res_psi += (omega << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC)) / 512;

  INT32_ANGLE_HIGH_RES_NORMALIZE(high_res_psi);

  // go back to angle_frac
  ypr_sp->psi = high_res_psi >> (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC);
  ypr_sp->theta = ypr_sp->theta + v_control_pitch;
}

void guidance_hybrid_position_to_airspeed(void)
{
  /* compute position error    */
  VECT2_DIFF(guidance_h_pos_err, guidance_h.sp.pos, *stateGetPositionNed_i());

  // Compute ground speed setpoint
  struct Int32Vect2 guidance_hybrid_groundspeed_sp;
  VECT2_SDIV(guidance_hybrid_groundspeed_sp, guidance_h_pos_err, horizontal_speed_gain);

  int32_t norm_groundspeed_sp;
  norm_groundspeed_sp = int32_vect2_norm(&guidance_hybrid_groundspeed_sp);

  //create setpoint groundspeed with a norm of 15 m/s
  if (force_forward_flight) {
    //scale the groundspeed_sp to 15 m/s if large enough to begin with
    if (norm_groundspeed_sp > 1 << 8) {
      guidance_hybrid_groundspeed_sp.x = guidance_hybrid_groundspeed_sp.x * (max_airspeed << 8) / norm_groundspeed_sp;
      guidance_hybrid_groundspeed_sp.y = guidance_hybrid_groundspeed_sp.y * (max_airspeed << 8) / norm_groundspeed_sp;
    } else { //groundspeed_sp is very small, so continue with the current heading
      int32_t psi = guidance_hybrid_ypr_sp.psi;
      int32_t s_psi, c_psi;
      PPRZ_ITRIG_SIN(s_psi, psi);
      PPRZ_ITRIG_COS(c_psi, psi);
      guidance_hybrid_groundspeed_sp.x = (15 * c_psi) >> (INT32_TRIG_FRAC - 8);
      guidance_hybrid_groundspeed_sp.y = (15 * s_psi) >> (INT32_TRIG_FRAC - 8);
    }
  }

  struct Int32Vect2 airspeed_sp;
  INT_VECT2_ZERO(airspeed_sp);
  VECT2_ADD(airspeed_sp, guidance_hybrid_groundspeed_sp);
  VECT2_ADD(airspeed_sp, wind_estimate);

  int32_t norm_airspeed_sp;
  norm_airspeed_sp = int32_vect2_norm(&airspeed_sp);

  //Check if the airspeed_sp is larger than the max airspeed. If so, give the wind cancellatioin priority.
  if (norm_airspeed_sp > (max_airspeed << 8) && norm_groundspeed_sp > 0) {
    int32_t av = INT_MULT_RSHIFT(guidance_hybrid_groundspeed_sp.x, guidance_hybrid_groundspeed_sp.x,
                                 8) + INT_MULT_RSHIFT(guidance_hybrid_groundspeed_sp.y, guidance_hybrid_groundspeed_sp.y, 8);
    int32_t bv = 2 * (INT_MULT_RSHIFT(wind_estimate.x, guidance_hybrid_groundspeed_sp.x,
                                      8) + INT_MULT_RSHIFT(wind_estimate.y, guidance_hybrid_groundspeed_sp.y, 8));
    int32_t cv = INT_MULT_RSHIFT(wind_estimate.x, wind_estimate.x, 8) + INT_MULT_RSHIFT(wind_estimate.y, wind_estimate.y,
                 8) - (max_airspeed << 8) * max_airspeed;

    float dv = POS_FLOAT_OF_BFP(bv) * POS_FLOAT_OF_BFP(bv) - 4.0 * POS_FLOAT_OF_BFP(av) * POS_FLOAT_OF_BFP(cv);
    float d_sqrt_f = sqrtf(dv);
    int32_t d_sqrt = POS_BFP_OF_REAL(d_sqrt_f);

    int32_t result = ((-bv + d_sqrt) << 8) / (2 * av);

    guidance_hybrid_airspeed_sp.x = wind_estimate.x + INT_MULT_RSHIFT(guidance_hybrid_groundspeed_sp.x, result, 8);
    guidance_hybrid_airspeed_sp.y = wind_estimate.y + INT_MULT_RSHIFT(guidance_hybrid_groundspeed_sp.y, result, 8);
  } else {
    // Add the wind to get the airspeed setpoint
    guidance_hybrid_airspeed_sp = guidance_hybrid_groundspeed_sp;
    VECT2_ADD(guidance_hybrid_airspeed_sp, wind_estimate);
  }

  // limit the airspeed setpoint to 15 m/s, because else saturation+windup will occur
  norm_airspeed_sp = int32_vect2_norm(&guidance_hybrid_airspeed_sp);
  // add check for non-zero airspeed (in case the max_airspeed is ever set to zero
  if ((norm_airspeed_sp > (max_airspeed << 8)) && (norm_airspeed_sp != 0)) {
    guidance_hybrid_airspeed_sp.x = guidance_hybrid_airspeed_sp.x * (max_airspeed << 8) / norm_airspeed_sp;
    guidance_hybrid_airspeed_sp.y = guidance_hybrid_airspeed_sp.y * (max_airspeed << 8) / norm_airspeed_sp;
  }
}

void guidance_hybrid_determine_wind_estimate(void)
{

  /* compute speed error    */
  struct Int32Vect2 wind_estimate_measured;
  struct Int32Vect2 measured_ground_speed;
  INT32_VECT2_RSHIFT(measured_ground_speed, *stateGetSpeedNed_i(), 11);
  VECT2_DIFF(wind_estimate_measured, guidance_hybrid_ref_airspeed, measured_ground_speed);

  //Low pass wind_estimate, because we know the wind usually only changes slowly
  //But not too slow, because the wind_estimate is also an adaptive element for the airspeed model inaccuracies
  wind_estimate_high_res.x += (((wind_estimate_measured.x - wind_estimate.x) > 0) * 2 - 1) * wind_gain;
  wind_estimate_high_res.y += (((wind_estimate_measured.y - wind_estimate.y) > 0) * 2 - 1) * wind_gain;

  wind_estimate.x = ((wind_estimate_high_res.x) >> 8);
  wind_estimate.y = ((wind_estimate_high_res.y) >> 8);
}

void guidance_hybrid_set_cmd_i(struct Int32Eulers *sp_cmd)
{
  /// @todo calc sp_quat in fixed-point

  /* orientation vector describing simultaneous rotation of roll/pitch */
  struct FloatVect3 ov;
  ov.x = ANGLE_FLOAT_OF_BFP(sp_cmd->phi);
  ov.y = ANGLE_FLOAT_OF_BFP(sp_cmd->theta);
  ov.z = 0.0;
  /* quaternion from that orientation vector */
  struct FloatQuat q_rp;
  float_quat_of_orientation_vect(&q_rp, &ov);
  struct Int32Quat q_rp_i;
  QUAT_BFP_OF_REAL(q_rp_i, q_rp);

  //   get the vertical vector to rotate the roll pitch setpoint around
  struct Int32Vect3 zaxis = {0, 0, 1};

  /* get current heading setpoint */
  struct Int32Quat q_yaw_sp;
  int32_quat_of_axis_angle(&q_yaw_sp, &zaxis, sp_cmd->psi);

  //   first apply the roll/pitch setpoint and then the yaw
  int32_quat_comp(&stab_att_sp_quat, &q_yaw_sp, &q_rp_i);

  int32_eulers_of_quat(&stab_att_sp_euler, &stab_att_sp_quat);
}

void guidance_hybrid_vertical(void)
{
  if (guidance_hybrid_norm_ref_airspeed < (4 << 8)) {
    //if airspeed ref < 4 only thrust
    stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
    v_control_pitch = 0;
    guidance_v_kp = GUIDANCE_V_HOVER_KP;
    guidance_v_kd = GUIDANCE_V_HOVER_KD;
    guidance_v_ki = GUIDANCE_V_HOVER_KI;
  } else if (guidance_hybrid_norm_ref_airspeed > (8 << 8)) { //if airspeed ref > 8 only pitch,
    //at 15 m/s the thrust has to be 33%
    stabilization_cmd[COMMAND_THRUST] = MAX_PPRZ / 5 + (((guidance_hybrid_norm_ref_airspeed - (8 << 8)) / 7 *
                                        (MAX_PPRZ / 3 - MAX_PPRZ / 5)) >> 8) + (guidance_v_delta_t - MAX_PPRZ / 2) / 10;
    //stabilization_cmd[COMMAND_THRUST] = MAX_PPRZ/5;
    // stabilization_cmd[COMMAND_THRUST] = ((guidance_hybrid_norm_ref_airspeed - (8<<8)) / 7 * (MAX_PPRZ/3 - MAX_PPRZ/5))>>8 + 9600/5;
    //Control altitude with pitch, now only proportional control
    float alt_control_pitch = (guidance_v_delta_t - MAX_PPRZ * guidance_v_nominal_throttle) * alt_pitch_gain;
    v_control_pitch = ANGLE_BFP_OF_REAL(alt_control_pitch / (MAX_PPRZ * guidance_v_nominal_throttle));
    guidance_v_kp = GUIDANCE_V_HOVER_KP / 2;
    guidance_v_kd = GUIDANCE_V_HOVER_KD / 2;
    guidance_v_ki = GUIDANCE_V_HOVER_KI / 2;
  } else { //if airspeed ref > 4 && < 8 both
    int32_t airspeed_transition = (guidance_hybrid_norm_ref_airspeed - (4 << 8)) / 4; //divide by 4 to scale it to 0-1 (<<8)
    stabilization_cmd[COMMAND_THRUST] = ((MAX_PPRZ / 5 + (guidance_v_delta_t - MAX_PPRZ / 2) / 10) * airspeed_transition +
                                         guidance_v_delta_t * ((1 << 8) - airspeed_transition)) >> 8;
    float alt_control_pitch = (guidance_v_delta_t - MAX_PPRZ * guidance_v_nominal_throttle) * alt_pitch_gain;
    v_control_pitch = INT_MULT_RSHIFT((int32_t) ANGLE_BFP_OF_REAL(alt_control_pitch / (MAX_PPRZ *
                                      guidance_v_nominal_throttle)), airspeed_transition, 8);
    guidance_v_kp = GUIDANCE_V_HOVER_KP;
    guidance_v_kd = GUIDANCE_V_HOVER_KD;
    guidance_v_ki = GUIDANCE_V_HOVER_KI;
  }
}
