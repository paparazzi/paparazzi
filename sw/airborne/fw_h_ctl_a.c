/*
 * Paparazzi $Id: fw_h_ctl.c 3603 2009-07-01 20:06:53Z hecto $
 *  
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin, Michel Gorraz
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
 *
 */

/** 
 *
 * fixed wing horizontal adaptive control
 *
 */

#include "std.h"
#include "led.h"
#include "fw_h_ctl.h"
#include "fw_h_ctl_a.h"
#include "estimator.h"
#include "nav.h"
#include "airframe.h"
#include "fw_v_ctl.h"
#include "autopilot.h"


/* outer loop parameters */
float h_ctl_course_setpoint; /* rad, CW/north */
float h_ctl_course_pre_bank;
float h_ctl_course_pre_bank_correction;
float h_ctl_course_pgain;
float h_ctl_course_dgain;
float h_ctl_roll_max_setpoint;

/* roll and pitch disabling */
bool_t h_ctl_disabled;

/* AUTO1 rate mode */
bool_t h_ctl_auto1_rate;


float h_ctl_ref_roll_angle;
float h_ctl_ref_roll_rate;
float h_ctl_ref_roll_accel;
float h_ctl_ref_pitch_angle;
float h_ctl_ref_pitch_rate;
float h_ctl_ref_pitch_accel;
#define H_CTL_REF_W 3.
#define H_CTL_REF_XI 1.

/* inner roll loop parameters */
float  h_ctl_roll_setpoint;
float h_ctl_roll_attitude_gain;
float h_ctl_roll_rate_gain;
float  h_ctl_roll_igain;
float  h_ctl_pitch_igain;
float  h_ctl_roll_sum_err;
float  h_ctl_pitch_sum_err;
float h_ctl_roll_Kff;
pprz_t h_ctl_aileron_setpoint;
float  h_ctl_roll_slew;

float  h_ctl_roll_pgain;

/* inner pitch loop parameters */
float  h_ctl_pitch_setpoint;
float  h_ctl_pitch_loop_setpoint;
float  h_ctl_pitch_pgain;
float  h_ctl_pitch_dgain;
pprz_t h_ctl_elevator_setpoint;

/* inner loop pre-command */
float h_ctl_aileron_of_throttle;
float h_ctl_elevator_of_roll;


inline static void h_ctl_roll_loop( void );
inline static void h_ctl_pitch_loop( void );

#ifndef H_CTL_COURSE_PRE_BANK_CORRECTION
#define H_CTL_COURSE_PRE_BANK_CORRECTION 1.
#endif

#ifndef H_CTL_COURSE_DGAIN
#define H_CTL_COURSE_DGAIN 0.
#endif

#ifndef H_CTL_ROLL_RATE_GAIN
#define H_CTL_ROLL_RATE_GAIN 0.
#endif

#ifdef AGR_CLIMB
static float nav_ratio;
#endif

void h_ctl_init( void ) {
  h_ctl_course_setpoint = 0.;
  h_ctl_course_pre_bank = 0.;
  h_ctl_course_pre_bank_correction = H_CTL_COURSE_PRE_BANK_CORRECTION;
  h_ctl_course_pgain = H_CTL_COURSE_PGAIN;
  h_ctl_course_dgain = H_CTL_COURSE_DGAIN;
  h_ctl_roll_max_setpoint = H_CTL_ROLL_MAX_SETPOINT;

  h_ctl_disabled = FALSE;

  h_ctl_roll_setpoint = 0.;
  h_ctl_aileron_setpoint = 0;
#ifdef H_CTL_AILERON_OF_THROTTLE
  h_ctl_aileron_of_throttle = H_CTL_AILERON_OF_THROTTLE;
#endif

  h_ctl_pitch_setpoint = 0.;
  h_ctl_pitch_loop_setpoint = 0.;
  h_ctl_pitch_pgain = H_CTL_PITCH_PGAIN;
  h_ctl_pitch_dgain = H_CTL_PITCH_DGAIN;
  h_ctl_elevator_setpoint = 0;
  h_ctl_elevator_of_roll = H_CTL_ELEVATOR_OF_ROLL;

  h_ctl_roll_attitude_gain = H_CTL_ROLL_ATTITUDE_GAIN;
  h_ctl_roll_rate_gain = H_CTL_ROLL_RATE_GAIN;

  h_ctl_roll_igain = H_CTL_ROLL_IGAIN;
  h_ctl_roll_sum_err = 0;
  h_ctl_roll_Kff = H_CTL_ROLL_KFF;

#ifdef AGR_CLIMB
nav_ratio=0;
#endif
}

/** 
 * \brief 
 *
 */
void h_ctl_course_loop ( void ) {
  static float last_err;

  // Ground path error
  float err = estimator_hspeed_dir - h_ctl_course_setpoint;
  NormRadAngle(err);

  float d_err = err - last_err;
  last_err = err;

  NormRadAngle(d_err);

  float speed_depend_nav = estimator_hspeed_mod/NOMINAL_AIRSPEED; 
  Bound(speed_depend_nav, 0.66, 1.5);

  float cmd = h_ctl_course_pgain * speed_depend_nav * (err + d_err * h_ctl_course_dgain);



#if defined AGR_CLIMB
  /** limit navigation during extreme altitude changes */
  if (AGR_BLEND_START > AGR_BLEND_END && AGR_BLEND_END > 0) { /* prevent divide by zero, reversed or negative values */
    if (v_ctl_auto_throttle_submode == V_CTL_AUTO_THROTTLE_AGRESSIVE || V_CTL_AUTO_THROTTLE_BLENDED) {
      BoundAbs(cmd, h_ctl_roll_max_setpoint); /* bound cmd before NAV_RATIO and again after */
      if (v_ctl_altitude_error < 0) {
	nav_ratio = AGR_CLIMB_NAV_RATIO + (1 - AGR_CLIMB_NAV_RATIO) * (1 - (fabs(v_ctl_altitude_error) - AGR_BLEND_END) / (AGR_BLEND_START - AGR_BLEND_END));
	Bound (nav_ratio, AGR_CLIMB_NAV_RATIO, 1);
      } else {
	nav_ratio = AGR_DESCENT_NAV_RATIO + (1 - AGR_DESCENT_NAV_RATIO) * (1 - (fabs(v_ctl_altitude_error) - AGR_BLEND_END) / (AGR_BLEND_START - AGR_BLEND_END));
	Bound (nav_ratio, AGR_DESCENT_NAV_RATIO, 1);
      }
      cmd *= nav_ratio;
    }
  }
#endif

  float roll_setpoint = cmd + h_ctl_course_pre_bank_correction * h_ctl_course_pre_bank;

  h_ctl_roll_setpoint = roll_setpoint;

  BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);
}

static float airspeed_ratio2;

void h_ctl_attitude_loop ( void ) {
  if (!h_ctl_disabled) {
    float throttle_diff =  v_ctl_throttle_setpoint / (float)MAX_PPRZ - v_ctl_auto_throttle_nominal_cruise_throttle;
    float airspeed = NOMINAL_AIRSPEED; /* Estimated from the throttle */
    if (throttle_diff > 0)
      airspeed += throttle_diff / (V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE - v_ctl_auto_throttle_nominal_cruise_throttle) * (MAXIMUM_AIRSPEED - NOMINAL_AIRSPEED);
    else
      airspeed += throttle_diff / (v_ctl_auto_throttle_nominal_cruise_throttle - V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE) * (NOMINAL_AIRSPEED - MINIMUM_AIRSPEED);

    float airspeed_ratio = airspeed / NOMINAL_AIRSPEED;
    Bound(airspeed_ratio, 0.5, 2.);
    airspeed_ratio2 = airspeed_ratio*airspeed_ratio;
    h_ctl_roll_loop();
    h_ctl_pitch_loop();
  }
}

#define H_CTL_REF_DT (1./60.)

inline static void h_ctl_roll_loop( void ) {

  if (pprz_mode == PPRZ_MODE_MANUAL)
    h_ctl_roll_sum_err = 0;

  h_ctl_ref_roll_accel = H_CTL_REF_W*H_CTL_REF_W * (h_ctl_roll_setpoint - h_ctl_ref_roll_angle) - 2*H_CTL_REF_XI*H_CTL_REF_W * h_ctl_ref_roll_rate;
  h_ctl_ref_roll_rate += h_ctl_ref_roll_accel * H_CTL_REF_DT;
  h_ctl_ref_roll_angle += h_ctl_ref_roll_rate * H_CTL_REF_DT;

  float err = estimator_phi - h_ctl_ref_roll_angle;
#ifdef SITL
  static float last_err = 0;
  estimator_p = (err - last_err)/(1/60.);
  last_err = err;
#endif
  float derr = estimator_p - h_ctl_ref_roll_rate;
  h_ctl_roll_sum_err += err * H_CTL_REF_DT;
  BoundAbs(h_ctl_roll_sum_err, H_CTL_ROLL_SUM_ERR_MAX);
  float cmd = h_ctl_roll_Kff * h_ctl_ref_roll_accel
    - h_ctl_roll_attitude_gain * err
    - h_ctl_roll_rate_gain * derr
    - h_ctl_roll_igain * h_ctl_roll_sum_err
    + v_ctl_throttle_setpoint * h_ctl_aileron_of_throttle;

  //  cmd /= airspeed_ratio2;

  h_ctl_aileron_setpoint = TRIM_PPRZ(cmd); 
}



#ifdef LOITER_TRIM

float v_ctl_auto_throttle_loiter_trim = V_CTL_AUTO_THROTTLE_LOITER_TRIM;
float v_ctl_auto_throttle_dash_trim = V_CTL_AUTO_THROTTLE_DASH_TRIM;

inline static float loiter(void) {
  float elevator_trim;

  float throttle_dif = v_ctl_auto_throttle_cruise_throttle - v_ctl_auto_throttle_nominal_cruise_throttle;
  if (throttle_dif > 0) {
    float max_dif = Max(V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE - v_ctl_auto_throttle_nominal_cruise_throttle, 0.1);
    elevator_trim = throttle_dif / max_dif * v_ctl_auto_throttle_dash_trim;
  } else {
    float max_dif = Max(v_ctl_auto_throttle_nominal_cruise_throttle - V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE, 0.1);
    elevator_trim = - throttle_dif / max_dif * v_ctl_auto_throttle_loiter_trim;
  }

  return elevator_trim / h_ctl_pitch_pgain; /* Normalisation to keep historical values */
}
#endif


inline static void h_ctl_pitch_loop( void ) {
  h_ctl_ref_pitch_accel = H_CTL_REF_W*H_CTL_REF_W * (h_ctl_pitch_loop_setpoint - h_ctl_ref_pitch_angle) - 2*H_CTL_REF_XI*H_CTL_REF_W * h_ctl_ref_pitch_rate;
  h_ctl_ref_pitch_rate += h_ctl_ref_pitch_accel * H_CTL_REF_DT;
  h_ctl_ref_pitch_angle += h_ctl_ref_pitch_rate * H_CTL_REF_DT;

  static float last_err;
  /* sanity check */
  if (h_ctl_elevator_of_roll <0.)
    h_ctl_elevator_of_roll = 0.;

  h_ctl_pitch_loop_setpoint =
    h_ctl_pitch_setpoint 
    - h_ctl_elevator_of_roll / h_ctl_pitch_pgain * fabs(estimator_phi);

#ifdef LOITER_TRIM
  h_ctl_pitch_loop_setpoint -= loiter();
#endif

  float err = estimator_theta - h_ctl_ref_pitch_angle;

  if (pprz_mode == PPRZ_MODE_MANUAL)
    h_ctl_pitch_sum_err = 0;
  h_ctl_pitch_sum_err += err * H_CTL_REF_DT;
  BoundAbs(h_ctl_pitch_sum_err, H_CTL_ROLL_SUM_ERR_MAX);


  float d_err = (err - last_err)/H_CTL_REF_DT - h_ctl_ref_pitch_rate;
  last_err = err;
  float cmd = h_ctl_pitch_pgain * err + h_ctl_pitch_dgain * d_err + h_ctl_pitch_igain * h_ctl_pitch_sum_err;

  cmd /= airspeed_ratio2;

  h_ctl_elevator_setpoint = TRIM_PPRZ(cmd);
}
