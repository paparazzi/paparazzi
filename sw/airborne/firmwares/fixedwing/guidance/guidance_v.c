/*
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
 */

/**
 *  @file firmwares/fixedwing/guidance/guidance_v.c
 *  Vertical control for fixed wing vehicles.
 *
 */

#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "state.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/airframe.h"
#include "autopilot.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h" //> allow for roll control during landing final flare

/* mode */
uint8_t v_ctl_mode;

/* outer loop */
float v_ctl_altitude_setpoint;
float v_ctl_altitude_pre_climb;
float v_ctl_altitude_pgain;
float v_ctl_altitude_error;
float v_ctl_altitude_pre_climb_correction;
float v_ctl_altitude_max_climb;

/* inner loop */
float v_ctl_climb_setpoint;
uint8_t v_ctl_climb_mode;
uint8_t v_ctl_auto_throttle_submode;

#ifndef V_CTL_AUTO_THROTTLE_DGAIN
#define V_CTL_AUTO_THROTTLE_DGAIN 0.
#endif

/* "auto throttle" inner loop parameters */
float v_ctl_auto_throttle_cruise_throttle;
float v_ctl_auto_throttle_nominal_cruise_throttle;
float v_ctl_auto_throttle_min_cruise_throttle;
float v_ctl_auto_throttle_max_cruise_throttle;
float v_ctl_auto_throttle_climb_throttle_increment;
float v_ctl_auto_throttle_pgain;
float v_ctl_auto_throttle_igain;
float v_ctl_auto_throttle_dgain;
float v_ctl_auto_throttle_sum_err;
#define V_CTL_AUTO_THROTTLE_MAX_SUM_ERR 150
float v_ctl_auto_throttle_pitch_of_vz_pgain;
float v_ctl_auto_throttle_pitch_of_vz_dgain;

#ifndef V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_DGAIN
#define V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_DGAIN 0.
#endif

/* agressive tuning */
#ifdef TUNE_AGRESSIVE_CLIMB
float agr_climb_throttle;
float agr_climb_pitch;
float agr_climb_nav_ratio;
float agr_descent_throttle;
float agr_descent_pitch;
float agr_descent_nav_ratio;
#endif

/* "auto pitch" inner loop parameters */
float v_ctl_auto_pitch_pgain;
float v_ctl_auto_pitch_igain;
float v_ctl_auto_pitch_sum_err;
#define V_CTL_AUTO_PITCH_MAX_SUM_ERR 100

pprz_t v_ctl_throttle_setpoint;
pprz_t v_ctl_throttle_slewed;
float v_ctl_pitch_setpoint;
#ifndef V_CTL_PITCH_TRIM
#define V_CTL_PITCH_TRIM 0.
#endif
float v_ctl_pitch_trim;

inline static void v_ctl_climb_auto_throttle_loop(void);
#ifdef V_CTL_AUTO_PITCH_PGAIN
inline static void v_ctl_climb_auto_pitch_loop(void);
#endif

#if USE_AIRSPEED
float v_ctl_auto_airspeed_setpoint;
float v_ctl_auto_airspeed_controlled;
float v_ctl_auto_airspeed_pgain;
float v_ctl_auto_airspeed_igain;
float v_ctl_auto_airspeed_sum_err;
float v_ctl_auto_groundspeed_setpoint;
float v_ctl_auto_groundspeed_pgain;
float v_ctl_auto_groundspeed_igain;
float v_ctl_auto_groundspeed_sum_err;
#define V_CTL_AUTO_AIRSPEED_MAX_SUM_ERR 200
#define V_CTL_AUTO_GROUNDSPEED_MAX_SUM_ERR 100
#define V_CTL_AUTO_CLIMB_LIMIT 0.5/4.0 // m/s/s
#define V_CTL_AUTO_AGR_CLIMB_GAIN 2.0 // altitude gain multiplier while in aggressive climb mode
#endif

#ifndef V_CTL_ALTITUDE_PRE_CLIMB_CORRECTION
#define V_CTL_ALTITUDE_PRE_CLIMB_CORRECTION 1.0f
#endif

#if CTRL_VERTICAL_LANDING
#ifndef V_CTL_LANDING_THROTTLE_PGAIN
#error "V_CTL_LANDING_THROTTLE_PGAIN undefined, please define it in your airfame config file"
#endif
#ifndef V_CTL_LANDING_THROTTLE_IGAIN
#error "V_CTL_LANDING_THROTTLE_IGAIN undefined, please define it in your airfame config file"
#endif
#ifndef V_CTL_LANDING_THROTTLE_MAX
INFO("V_CTL_LANDING_THROTTLE_MAX undefined, using V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE instead")
#define V_CTL_LANDING_THROTTLE_MAX V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE
#endif
#ifndef V_CTL_LANDING_DESIRED_SPEED
#error "V_CTL_LANDING_DESIRED_SPEED undefined, please define it in your airfame config file"
#endif


#ifndef V_CTL_LANDING_PITCH_PGAIN
INFO("V_CTL_LANDING_PITCH_PGAIN undefined, using V_CTL_AUTO_PITCH_PGAIN instead")
#define V_CTL_LANDING_PITCH_PGAIN V_CTL_AUTO_PITCH_PGAIN
#endif
#ifndef V_CTL_LANDING_PITCH_IGAIN
INFO("V_CTL_LANDING_PITCH_IGAIN undefined, using V_CTL_AUTO_PITCH_IGAIN instead")
#define V_CTL_LANDING_PITCH_IGAIN V_CTL_AUTO_PITCH_IGAIN
#endif
#ifndef V_CTL_LANDING_PITCH_LIMITS
INFO("V_CTL_LANDING_PITCH_LIMITS undefined, using V_CTL_AUTO_PITCH_MAX_PITCH instead")
#define V_CTL_LANDING_PITCH_LIMITS V_CTL_AUTO_PITCH_MAX_PITCH
#endif
#ifndef V_CTL_LANDING_PITCH_FLARE
#error "V_CTL_LANDING_PITCH_FLARE undefined, please define it in your airfame config file"
#endif
#ifndef V_CTL_LANDING_ALT_THROTTLE_KILL
#error "V_CTL_LANDING_ALT_THROTTLE_KILL undefined, please define it in your airfame config file"
#endif
#ifndef V_CTL_LANDING_ALT_FLARE
#error "V_CTL_LANDING_ALT_FLARE undefined, please define it in your airfame config file"
#endif

float v_ctl_landing_throttle_pgain;
float v_ctl_landing_throttle_igain;
float v_ctl_landing_throttle_max;
float v_ctl_landing_desired_speed;
float v_ctl_landing_pitch_pgain;
float v_ctl_landing_pitch_igain;
float v_ctl_landing_pitch_limits;
float v_ctl_landing_pitch_flare;
float v_ctl_landing_alt_throttle_kill; //> must be greater than alt_flare
float v_ctl_landing_alt_flare;
#endif /* CTRL_VERTICAL_LANDING */

void v_ctl_init(void)
{
  /* mode */
  v_ctl_mode = V_CTL_MODE_MANUAL;

#if CTRL_VERTICAL_LANDING
  /* improved landing routine */
  v_ctl_landing_throttle_pgain = V_CTL_LANDING_THROTTLE_PGAIN;
  v_ctl_landing_throttle_igain = V_CTL_LANDING_THROTTLE_IGAIN;
  v_ctl_landing_throttle_max = V_CTL_LANDING_THROTTLE_MAX;
  v_ctl_landing_desired_speed = V_CTL_LANDING_DESIRED_SPEED;
  v_ctl_landing_pitch_pgain = V_CTL_LANDING_PITCH_PGAIN;
  v_ctl_landing_pitch_igain = V_CTL_LANDING_PITCH_IGAIN;
  v_ctl_landing_pitch_limits = V_CTL_LANDING_PITCH_LIMITS;
  v_ctl_landing_pitch_flare = V_CTL_LANDING_PITCH_FLARE;
  v_ctl_landing_alt_throttle_kill = V_CTL_LANDING_ALT_THROTTLE_KILL;
  v_ctl_landing_alt_flare = V_CTL_LANDING_ALT_FLARE;
#endif /* CTRL_VERTICAL_LANDING */

  /* outer loop */
  v_ctl_altitude_setpoint = 0.;
  v_ctl_altitude_pre_climb = 0.;
  v_ctl_altitude_pgain = V_CTL_ALTITUDE_PGAIN;
  v_ctl_altitude_error = 0.;
  v_ctl_altitude_pre_climb_correction = V_CTL_ALTITUDE_PRE_CLIMB_CORRECTION;
  v_ctl_altitude_max_climb = V_CTL_ALTITUDE_MAX_CLIMB;

  /* inner loops */
  v_ctl_climb_setpoint = 0.;
  v_ctl_climb_mode = V_CTL_CLIMB_MODE_AUTO_THROTTLE;
  v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_STANDARD;

  v_ctl_pitch_setpoint = 0.;
  v_ctl_pitch_trim = V_CTL_PITCH_TRIM;

  /* "auto throttle" inner loop parameters */
  v_ctl_auto_throttle_nominal_cruise_throttle = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  v_ctl_auto_throttle_min_cruise_throttle = V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE;
  v_ctl_auto_throttle_max_cruise_throttle = V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE;
  v_ctl_auto_throttle_cruise_throttle = v_ctl_auto_throttle_nominal_cruise_throttle;
  v_ctl_auto_throttle_climb_throttle_increment = V_CTL_AUTO_THROTTLE_CLIMB_THROTTLE_INCREMENT;
  v_ctl_auto_throttle_pgain = V_CTL_AUTO_THROTTLE_PGAIN;
  v_ctl_auto_throttle_igain = V_CTL_AUTO_THROTTLE_IGAIN;
  v_ctl_auto_throttle_dgain = V_CTL_AUTO_THROTTLE_DGAIN;
  v_ctl_auto_throttle_sum_err = 0.;
  v_ctl_auto_throttle_pitch_of_vz_pgain = V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_PGAIN;
  v_ctl_auto_throttle_pitch_of_vz_dgain = V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_DGAIN;

#ifdef V_CTL_AUTO_PITCH_PGAIN
  /* "auto pitch" inner loop parameters */
  v_ctl_auto_pitch_pgain = V_CTL_AUTO_PITCH_PGAIN;
  v_ctl_auto_pitch_igain = V_CTL_AUTO_PITCH_IGAIN;
  v_ctl_auto_pitch_sum_err = 0.;
#endif

#if USE_AIRSPEED
  v_ctl_auto_airspeed_setpoint = V_CTL_AUTO_AIRSPEED_SETPOINT;
  v_ctl_auto_airspeed_controlled = V_CTL_AUTO_AIRSPEED_SETPOINT;
  v_ctl_auto_airspeed_pgain = V_CTL_AUTO_AIRSPEED_PGAIN;
  v_ctl_auto_airspeed_igain = V_CTL_AUTO_AIRSPEED_IGAIN;
  v_ctl_auto_airspeed_sum_err = 0.;

  v_ctl_auto_groundspeed_setpoint = V_CTL_AUTO_GROUNDSPEED_SETPOINT;
  v_ctl_auto_groundspeed_pgain = V_CTL_AUTO_GROUNDSPEED_PGAIN;
  v_ctl_auto_groundspeed_igain = V_CTL_AUTO_GROUNDSPEED_IGAIN;
  v_ctl_auto_groundspeed_sum_err = 0.;
#endif

  v_ctl_throttle_setpoint = 0;

  /*agressive tuning*/
#ifdef TUNE_AGRESSIVE_CLIMB
  agr_climb_throttle = AGR_CLIMB_THROTTLE;
#undef   AGR_CLIMB_THROTTLE
#define AGR_CLIMB_THROTTLE agr_climb_throttle
  agr_climb_pitch = AGR_CLIMB_PITCH;
#undef   AGR_CLIMB_PITCH
#define   AGR_CLIMB_PITCH agr_climb_pitch
  agr_climb_nav_ratio = AGR_CLIMB_NAV_RATIO;
#undef   AGR_CLIMB_NAV_RATIO
#define   AGR_CLIMB_NAV_RATIO agr_climb_nav_ratio
  agr_descent_throttle = AGR_DESCENT_THROTTLE;
#undef   AGR_DESCENT_THROTTLE
#define   AGR_DESCENT_THROTTLE agr_descent_throttle
  agr_descent_pitch = AGR_DESCENT_PITCH;
#undef   AGR_DESCENT_PITCH
#define   AGR_DESCENT_PITCH agr_descent_pitch
  agr_descent_nav_ratio = AGR_DESCENT_NAV_RATIO;
#undef   AGR_DESCENT_NAV_RATIO
#define   AGR_DESCENT_NAV_RATIO agr_descent_nav_ratio
#endif
}

void v_ctl_guidance_loop(void)
{
  if (v_ctl_mode == V_CTL_MODE_AUTO_ALT) {
    v_ctl_altitude_loop();
  }
#if CTRL_VERTICAL_LANDING
  if (v_ctl_mode == V_CTL_MODE_LANDING) {
    v_ctl_landing_loop();
  } else {
#endif
    if (v_ctl_mode == V_CTL_MODE_AUTO_THROTTLE) {
      v_ctl_throttle_setpoint = nav_throttle_setpoint;
      v_ctl_pitch_setpoint = nav_pitch;
    } else {
      if (v_ctl_mode >= V_CTL_MODE_AUTO_CLIMB) {
        v_ctl_climb_loop();
      } /* v_ctl_mode >= V_CTL_MODE_AUTO_CLIMB */
    } /* v_ctl_mode == V_CTL_MODE_AUTO_THROTTLE */
#if CTRL_VERTICAL_LANDING
  } /* v_ctl_mode == V_CTL_MODE_LANDING */
#endif

#if defined V_CTL_THROTTLE_IDLE
  Bound(v_ctl_throttle_setpoint, TRIM_PPRZ(V_CTL_THROTTLE_IDLE * MAX_PPRZ), MAX_PPRZ);
#endif

#ifdef V_CTL_POWER_CTL_BAT_NOMINAL
  if (ap_electrical.vsupply > 0.) {
    v_ctl_throttle_setpoint *= V_CTL_POWER_CTL_BAT_NOMINAL / ap_electrical.vsupply;
    v_ctl_throttle_setpoint = TRIM_UPPRZ(v_ctl_throttle_setpoint);
  }
#endif

  if (autopilot.kill_throttle || (!autopilot.flight_time && !autopilot.launch)) {
    v_ctl_throttle_setpoint = 0;
  }
}


/**
 * outer loop
 * \brief Computes v_ctl_climb_setpoint and sets v_ctl_auto_throttle_submode
 */
void v_ctl_altitude_loop(void)
{
  float altitude_pgain_boost = 1.0;

#if USE_AIRSPEED && defined(AGR_CLIMB)
  // Aggressive climb mode (boost gain of altitude loop)
  if (v_ctl_climb_mode == V_CTL_CLIMB_MODE_AUTO_THROTTLE) {
    float dist = fabs(v_ctl_altitude_error);
    altitude_pgain_boost = 1.0 + (V_CTL_AUTO_AGR_CLIMB_GAIN - 1.0) * (dist - AGR_BLEND_END) /
                           (AGR_BLEND_START - AGR_BLEND_END);
    Bound(altitude_pgain_boost, 1.0, V_CTL_AUTO_AGR_CLIMB_GAIN);
  }
#endif

  v_ctl_altitude_error = v_ctl_altitude_setpoint - stateGetPositionUtm_f()->alt;
  v_ctl_climb_setpoint = altitude_pgain_boost * v_ctl_altitude_pgain * v_ctl_altitude_error
                         + v_ctl_altitude_pre_climb * v_ctl_altitude_pre_climb_correction;
  BoundAbs(v_ctl_climb_setpoint, v_ctl_altitude_max_climb);

#ifdef AGR_CLIMB
  if (v_ctl_climb_mode == V_CTL_CLIMB_MODE_AUTO_THROTTLE) {
    float dist = fabs(v_ctl_altitude_error);
    if (dist < AGR_BLEND_END) {
      v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_STANDARD;
    } else if (dist > AGR_BLEND_START) {
      v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_AGRESSIVE;
    } else {
      v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_BLENDED;
    }
  }
#endif
}

void v_ctl_climb_loop(void)
{
  switch (v_ctl_climb_mode) {
    case V_CTL_CLIMB_MODE_AUTO_THROTTLE:
    default:
      v_ctl_climb_auto_throttle_loop();
      break;
#ifdef V_CTL_AUTO_PITCH_PGAIN
#pragma message "AUTO PITCH Enabled!"
    case V_CTL_CLIMB_MODE_AUTO_PITCH:
      v_ctl_climb_auto_pitch_loop();
      break;
#endif
  }
}

void v_ctl_landing_loop(void)
{
#if CTRL_VERTICAL_LANDING
  static float land_speed_i_err;
  static float land_alt_i_err;
  static float kill_alt;
  float land_speed_err = v_ctl_landing_desired_speed - stateGetHorizontalSpeedNorm_f();
  float land_alt_err = v_ctl_altitude_setpoint - stateGetPositionUtm_f()->alt;

  if (autopilot_throttle_killed()
      && (kill_alt - v_ctl_altitude_setpoint)
          > (v_ctl_landing_alt_throttle_kill - v_ctl_landing_alt_flare)) {
    v_ctl_throttle_setpoint = 0.0;  // Throttle is already in KILL (command redundancy)
    nav_pitch = v_ctl_landing_pitch_flare;  // desired final flare pitch
    lateral_mode = LATERAL_MODE_ROLL;  //override course correction during flare - eliminate possibility of catching wing tip due to course correction
    h_ctl_roll_setpoint = 0.0;  // command zero roll during flare
  } else {
    // set throttle based on altitude error
    v_ctl_throttle_setpoint = land_alt_err * v_ctl_landing_throttle_pgain
        + land_alt_i_err * v_ctl_landing_throttle_igain;
    Bound(v_ctl_throttle_setpoint, 0, v_ctl_landing_throttle_max * MAX_PPRZ);  // cut off throttle cmd at specified MAX

    land_alt_i_err += land_alt_err / CONTROL_FREQUENCY;  // integrator land_alt_err, divide by control frequency
    BoundAbs(land_alt_i_err, 50);  // integrator sat limits

    // set pitch based on ground speed error
    nav_pitch -= land_speed_err * v_ctl_landing_pitch_pgain / 1000
        + land_speed_i_err * v_ctl_landing_pitch_igain / 1000;  // 1000 is a multiplier to get more reasonable gains for ctl_basic
    Bound(nav_pitch, -v_ctl_landing_pitch_limits, v_ctl_landing_pitch_limits);  //max pitch authority for landing

    land_speed_i_err += land_speed_err / CONTROL_FREQUENCY;  // integrator land_speed_err, divide by control frequency
    BoundAbs(land_speed_i_err, .2);  // integrator sat limits

    // update kill_alt until final kill throttle is initiated - allows for mode switch to first part of if statement above
    // eliminates the need for knowing the altitude of TD
    if (!autopilot_throttle_killed()) {
      kill_alt = v_ctl_altitude_setpoint;  //
      if (land_alt_err > 0.0) {
        nav_pitch = 0.01;  //  if below desired alt close to ground command level pitch
      }
    }
  }
#endif /* CTRL_VERTICAL_LANDING */
}

/**
 * auto throttle inner loop
 * \brief
 */

#if !USE_AIRSPEED

inline static void v_ctl_climb_auto_throttle_loop(void)
{
  static float last_err;

  float f_throttle = 0;
  float err  = stateGetSpeedEnu_f()->z - v_ctl_climb_setpoint;
  float d_err = err - last_err;
  last_err = err;
  float controlled_throttle = v_ctl_auto_throttle_cruise_throttle
                              + v_ctl_auto_throttle_climb_throttle_increment * v_ctl_climb_setpoint
                              - v_ctl_auto_throttle_pgain *
                              (err + v_ctl_auto_throttle_igain * v_ctl_auto_throttle_sum_err
                               + v_ctl_auto_throttle_dgain * d_err);

  /* pitch pre-command */
  float v_ctl_pitch_of_vz = (v_ctl_climb_setpoint + d_err * v_ctl_auto_throttle_pitch_of_vz_dgain) *
                            v_ctl_auto_throttle_pitch_of_vz_pgain;

#if defined AGR_CLIMB
  switch (v_ctl_auto_throttle_submode) {
    case V_CTL_AUTO_THROTTLE_AGRESSIVE:
      if (v_ctl_climb_setpoint > 0) { /* Climbing */
        f_throttle =  AGR_CLIMB_THROTTLE;
        v_ctl_pitch_setpoint = AGR_CLIMB_PITCH;
      } else { /* Going down */
        f_throttle =  AGR_DESCENT_THROTTLE;
        v_ctl_pitch_setpoint = AGR_DESCENT_PITCH;
      }
      break;

    case V_CTL_AUTO_THROTTLE_BLENDED: {
      float ratio = (fabs(v_ctl_altitude_error) - AGR_BLEND_END)
                    / (AGR_BLEND_START - AGR_BLEND_END);
      f_throttle = (1 - ratio) * controlled_throttle;
      v_ctl_pitch_setpoint = (1 - ratio) * (v_ctl_pitch_of_vz + v_ctl_pitch_trim);
      v_ctl_auto_throttle_sum_err += (1 - ratio) * err;
      BoundAbs(v_ctl_auto_throttle_sum_err, V_CTL_AUTO_THROTTLE_MAX_SUM_ERR);
      /* positive error -> too low */
      if (v_ctl_altitude_error > 0) {
        f_throttle +=  ratio * AGR_CLIMB_THROTTLE;
        v_ctl_pitch_setpoint += ratio * AGR_CLIMB_PITCH;
      } else {
        f_throttle += ratio * AGR_DESCENT_THROTTLE;
        v_ctl_pitch_setpoint += ratio * AGR_DESCENT_PITCH;
      }
      break;
    }

    case V_CTL_AUTO_THROTTLE_STANDARD:
    default:
#endif
      f_throttle = controlled_throttle;
      v_ctl_auto_throttle_sum_err += err;
      BoundAbs(v_ctl_auto_throttle_sum_err, V_CTL_AUTO_THROTTLE_MAX_SUM_ERR);
      v_ctl_pitch_setpoint = v_ctl_pitch_of_vz + v_ctl_pitch_trim + nav_pitch;
#if defined AGR_CLIMB
      break;
  } /* switch submode */
#endif

  v_ctl_throttle_setpoint = TRIM_UPPRZ(f_throttle * MAX_PPRZ);
}

#else // USE_AIRSPEED

inline static void v_ctl_climb_auto_throttle_loop(void)
{
  float f_throttle = 0;
  float controlled_throttle;
  float v_ctl_pitch_of_vz;

  // Limit rate of change of climb setpoint (to ensure that airspeed loop can catch-up)
  static float v_ctl_climb_setpoint_last = 0;
  float diff_climb = v_ctl_climb_setpoint - v_ctl_climb_setpoint_last;
  Bound(diff_climb, -V_CTL_AUTO_CLIMB_LIMIT, V_CTL_AUTO_CLIMB_LIMIT);
  v_ctl_climb_setpoint = v_ctl_climb_setpoint_last + diff_climb;
  v_ctl_climb_setpoint_last = v_ctl_climb_setpoint;

  // Pitch control (input: rate of climb error, output: pitch setpoint)
  float err  = stateGetSpeedEnu_f()->z - v_ctl_climb_setpoint;
  v_ctl_auto_pitch_sum_err += err;
  BoundAbs(v_ctl_auto_pitch_sum_err, V_CTL_AUTO_PITCH_MAX_SUM_ERR);
  v_ctl_pitch_of_vz = -v_ctl_auto_pitch_pgain *
                      (err + v_ctl_auto_pitch_igain * v_ctl_auto_pitch_sum_err);

  // Ground speed control loop (input: groundspeed error, output: airspeed controlled)
  float err_groundspeed = (v_ctl_auto_groundspeed_setpoint - stateGetHorizontalSpeedNorm_f());
  v_ctl_auto_groundspeed_sum_err += err_groundspeed;
  BoundAbs(v_ctl_auto_groundspeed_sum_err, V_CTL_AUTO_GROUNDSPEED_MAX_SUM_ERR);
  v_ctl_auto_airspeed_controlled = (err_groundspeed + v_ctl_auto_groundspeed_sum_err * v_ctl_auto_groundspeed_igain) *
                                   v_ctl_auto_groundspeed_pgain;

  // Do not allow controlled airspeed below the setpoint
  if (v_ctl_auto_airspeed_controlled < v_ctl_auto_airspeed_setpoint) {
    v_ctl_auto_airspeed_controlled = v_ctl_auto_airspeed_setpoint;
    v_ctl_auto_groundspeed_sum_err = v_ctl_auto_airspeed_controlled / (v_ctl_auto_groundspeed_pgain *
                                     v_ctl_auto_groundspeed_igain); // reset integrator of ground speed loop
  }

  // Airspeed control loop (input: airspeed controlled, output: throttle controlled)
  float err_airspeed = (v_ctl_auto_airspeed_controlled - stateGetAirspeed_f());
  v_ctl_auto_airspeed_sum_err += err_airspeed;
  BoundAbs(v_ctl_auto_airspeed_sum_err, V_CTL_AUTO_AIRSPEED_MAX_SUM_ERR);
  controlled_throttle = (err_airspeed + v_ctl_auto_airspeed_sum_err * v_ctl_auto_airspeed_igain) *
                        v_ctl_auto_airspeed_pgain;

  // Done, set outputs
  Bound(controlled_throttle, 0, v_ctl_auto_throttle_max_cruise_throttle);
  f_throttle = controlled_throttle;
  v_ctl_pitch_setpoint = v_ctl_pitch_of_vz + v_ctl_pitch_trim;
  v_ctl_throttle_setpoint = TRIM_UPPRZ(f_throttle * MAX_PPRZ);
  Bound(v_ctl_pitch_setpoint, V_CTL_AUTO_PITCH_MIN_PITCH, V_CTL_AUTO_PITCH_MAX_PITCH);
}

#endif // USE_AIRSPEED


/**
 * auto pitch inner loop
 * \brief computes a v_ctl_pitch_setpoint from a climb_setpoint given a fixed throttle
 */
#ifdef V_CTL_AUTO_PITCH_PGAIN
inline static void v_ctl_climb_auto_pitch_loop(void)
{
  float err  = stateGetSpeedEnu_f()->z - v_ctl_climb_setpoint;
  v_ctl_throttle_setpoint = nav_throttle_setpoint;
  v_ctl_auto_pitch_sum_err += err;
  BoundAbs(v_ctl_auto_pitch_sum_err, V_CTL_AUTO_PITCH_MAX_SUM_ERR);
  v_ctl_pitch_setpoint = v_ctl_pitch_trim - v_ctl_auto_pitch_pgain *
                         (err + v_ctl_auto_pitch_igain * v_ctl_auto_pitch_sum_err);
  Bound(v_ctl_pitch_setpoint, V_CTL_AUTO_PITCH_MIN_PITCH, V_CTL_AUTO_PITCH_MAX_PITCH);
}
#endif

#ifdef V_CTL_THROTTLE_SLEW_LIMITER
#define V_CTL_THROTTLE_SLEW (1./CONTROL_FREQUENCY/(V_CTL_THROTTLE_SLEW_LIMITER))
#endif

#ifndef V_CTL_THROTTLE_SLEW
#define V_CTL_THROTTLE_SLEW 1.
#endif
/** \brief Computes slewed throttle from throttle setpoint
    called at 20Hz
 */
void v_ctl_throttle_slew(void)
{
  pprz_t diff_throttle = v_ctl_throttle_setpoint - v_ctl_throttle_slewed;
  BoundAbs(diff_throttle, TRIM_PPRZ(V_CTL_THROTTLE_SLEW * MAX_PPRZ));
  v_ctl_throttle_slewed += diff_throttle;
}
