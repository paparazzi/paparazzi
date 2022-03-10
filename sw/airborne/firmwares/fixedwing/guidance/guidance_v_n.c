/*
 * Copyright (C) 2010 ENAC
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
 *  @file firmwares/fixedwing/guidance/guidance_v_n.c
 *  "New" vertical control for fixed wing vehicles.
 *
 */

#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "firmwares/fixedwing/guidance/guidance_v_n.h"
#include "state.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/airframe.h"
#include "autopilot.h"

/* mode */
uint8_t v_ctl_mode;

/* outer loop */
float v_ctl_altitude_setpoint;
float v_ctl_altitude_pre_climb;
float v_ctl_altitude_pgain;
float v_ctl_altitude_error;

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
#define V_CTL_AUTO_THROTTLE_MAX_SUM_ERR 0.4
float v_ctl_auto_throttle_pitch_of_vz_pgain;
float v_ctl_auto_throttle_pitch_of_vz_dgain;

#ifndef V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_DGAIN
#define V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_DGAIN 0.
#endif

/* "auto pitch" inner loop parameters */
float v_ctl_auto_pitch_pgain;
float v_ctl_auto_pitch_dgain;
float v_ctl_auto_pitch_igain;
float v_ctl_auto_pitch_sum_err;
#define V_CTL_AUTO_PITCH_MAX_SUM_ERR (RadOfDeg(10.))

#ifndef V_CTL_AUTO_PITCH_DGAIN
#define V_CTL_AUTO_PITCH_DGAIN 0.
#endif
#ifndef V_CTL_AUTO_PITCH_IGAIN
#define V_CTL_AUTO_PITCH_IGAIN 0.
#endif

float controlled_throttle;
pprz_t v_ctl_throttle_setpoint;
pprz_t v_ctl_throttle_slewed;
float v_ctl_pitch_setpoint;
#ifndef V_CTL_PITCH_TRIM
#define V_CTL_PITCH_TRIM 0.
#endif
float v_ctl_pitch_trim;

// Set higher than 2*V_CTL_ALTITUDE_MAX_CLIMB to disable
#ifndef V_CTL_AUTO_CLIMB_LIMIT
#define V_CTL_AUTO_CLIMB_LIMIT (0.5/4.0) // m/s/s
#endif

uint8_t v_ctl_speed_mode;

#if USE_AIRSPEED
float v_ctl_auto_airspeed_setpoint;
float v_ctl_auto_airspeed_controlled;
float v_ctl_auto_airspeed_throttle_pgain;
float v_ctl_auto_airspeed_throttle_dgain;
float v_ctl_auto_airspeed_throttle_igain;
float v_ctl_auto_airspeed_throttle_sum_err;
float v_ctl_auto_airspeed_pitch_pgain;
float v_ctl_auto_airspeed_pitch_dgain;
float v_ctl_auto_airspeed_pitch_igain;
float v_ctl_auto_airspeed_pitch_sum_err;
float v_ctl_auto_groundspeed_setpoint;
float v_ctl_auto_groundspeed_pgain;
float v_ctl_auto_groundspeed_igain;
float v_ctl_auto_groundspeed_sum_err;
#define V_CTL_AUTO_AIRSPEED_PITCH_MAX_SUM_ERR (RadOfDeg(15.))
#define V_CTL_AUTO_AIRSPEED_THROTTLE_MAX_SUM_ERR 0.2
#define V_CTL_AUTO_GROUNDSPEED_MAX_SUM_ERR 100
#endif

void v_ctl_init(void)
{
  /* mode */
  v_ctl_mode = V_CTL_MODE_MANUAL;
  v_ctl_speed_mode = V_CTL_SPEED_THROTTLE;

  /* outer loop */
  v_ctl_altitude_setpoint = 0.;
  v_ctl_altitude_pre_climb = 0.;
  v_ctl_altitude_pgain = V_CTL_ALTITUDE_PGAIN;
  v_ctl_altitude_error = 0.;

  /* inner loops */
  v_ctl_climb_setpoint = 0.;
  v_ctl_climb_mode = V_CTL_CLIMB_MODE_AUTO_THROTTLE;
  v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_STANDARD;

  v_ctl_pitch_setpoint = 0.;
  v_ctl_pitch_trim = V_CTL_PITCH_TRIM;

  /* "auto throttle" inner loop parameters */
  v_ctl_auto_throttle_nominal_cruise_throttle = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  v_ctl_auto_throttle_cruise_throttle = v_ctl_auto_throttle_nominal_cruise_throttle;
  v_ctl_auto_throttle_min_cruise_throttle = V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE;
  v_ctl_auto_throttle_max_cruise_throttle = V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE;
  v_ctl_auto_throttle_climb_throttle_increment = V_CTL_AUTO_THROTTLE_CLIMB_THROTTLE_INCREMENT;
  v_ctl_auto_throttle_pgain = V_CTL_AUTO_THROTTLE_PGAIN;
  v_ctl_auto_throttle_igain = V_CTL_AUTO_THROTTLE_IGAIN;
  v_ctl_auto_throttle_dgain = V_CTL_AUTO_THROTTLE_DGAIN;
  v_ctl_auto_throttle_sum_err = 0.;
  v_ctl_auto_throttle_pitch_of_vz_pgain = V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_PGAIN;
  v_ctl_auto_throttle_pitch_of_vz_dgain = V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_DGAIN;

  /* "auto pitch" inner loop parameters */
  v_ctl_auto_pitch_pgain = V_CTL_AUTO_PITCH_PGAIN;
  v_ctl_auto_pitch_dgain = V_CTL_AUTO_PITCH_DGAIN;
  v_ctl_auto_pitch_igain = V_CTL_AUTO_PITCH_IGAIN;
  v_ctl_auto_pitch_sum_err = 0.;

#if USE_AIRSPEED
  v_ctl_auto_airspeed_setpoint = V_CTL_AUTO_AIRSPEED_SETPOINT;
  v_ctl_auto_airspeed_controlled = V_CTL_AUTO_AIRSPEED_SETPOINT;
  v_ctl_auto_airspeed_throttle_pgain = V_CTL_AUTO_AIRSPEED_THROTTLE_PGAIN;
  v_ctl_auto_airspeed_throttle_dgain = V_CTL_AUTO_AIRSPEED_THROTTLE_DGAIN;
  v_ctl_auto_airspeed_throttle_igain = V_CTL_AUTO_AIRSPEED_THROTTLE_IGAIN;
  v_ctl_auto_airspeed_throttle_sum_err = 0.;
  v_ctl_auto_airspeed_pitch_pgain = V_CTL_AUTO_AIRSPEED_PITCH_PGAIN;
  v_ctl_auto_airspeed_pitch_dgain = V_CTL_AUTO_AIRSPEED_PITCH_DGAIN;
  v_ctl_auto_airspeed_pitch_igain = V_CTL_AUTO_AIRSPEED_PITCH_IGAIN;
  v_ctl_auto_airspeed_pitch_sum_err = 0.;

  v_ctl_auto_groundspeed_setpoint = V_CTL_AUTO_GROUNDSPEED_SETPOINT;
  v_ctl_auto_groundspeed_pgain = V_CTL_AUTO_GROUNDSPEED_PGAIN;
  v_ctl_auto_groundspeed_igain = V_CTL_AUTO_GROUNDSPEED_IGAIN;
  v_ctl_auto_groundspeed_sum_err = 0.;
#endif

  controlled_throttle = 0.;
  v_ctl_throttle_setpoint = 0;
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
  if (electrical.vsupply > 0.) {
    v_ctl_throttle_setpoint *= V_CTL_POWER_CTL_BAT_NOMINAL / electrical.vsupply;
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

// Don't use lead controller unless you know what you're doing
#define LEAD_CTRL_TAU 0.8
#define LEAD_CTRL_A 1.
#define LEAD_CTRL_Te (1./60.)

void v_ctl_altitude_loop(void)
{
  static float v_ctl_climb_setpoint_last = 0.;
  //static float last_lead_input = 0.;

  // Altitude error
  v_ctl_altitude_error = v_ctl_altitude_setpoint - stateGetPositionUtm_f()->alt;
  v_ctl_climb_setpoint = v_ctl_altitude_pgain * v_ctl_altitude_error + v_ctl_altitude_pre_climb;

  // Lead controller
  //v_ctl_climb_setpoint = ((LEAD_CTRL_TAU*LEAD_CTRL_A + LEAD_CTRL_Te)*climb_sp + LEAD_CTRL_TAU*(v_ctl_climb_setpoint - LEAD_CTRL_A*last_lead_input)) / (LEAD_CTRL_TAU + LEAD_CTRL_Te);
  //last_lead_input = pitch_sp;

  // Limit rate of change of climb setpoint
  float diff_climb = v_ctl_climb_setpoint - v_ctl_climb_setpoint_last;
  BoundAbs(diff_climb, V_CTL_AUTO_CLIMB_LIMIT);
  v_ctl_climb_setpoint = v_ctl_climb_setpoint_last + diff_climb;

  // Limit climb setpoint
  BoundAbs(v_ctl_climb_setpoint, V_CTL_ALTITUDE_MAX_CLIMB);
  v_ctl_climb_setpoint_last = v_ctl_climb_setpoint;
}

static inline void v_ctl_set_pitch(void)
{
  static float last_err = 0.;

  if (autopilot_get_mode() == AP_MODE_MANUAL || autopilot.launch == false) {
    v_ctl_auto_pitch_sum_err = 0;
  }

  // Compute errors
  float err = v_ctl_climb_setpoint - stateGetSpeedEnu_f()->z;
  float d_err = err - last_err;
  last_err = err;

  if (v_ctl_auto_pitch_igain > 0.) {
    v_ctl_auto_pitch_sum_err += err * (1. / 60.);
    BoundAbs(v_ctl_auto_pitch_sum_err, V_CTL_AUTO_PITCH_MAX_SUM_ERR / v_ctl_auto_pitch_igain);
  }

  // PI loop + feedforward ctl
  v_ctl_pitch_setpoint = nav_pitch
                         + v_ctl_pitch_trim
                         + v_ctl_auto_throttle_pitch_of_vz_pgain * v_ctl_climb_setpoint
                         + v_ctl_auto_pitch_pgain * err
                         + v_ctl_auto_pitch_dgain * d_err
                         + v_ctl_auto_pitch_igain * v_ctl_auto_pitch_sum_err;

}

static inline void v_ctl_set_throttle(void)
{
  static float last_err = 0.;

  if (autopilot_get_mode() == AP_MODE_MANUAL || autopilot.launch == false) {
    v_ctl_auto_throttle_sum_err = 0;
  }

  // Compute errors
  float err =  v_ctl_climb_setpoint - stateGetSpeedEnu_f()->z;
  float d_err = err - last_err;
  last_err = err;

  if (v_ctl_auto_throttle_igain > 0.) {
    v_ctl_auto_throttle_sum_err += err * (1. / 60.);
    BoundAbs(v_ctl_auto_throttle_sum_err, V_CTL_AUTO_THROTTLE_MAX_SUM_ERR / v_ctl_auto_throttle_igain);
  }

  // PID loop + feedforward ctl
  controlled_throttle = v_ctl_auto_throttle_cruise_throttle
                        + v_ctl_auto_throttle_climb_throttle_increment * v_ctl_climb_setpoint
                        + v_ctl_auto_throttle_pgain * err
                        + v_ctl_auto_throttle_dgain * d_err
                        + v_ctl_auto_throttle_igain * v_ctl_auto_throttle_sum_err;

}

#if USE_AIRSPEED
#define AIRSPEED_LOOP_PERIOD (1./60.)

// Airspeed control loop (input: [airspeed controlled, climb_setpoint], output: [throttle controlled, pitch setpoint])
static inline void v_ctl_set_airspeed(void)
{
  static float last_err_vz = 0.;
  static float last_err_as = 0.;

  // Bound airspeed setpoint
  Bound(v_ctl_auto_airspeed_setpoint, V_CTL_AIRSPEED_MIN, V_CTL_AIRSPEED_MAX);

  // Compute errors
  float err_vz = v_ctl_climb_setpoint - stateGetSpeedEnu_f()->z;
  float d_err_vz = (err_vz - last_err_vz) * AIRSPEED_LOOP_PERIOD;
  last_err_vz = err_vz;
  if (v_ctl_auto_throttle_igain > 0.) {
    v_ctl_auto_throttle_sum_err += err_vz * AIRSPEED_LOOP_PERIOD;
    BoundAbs(v_ctl_auto_throttle_sum_err, V_CTL_AUTO_THROTTLE_MAX_SUM_ERR / v_ctl_auto_throttle_igain);
  }
  if (v_ctl_auto_pitch_igain > 0.) {
    v_ctl_auto_pitch_sum_err += err_vz * AIRSPEED_LOOP_PERIOD;
    BoundAbs(v_ctl_auto_pitch_sum_err, V_CTL_AUTO_PITCH_MAX_SUM_ERR / v_ctl_auto_pitch_igain);
  }

  float err_airspeed = v_ctl_auto_airspeed_setpoint - stateGetAirspeed_f();
  float d_err_airspeed = (err_airspeed - last_err_as) * AIRSPEED_LOOP_PERIOD;
  last_err_as = err_airspeed;
  if (v_ctl_auto_airspeed_throttle_igain > 0.) {
    v_ctl_auto_airspeed_throttle_sum_err += err_airspeed * AIRSPEED_LOOP_PERIOD;
    BoundAbs(v_ctl_auto_airspeed_throttle_sum_err,
             V_CTL_AUTO_AIRSPEED_THROTTLE_MAX_SUM_ERR / v_ctl_auto_airspeed_throttle_igain);
  }
  if (v_ctl_auto_airspeed_pitch_igain > 0.) {
    v_ctl_auto_airspeed_pitch_sum_err += err_airspeed * AIRSPEED_LOOP_PERIOD;
    BoundAbs(v_ctl_auto_airspeed_pitch_sum_err, V_CTL_AUTO_AIRSPEED_PITCH_MAX_SUM_ERR / v_ctl_auto_airspeed_pitch_igain);
  }


  // Reset integrators in manual or before flight
  if (autopilot_get_mode() == AP_MODE_MANUAL || autopilot.launch == false) {
    v_ctl_auto_throttle_sum_err = 0.;
    v_ctl_auto_pitch_sum_err = 0.;
    v_ctl_auto_airspeed_throttle_sum_err = 0.;
    v_ctl_auto_airspeed_pitch_sum_err = 0.;
  }

  // Pitch loop
  v_ctl_pitch_setpoint =
    v_ctl_auto_throttle_pitch_of_vz_pgain * v_ctl_climb_setpoint
    + v_ctl_pitch_trim
    + v_ctl_auto_pitch_pgain * err_vz
    + v_ctl_auto_pitch_dgain * d_err_vz
    + v_ctl_auto_pitch_igain * v_ctl_auto_pitch_sum_err
    - v_ctl_auto_airspeed_pitch_pgain * err_airspeed
    - v_ctl_auto_airspeed_pitch_dgain * d_err_airspeed
    - v_ctl_auto_airspeed_pitch_igain * v_ctl_auto_airspeed_pitch_sum_err;

  // Throttle loop
  controlled_throttle = v_ctl_auto_throttle_cruise_throttle
                        + v_ctl_auto_throttle_climb_throttle_increment * v_ctl_climb_setpoint
                        + v_ctl_auto_throttle_pgain * err_vz
                        + v_ctl_auto_throttle_dgain * d_err_vz
                        + v_ctl_auto_throttle_igain * v_ctl_auto_throttle_sum_err
                        + v_ctl_auto_airspeed_throttle_pgain * err_airspeed
                        + v_ctl_auto_airspeed_throttle_dgain * d_err_airspeed
                        + v_ctl_auto_airspeed_throttle_igain * v_ctl_auto_airspeed_throttle_sum_err;

}

static inline void v_ctl_set_groundspeed(void)
{
  // Ground speed control loop (input: groundspeed error, output: airspeed controlled)
  float err_groundspeed = v_ctl_auto_groundspeed_setpoint - stateGetHorizontalSpeedNorm_f();
  v_ctl_auto_groundspeed_sum_err += err_groundspeed;
  BoundAbs(v_ctl_auto_groundspeed_sum_err, V_CTL_AUTO_GROUNDSPEED_MAX_SUM_ERR);
  v_ctl_auto_airspeed_setpoint = err_groundspeed * v_ctl_auto_groundspeed_pgain + v_ctl_auto_groundspeed_sum_err *
                                 v_ctl_auto_groundspeed_igain;

}
#endif

void v_ctl_climb_loop(void)
{

  switch (v_ctl_speed_mode) {
    case V_CTL_SPEED_THROTTLE:
      // Set pitch
      v_ctl_set_pitch();
      // Set throttle
      v_ctl_set_throttle();
      break;
#if USE_AIRSPEED
    case V_CTL_SPEED_AIRSPEED:
      v_ctl_set_airspeed();
      break;
    case V_CTL_SPEED_GROUNDSPEED:
      v_ctl_set_groundspeed();
      v_ctl_set_airspeed();
      break;
#endif
    default:
      controlled_throttle = v_ctl_auto_throttle_cruise_throttle; // ???
      break;
  }

  // Set Pitch output
  Bound(v_ctl_pitch_setpoint, V_CTL_AUTO_PITCH_MIN_PITCH, V_CTL_AUTO_PITCH_MAX_PITCH);
  // Set Throttle output
  v_ctl_throttle_setpoint = TRIM_UPPRZ(controlled_throttle * MAX_PPRZ);

}

#ifdef V_CTL_THROTTLE_SLEW_LIMITER
#define V_CTL_THROTTLE_SLEW (1./CONTROL_FREQUENCY/(V_CTL_THROTTLE_SLEW_LIMITER))
#endif

#ifndef V_CTL_THROTTLE_SLEW
#define V_CTL_THROTTLE_SLEW (1.)
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
