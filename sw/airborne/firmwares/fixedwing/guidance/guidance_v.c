/*
 * $Id$
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
 *  \file v_ctl_ctl
 *  \brief Vertical control for fixed wing vehicles.
 *
 */

#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "estimator.h"
#include "subsystems/nav.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/autopilot.h"

/* mode */
uint8_t v_ctl_mode;
uint8_t v_ctl_airspeed_mode;

/* outer loop */
float v_ctl_altitude_setpoint;
float v_ctl_altitude_pre_climb;
float v_ctl_altitude_pgain;
float v_ctl_altitude_error;

/* inner loop */
float v_ctl_climb_setpoint;
uint8_t v_ctl_climb_mode;
uint8_t v_ctl_auto_throttle_submode;

/* "auto throttle" inner loop parameters */
float v_ctl_auto_throttle_cruise_throttle;
float v_ctl_auto_throttle_nominal_cruise_throttle;
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

inline static void v_ctl_climb_auto_throttle_loop( void );
//#ifdef V_CTL_AUTO_PITCH_PGAIN
//inline static void v_ctl_climb_auto_pitch_loop( void );
//#endif

#ifdef USE_AIRSPEED

//Airspeed Setpoint
float v_ctl_auto_airspeed_setpoint;
float v_ctl_auto_airspeed_setpoint_new;
float v_ctl_auto_airspeed_setpoint_slew_increment;
float v_ctl_auto_airspeed_controlled;	
//Vassilis
float v_ctl_auto_airspeed_throttle_pgain_vas;
float v_ctl_auto_airspeed_throttle_igain_vas;
float v_ctl_auto_airspeed_pitch_pgain_vas;
float v_ctl_auto_airspeed_pitch_igain_vas;
//ZHAW Pitch Simple
float v_ctl_auto_airspeed_throttle_pgain_zps;
float v_ctl_auto_airspeed_throttle_igain_zps;
float v_ctl_auto_airspeed_prethrottle_zps;
float v_ctl_auto_airspeed_pitch_pgain_zps;
float v_ctl_auto_airspeed_pitch_igain_zps;
//Error boundary
float v_ctl_auto_airspeed_sum_err;
float v_ctl_auto_alt_sum_err;
#define V_CTL_AUTO_AIRSPEED_MAX_SUM_ERR 200
#define V_CTL_AUTO_ALT_MAX_SUM_ERR 200
#define V_CTL_AUTO_PITCH_MAX_SUM_ERR 100
//Groundspeed conditions
float v_ctl_auto_groundspeed_setpoint;
float v_ctl_auto_groundspeed_pgain;
float v_ctl_auto_groundspeed_igain;
float v_ctl_auto_groundspeed_sum_err;

#define V_CTL_AUTO_GROUNDSPEED_MAX_SUM_ERR 100

#ifndef V_CTL_AUTO_CLIMB_LIMIT
  #define V_CTL_AUTO_CLIMB_LIMIT 0.5/4.0 // m/s/s
#endif

#define V_CTL_AUTO_AGR_CLIMB_GAIN 2.0 // altitude gain multiplier while in aggressive climb mode

#ifndef V_CTL_AIRSPEED_MODE
  #define V_CTL_AIRSPEED_MODE AS_MODE_STANDARD
#endif

#endif


void v_ctl_init( void ) {
  /* mode */
  v_ctl_mode = V_CTL_MODE_MANUAL;

  /* outer loop */
  v_ctl_altitude_setpoint = 0.;
  v_ctl_altitude_pre_climb = 0.;
  v_ctl_altitude_pgain = V_CTL_ALTITUDE_PGAIN;
  v_ctl_altitude_error = 0.;

  /* inner loops */
  v_ctl_climb_setpoint = 0.;
  v_ctl_climb_mode = V_CTL_CLIMB_MODE_AUTO_THROTTLE;
  v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_STANDARD;

  /* "auto throttle" inner loop parameters */
  v_ctl_auto_throttle_nominal_cruise_throttle = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  v_ctl_auto_throttle_cruise_throttle = v_ctl_auto_throttle_nominal_cruise_throttle;
  v_ctl_auto_throttle_climb_throttle_increment = V_CTL_AUTO_THROTTLE_CLIMB_THROTTLE_INCREMENT;
  v_ctl_auto_throttle_pgain = V_CTL_AUTO_THROTTLE_PGAIN;
  v_ctl_auto_throttle_igain = V_CTL_AUTO_THROTTLE_IGAIN;
  v_ctl_auto_throttle_dgain = 0.;
  v_ctl_auto_throttle_sum_err = 0.;
  v_ctl_auto_throttle_pitch_of_vz_pgain = V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_PGAIN;
  v_ctl_auto_throttle_pitch_of_vz_dgain = V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_DGAIN;

#ifdef V_CTL_AUTO_PITCH_PGAIN
  /* "auto pitch" inner loop parameters */
  v_ctl_auto_pitch_pgain = V_CTL_AUTO_PITCH_PGAIN;
  v_ctl_auto_pitch_igain = V_CTL_AUTO_PITCH_IGAIN;
  v_ctl_auto_pitch_sum_err = 0.;
#endif


#ifdef USE_AIRSPEED

  //Setpoint
  v_ctl_auto_airspeed_setpoint = V_CTL_AUTO_AIRSPEED_SETPOINT;
  v_ctl_auto_airspeed_setpoint_new = V_CTL_AUTO_AIRSPEED_SETPOINT;
  v_ctl_auto_airspeed_setpoint_slew_increment = V_CTL_AUTO_AIRSPEED_SETPOINT_SLEW_INCREMENT;
  v_ctl_auto_airspeed_controlled = V_CTL_AUTO_AIRSPEED_SETPOINT;

  //Vassilis
  v_ctl_auto_airspeed_throttle_pgain_vas = V_CTL_AUTO_AIRSPEED_THROTTLE_PGAIN_VAS;
  v_ctl_auto_airspeed_throttle_igain_vas = V_CTL_AUTO_AIRSPEED_THROTTLE_IGAIN_VAS;
  v_ctl_auto_airspeed_pitch_pgain_vas = V_CTL_AUTO_AIRSPEED_PITCH_PGAIN_VAS;
  v_ctl_auto_airspeed_pitch_igain_vas = V_CTL_AUTO_AIRSPEED_PITCH_IGAIN_VAS;

  //ZHAW Pitch Simple
  v_ctl_auto_airspeed_throttle_pgain_zps = V_CTL_AUTO_AIRSPEED_THROTTLE_PGAIN_ZPS;
  v_ctl_auto_airspeed_throttle_igain_zps = V_CTL_AUTO_AIRSPEED_THROTTLE_IGAIN_ZPS;
  v_ctl_auto_airspeed_prethrottle_zps = V_CTL_AUTO_AIRSPEED_PRETHROTTLE_ZPS;
  v_ctl_auto_airspeed_pitch_pgain_zps = V_CTL_AUTO_AIRSPEED_PITCH_PGAIN_ZPS;
  v_ctl_auto_airspeed_pitch_igain_zps = V_CTL_AUTO_AIRSPEED_PITCH_IGAIN_ZPS;

  //Errors
  v_ctl_auto_airspeed_sum_err = 0.;
  v_ctl_auto_alt_sum_err = 0.;

  //Groundspeed conditions
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

/**
 * outer loop
 * \brief Computes v_ctl_climb_setpoint and sets v_ctl_auto_throttle_submode
 */
void v_ctl_altitude_loop( void ) {
  float altitude_pgain_boost = 1.0;

#if defined(USE_AIRSPEED) && defined(AGR_CLIMB)
  // Aggressive climb mode (boost gain of altitude loop)
  if ( v_ctl_climb_mode == V_CTL_CLIMB_MODE_AUTO_THROTTLE) {
    float dist = fabs(v_ctl_altitude_error);
    altitude_pgain_boost = 1.0 + (V_CTL_AUTO_AGR_CLIMB_GAIN-1.0)*(dist-AGR_BLEND_END)/(AGR_BLEND_START-AGR_BLEND_END);
    Bound(altitude_pgain_boost, 1.0, V_CTL_AUTO_AGR_CLIMB_GAIN);
  }
#endif

  v_ctl_altitude_error = estimator_z - v_ctl_altitude_setpoint;
  if (v_ctl_airspeed_mode==AS_MODE_STANDARD || v_ctl_airspeed_mode==AS_MODE_VASSILLIS) {
  	v_ctl_climb_setpoint = altitude_pgain_boost * v_ctl_altitude_pgain * v_ctl_altitude_error + v_ctl_altitude_pre_climb;
  	BoundAbs(v_ctl_climb_setpoint, V_CTL_ALTITUDE_MAX_CLIMB);
  }

#ifdef AGR_CLIMB
  if ( v_ctl_climb_mode == V_CTL_CLIMB_MODE_AUTO_THROTTLE) {
    float dist = fabs(v_ctl_altitude_error);
    if (dist < AGR_BLEND_END) {
      v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_STANDARD;
    }
    else if (dist > AGR_BLEND_START) {
      v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_AGRESSIVE;
    }
    else {
      v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_BLENDED;
    }
  }
#endif
}

void v_ctl_climb_loop ( void ) {

  v_ctl_climb_auto_throttle_loop();

}

/**
 * auto throttle inner loop
 * \brief
 */

//#ifndef USE_AIRSPEED

inline static void v_ctl_climb_auto_throttle_loop(void) {

  float v_ctl_pitch_of_vz;
  float controlled_throttle;
  float f_throttle;

#ifndef USE_AIRSPEED
  v_ctl_airspeed_mode=AS_MODE_STANDARD;
#endif

  if (v_ctl_airspeed_mode==AS_MODE_STANDARD) {
    static float last_err;
    f_throttle = 0;

    float err  = estimator_z_dot - v_ctl_climb_setpoint;
    float d_err = err - last_err;
    last_err = err;
    controlled_throttle = v_ctl_auto_throttle_cruise_throttle
    + v_ctl_auto_throttle_climb_throttle_increment * v_ctl_climb_setpoint
    + v_ctl_auto_throttle_pgain *
    (err + v_ctl_auto_throttle_igain * v_ctl_auto_throttle_sum_err
    + v_ctl_auto_throttle_dgain * d_err);

    /* pitch pre-command */
    v_ctl_pitch_of_vz = (v_ctl_climb_setpoint + d_err * v_ctl_auto_throttle_pitch_of_vz_dgain) * v_ctl_auto_throttle_pitch_of_vz_pgain;

#if defined AGR_CLIMB
    switch (v_ctl_auto_throttle_submode) {
    case V_CTL_AUTO_THROTTLE_AGRESSIVE:
      if (v_ctl_climb_setpoint > 0) { /* Climbing */
        f_throttle =  AGR_CLIMB_THROTTLE;
      	nav_pitch = AGR_CLIMB_PITCH;
      }
      else { /* Going down */
     	f_throttle =  AGR_DESCENT_THROTTLE;
     	nav_pitch = AGR_DESCENT_PITCH;
      }
      break;

    case V_CTL_AUTO_THROTTLE_BLENDED: {
      float ratio = (fabs(v_ctl_altitude_error) - AGR_BLEND_END) / (AGR_BLEND_START - AGR_BLEND_END);
      f_throttle = (1-ratio) * controlled_throttle;
      nav_pitch = (1-ratio) * v_ctl_pitch_of_vz;
      v_ctl_auto_throttle_sum_err += (1-ratio) * err;
      BoundAbs(v_ctl_auto_throttle_sum_err, V_CTL_AUTO_THROTTLE_MAX_SUM_ERR);
      if (v_ctl_altitude_error < 0) {
        f_throttle +=  ratio * AGR_CLIMB_THROTTLE;
        nav_pitch += ratio * AGR_CLIMB_PITCH;
      } 
      else {
      	f_throttle += ratio * AGR_DESCENT_THROTTLE;
      	nav_pitch += ratio * AGR_DESCENT_PITCH;
      }
      break;
    }

    case V_CTL_AUTO_THROTTLE_STANDARD:
    default:
#endif
    f_throttle = controlled_throttle;
    v_ctl_auto_throttle_sum_err += err;
    BoundAbs(v_ctl_auto_throttle_sum_err, V_CTL_AUTO_THROTTLE_MAX_SUM_ERR);
    nav_pitch += v_ctl_pitch_of_vz;
#if defined AGR_CLIMB
    break;
    } /* switch submode */
#endif

  v_ctl_throttle_setpoint = TRIM_UPPRZ(f_throttle * MAX_PPRZ);
}

#if defined USE_AIRSPEED
  else { //v_ctl_airspeed_mode != AS_MODE_STANDARD

//***********AIRSPEED CONTROL (Vassilis and ZHAW Pitch Simple)***********************************

    controlled_throttle=0;
    v_ctl_pitch_of_vz=0;

    //To avoid heavy changes of the Airspeed Setpoint  -langede0
    if ( fabs( v_ctl_auto_airspeed_setpoint_new - v_ctl_auto_airspeed_setpoint ) > v_ctl_auto_airspeed_setpoint_slew_increment) {
      if (v_ctl_auto_airspeed_setpoint_new > v_ctl_auto_airspeed_setpoint) {
        v_ctl_auto_airspeed_setpoint= v_ctl_auto_airspeed_setpoint + v_ctl_auto_airspeed_setpoint_slew_increment;
      }
      else if (v_ctl_auto_airspeed_setpoint_new < v_ctl_auto_airspeed_setpoint) {
	v_ctl_auto_airspeed_setpoint= v_ctl_auto_airspeed_setpoint - v_ctl_auto_airspeed_setpoint_slew_increment;
      }
     }

    // Limit rate of change of climb setpoint (to ensure that airspeed loop can catch-up)
    static float v_ctl_climb_setpoint_last = 0;
    float diff_climb = v_ctl_climb_setpoint - v_ctl_climb_setpoint_last;
    //Bound(diff_climb, -V_CTL_AUTO_CLIMB_LIMIT, V_CTL_AUTO_CLIMB_LIMIT);
    v_ctl_climb_setpoint = v_ctl_climb_setpoint_last + diff_climb;
    v_ctl_climb_setpoint_last = v_ctl_climb_setpoint;

    // Pitch control (input: rate of climb error, output: pitch setpoint)
    float err  = estimator_z_dot - v_ctl_climb_setpoint;
    v_ctl_auto_pitch_sum_err += err;
    BoundAbs(v_ctl_auto_pitch_sum_err, V_CTL_AUTO_PITCH_MAX_SUM_ERR);

  switch (v_ctl_airspeed_mode) {
    case AS_MODE_VASSILLIS: //Vasilis - vas
      v_ctl_pitch_of_vz = v_ctl_auto_airspeed_pitch_pgain_vas * (err + v_ctl_auto_airspeed_pitch_igain_vas * v_ctl_auto_pitch_sum_err);
    break;

    case AS_MODE_ZHAW_PITCH_SIMPLE: //ZHAW Pitch Simple - zps
      v_ctl_auto_alt_sum_err += v_ctl_altitude_error;
      BoundAbs(v_ctl_auto_alt_sum_err, V_CTL_AUTO_ALT_MAX_SUM_ERR);
      controlled_throttle = v_ctl_auto_airspeed_prethrottle_zps + v_ctl_auto_airspeed_throttle_pgain_zps * (v_ctl_altitude_error + v_ctl_auto_airspeed_throttle_igain_zps * v_ctl_auto_alt_sum_err);
    break;
  }

  // Ground speed control loop (input: groundspeed error, output: airspeed controlled)
  float err_groundspeed = (v_ctl_auto_groundspeed_setpoint - estimator_hspeed_mod);
  v_ctl_auto_groundspeed_sum_err += err_groundspeed;
  BoundAbs(v_ctl_auto_groundspeed_sum_err, V_CTL_AUTO_GROUNDSPEED_MAX_SUM_ERR);
  v_ctl_auto_airspeed_controlled = (err_groundspeed + v_ctl_auto_groundspeed_sum_err * v_ctl_auto_groundspeed_igain) * v_ctl_auto_groundspeed_pgain;

  // Do not allow controlled airspeed below the setpoint
  if (v_ctl_auto_airspeed_controlled < v_ctl_auto_airspeed_setpoint) {
    v_ctl_auto_airspeed_controlled = v_ctl_auto_airspeed_setpoint;
    v_ctl_auto_groundspeed_sum_err = v_ctl_auto_airspeed_controlled/(v_ctl_auto_groundspeed_pgain*v_ctl_auto_groundspeed_igain); // reset integrator of ground speed loop
  }

  // Airspeed control loop (input: airspeed controlled, output: throttle controlled)
  float err_airspeed = (v_ctl_auto_airspeed_controlled - estimator_airspeed);
  v_ctl_auto_airspeed_sum_err += err_airspeed;
  BoundAbs(v_ctl_auto_airspeed_sum_err, V_CTL_AUTO_AIRSPEED_MAX_SUM_ERR);

  switch (v_ctl_airspeed_mode){
    case AS_MODE_VASSILLIS: //Vasilis
      controlled_throttle = (err_airspeed + v_ctl_auto_airspeed_sum_err * v_ctl_auto_airspeed_throttle_igain_vas) * v_ctl_auto_airspeed_throttle_pgain_vas;
    break;
    case AS_MODE_ZHAW_PITCH_SIMPLE: //AirSpeed Pitch Simple - ASPS
      v_ctl_pitch_of_vz = (err_airspeed + v_ctl_auto_airspeed_sum_err * v_ctl_auto_airspeed_pitch_igain_zps) * v_ctl_auto_airspeed_pitch_pgain_zps;
    break;
  }

  // Done, set outputs
  Bound(controlled_throttle, 0, V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE);
  f_throttle = controlled_throttle;
  nav_pitch = v_ctl_pitch_of_vz;
  v_ctl_throttle_setpoint = TRIM_UPPRZ(f_throttle * MAX_PPRZ);
  Bound(nav_pitch, V_CTL_AUTO_PITCH_MIN_PITCH, V_CTL_AUTO_PITCH_MAX_PITCH);
}

#endif // USE_AIRSPEED
}

/**
 * auto pitch inner loop
 * \brief computes a nav_pitch from a climb_setpoint given a fixed throttle
 */

#ifdef V_CTL_THROTTLE_SLEW_LIMITER
#define V_CTL_THROTTLE_SLEW (1./CONTROL_RATE/(V_CTL_THROTTLE_SLEW_LIMITER))
#endif

#ifndef V_CTL_THROTTLE_SLEW
#define V_CTL_THROTTLE_SLEW 1.
#endif
/** \brief Computes slewed throttle from throttle setpoint
    called at 20Hz
 */
void v_ctl_throttle_slew( void ) {
  pprz_t diff_throttle = v_ctl_throttle_setpoint - v_ctl_throttle_slewed;
  BoundAbs(diff_throttle, TRIM_PPRZ(V_CTL_THROTTLE_SLEW*MAX_PPRZ));
  v_ctl_throttle_slewed += diff_throttle;
}
