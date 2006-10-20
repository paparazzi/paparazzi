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

#include "fw_v_ctl.h"
#include "estimator.h"
#include "nav.h"
#include "airframe.h"

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
float v_ctl_auto_throttle_climb_throttle_increment;
float v_ctl_auto_throttle_pgain;
float v_ctl_auto_throttle_igain;
float v_ctl_auto_throttle_sum_err;
#define V_CTL_AUTO_THROTTLE_MAX_SUM_ERR 150
float v_ctl_auto_throttle_pitch_of_vz_pgain;

/* "auto pitch" inner loop parameters */
float v_ctl_auto_pitch_pgain;
float v_ctl_auto_pitch_igain;
float v_ctl_auto_pitch_sum_err;
#define V_CTL_AUTO_PITCH_MAX_SUM_ERR 100

pprz_t v_ctl_throttle_setpoint;

inline static void v_ctl_climb_auto_throttle_loop( void );
inline static void v_ctl_climb_auto_pitch_loop( void );

void v_ctl_init( void ) {
  /* outer loop */
  v_ctl_altitude_setpoint = 0.;
  v_ctl_altitude_pre_climb = 0.;
  v_ctl_altitude_pgain = V_CTL_ALTITUDE_PGAIN;
  v_ctl_altitude_error = 0.;

  /* inner loops */
  v_ctl_climb_setpoint = 0.;
  v_ctl_climb_mode = V_CTL_CLIMB_MODE_AUTO_THROTTLE;
#ifdef AGR_CLIMB
  v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_STANDARD;
#endif

  /* "auto throttle" inner loop parameters */
  v_ctl_auto_throttle_cruise_throttle = V_CTL_AUTO_THROTTLE_CRUISE_THROTTLE;
  v_ctl_auto_throttle_climb_throttle_increment = 
    V_CTL_AUTO_THROTTLE_CLIMB_THROTTLE_INCREMENT;
  v_ctl_auto_throttle_pgain = V_CTL_AUTO_THROTTLE_PGAIN;
  v_ctl_auto_throttle_igain = V_CTL_AUTO_THROTTLE_IGAIN;
  v_ctl_auto_throttle_sum_err = 0.;
  v_ctl_auto_throttle_pitch_of_vz_pgain = V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_PGAIN; 

  /* "auto pitch" inner loop parameters */
  v_ctl_auto_pitch_pgain = V_CTL_AUTO_PITCH_PGAIN;
  v_ctl_auto_pitch_igain = V_CTL_AUTO_PITCH_IGAIN;
  v_ctl_auto_pitch_sum_err = 0.;

  v_ctl_throttle_setpoint = 0;
}

/** 
 * outer loop
 * \brief Computes v_ctl_climb_setpoint and sets v_ctl_auto_throttle_submode 
 */
void v_ctl_altitude_loop( void ) {
  v_ctl_altitude_error = estimator_z - v_ctl_altitude_setpoint;
  v_ctl_climb_setpoint = v_ctl_altitude_pgain * v_ctl_altitude_error
    + v_ctl_altitude_pre_climb;
  BoundAbs(v_ctl_climb_setpoint, V_CTL_ALTITUDE_MAX_CLIMB);

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
  switch (v_ctl_climb_mode) {
  case V_CTL_CLIMB_MODE_AUTO_THROTTLE:
    v_ctl_climb_auto_throttle_loop();
    break;
  case V_CTL_CLIMB_MODE_AUTO_PITCH:
    v_ctl_climb_auto_pitch_loop();
    break;
  }
}

/** 
 * auto throttle inner loop
 * \brief 
 */
inline static void v_ctl_climb_auto_throttle_loop(void) {
  float f_throttle = 0;
  float err  = estimator_z_dot - v_ctl_climb_setpoint;
  float controlled_throttle = v_ctl_auto_throttle_cruise_throttle 
    + v_ctl_auto_throttle_climb_throttle_increment * v_ctl_climb_setpoint 
    + v_ctl_auto_throttle_pgain * 
    (err + v_ctl_auto_throttle_igain * v_ctl_auto_throttle_sum_err);
  
  /* pitch pre-command */
  float v_ctl_pitch_of_vz = v_ctl_climb_setpoint * v_ctl_auto_throttle_pitch_of_vz_pgain;

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
    float ratio = (fabs(v_ctl_altitude_error) - AGR_BLEND_END) 
      / (AGR_BLEND_START - AGR_BLEND_END);
    f_throttle = (1-ratio) * controlled_throttle;
    nav_pitch = (1-ratio) * v_ctl_pitch_of_vz;
    v_ctl_auto_throttle_sum_err += (1-ratio) * err;
    if (v_ctl_altitude_error < 0) {
      f_throttle +=  ratio * AGR_CLIMB_THROTTLE;
      nav_pitch += ratio * AGR_CLIMB_PITCH;
    } else {
      f_throttle += ratio * AGR_DESCENT_THROTTLE;
      nav_pitch += ratio * AGR_DESCENT_PITCH;
    }
    break;
  }
    
  case V_CTL_AUTO_THROTTLE_STANDARD:
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


/** 
 * auto pitch inner loop
 * \brief computes a nav_pitch from a climb_setpoint given a fixed throttle
 */
inline static void v_ctl_climb_auto_pitch_loop(void) {
  float err  = estimator_z_dot - v_ctl_climb_setpoint;
  v_ctl_throttle_setpoint = nav_desired_gaz;
  v_ctl_auto_pitch_sum_err += err;
  BoundAbs(v_ctl_auto_pitch_sum_err, V_CTL_AUTO_PITCH_MAX_SUM_ERR);
  nav_pitch = v_ctl_auto_pitch_pgain * 
    (err + v_ctl_auto_pitch_igain * v_ctl_auto_pitch_sum_err);
  Bound(nav_pitch, V_CTL_AUTO_PITCH_MIN_PITCH, V_CTL_AUTO_PITCH_MAX_PITCH);
}

#ifndef V_CTL_THROTTLE_SLEW
#define V_CTL_THROTTLE_SLEW 1.
#endif
void v_ctl_throttle_slew( void ) {
  static pprz_t last_throttle;
  pprz_t diff_throttle = v_ctl_throttle_setpoint - last_throttle;
  BoundAbs(diff_throttle, TRIM_PPRZ(V_CTL_THROTTLE_SLEW*MAX_PPRZ));
  v_ctl_throttle_setpoint = last_throttle + diff_throttle;
  last_throttle = v_ctl_throttle_setpoint;
}



