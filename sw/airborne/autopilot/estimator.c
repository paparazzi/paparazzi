/*
 * Paparazzi autopilot $Id$
 *  
 * Copyright (C) 2004  Pascal Brisset, Antoine Drouin
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

#include <inttypes.h>
#include <math.h>

#include "estimator.h"
#include "gps.h"
#include "pid.h"
#include "infrared.h"
#include "autopilot.h"


/* position in meters */
float estimator_x;
float estimator_y;
float estimator_z;

/* attitude in radian */
float estimator_phi;
float estimator_psi;
float estimator_theta;

/* speed in meters per second */
float estimator_x_dot;
float estimator_y_dot;
float estimator_z_dot;

/* rotational speed in radians per second */
float estimator_phi_dot;
float estimator_psi_dot;
float estimator_theta_dot;

/* flight time in seconds */
uint16_t estimator_flight_time; 
/* flight time in seconds */
float estimator_t; 

/* horizontal speed in module and dir */
float estimator_hspeed_mod;
float estimator_hspeed_dir;

float estimator_rad_of_ir, estimator_ir, estimator_rad;

#define EstimatorSetPos(x, y, z) { estimator_x = x; estimator_y = y; estimator_z = z; }
#define EstimatorSetAtt(phi, psi, theta) { estimator_phi = phi; estimator_psi = psi; estimator_theta = theta; }


// FIXME maybe vz = -climb for NED??
#define EstimatorSetSpeedCart(vx, vy, vz) { \
  estimator_vx = vx; \
  estimator_vy = vy; \
  estimator_vz = vz; \
}
//  estimator_hspeed_mod = sqrt( estimator_vx * estimator_vx + estimator_vy * estimator_vy);
//  estimator_hspeed_dir = atan2(estimator_vy, estimator_vx);


#define EstimatorSetSpeedPol(vhmod, vhdir, vz) { \
  estimator_hspeed_mod = vhmod; \
  estimator_hspeed_dir = vhdir; \
  estimator_z_dot = vz; \
}
//FIXME is this true ?? estimator_vx = estimator_hspeed_mod * cos(estimator_hspeed_dir);
//FIXME is this true ?? estimator_vy = estimator_hspeed_mod * sin(estimator_hspeed_dir);

#define EstimatorSetRotSpeed(phi_dot, psi_dot, theta_dot) { \
  estimator_phi_dot = phi_dot; \
  estimator_psi_dot = psi_dot; \
  estimator_theta_dot = theta_dot; \
}

inline void estimator_update_lls( void );

void estimator_init( void ) {

  EstimatorSetPos (0., 0., 0.);

  EstimatorSetAtt (0., 0., 0);

  EstimatorSetSpeedPol ( 0., 0., 0.);

  EstimatorSetRotSpeed (0., 0., 0.);

  estimator_flight_time = 0;

  estimator_rad_of_ir = ir_rad_of_ir;
}

#define EstimatorIrGainIsCorrect() (TRUE)

void estimator_update_state_infrared( void ) {
  float rad_of_ir = (ir_estim_mode == IR_ESTIM_MODE_ON && EstimatorIrGainIsCorrect()) ? 
    estimator_rad_of_ir : ir_rad_of_ir;
  estimator_phi  = rad_of_ir * ir_roll;

  estimator_theta = rad_of_ir * ir_pitch;
}

#define INIT_WEIGHT 100. /* The number of times the initial value has to be taken */
#define RHO 0.999 /* The higher, the slower the estimation is changing */

#define g 9.81


void estimator_update_ir_estim( void ) {
  static float last_hspeed_dir;
  static float last_t;
  static bool_t initialized = FALSE;
  static float sum_xy, sum_xx;

  if (initialized) {
    float dt = gps_ftow - last_t;
    if (dt > 0.1) { // Against division by zero
      float phi = (estimator_hspeed_dir - last_hspeed_dir); 
      NORM_RAD_ANGLE(phi);
      phi = phi/dt*NOMINAL_AIRSPEED/g; /* tan linearized */
      NORM_RAD_ANGLE(phi);
      estimator_ir = (float)ir_roll;
      estimator_rad = phi;
      float absphi = fabs(phi);
      if (absphi < 1.0 && absphi > 0.05 && (- ir_contrast/2 < ir_roll && ir_roll < ir_contrast/2)) {
	sum_xy = estimator_rad * estimator_ir + RHO * sum_xy;
	sum_xx = estimator_ir * estimator_ir + RHO * sum_xx;
#if defined IR_RAD_OF_IR_MIN_VALUE & defined IR_RAD_OF_IR_MAX_VALUE
	float result = sum_xy / sum_xx;
	if (result < IR_RAD_OF_IR_MIN_VALUE)
	  estimator_rad_of_ir = IR_RAD_OF_IR_MIN_VALUE;
	else if (result > IR_RAD_OF_IR_MAX_VALUE)
	  estimator_rad_of_ir = IR_RAD_OF_IR_MAX_VALUE;
	else
	  estimator_rad_of_ir = result;
#else
	  estimator_rad_of_ir = sum_xy / sum_xx;
#endif
      }
    } 
  } else {
    initialized = TRUE;
    float init_ir2 = ir_contrast;
    init_ir2 = init_ir2*init_ir2;
    sum_xy = INIT_WEIGHT * estimator_rad_of_ir * init_ir2;
    sum_xx = INIT_WEIGHT * init_ir2;
  }

  last_hspeed_dir = estimator_hspeed_dir;
  last_t = gps_ftow;
}


void estimator_update_state_gps( void ) {
  if (GPS_FIX_VALID(gps_mode)) {
    EstimatorSetPos(gps_east, gps_north, gps_falt);
    EstimatorSetSpeedPol(gps_fspeed, gps_fcourse, gps_fclimb);
    
    if (estimator_flight_time)
      estimator_update_ir_estim();
  }
}

void estimator_propagate_state( void ) {
  
}
