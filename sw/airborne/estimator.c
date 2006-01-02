/*
 * Paparazzi autopilot $Id$
 *  
 * Copyright (C) 2004-2005 Pascal Brisset, Antoine Drouin
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

/** \file estimator.c
 * \brief State estimate, fusioning sensors
 */

#include <inttypes.h>
#include <math.h>

#include "estimator.h"
//#include "gps.h"
//#include "infrared.h"
//#include "autopilot.h"
//#include "flight_plan.h"


/* position in meters */
float estimator_x;
float estimator_y;
float estimator_z;

float estimator_z_dot;

/* attitude in radian */
float estimator_phi;
float estimator_psi;
float estimator_theta;

/* flight time in seconds */
uint16_t estimator_flight_time; 
/* flight time in seconds */
float estimator_t;

/* horizontal speed in module and dir */
float estimator_hspeed_mod;
float estimator_hspeed_dir;


#define NORM_RAD_ANGLE2(x) { \
    while (x > 2 * M_PI) x -= 2 * M_PI; \
    while (x < 0 ) x += 2 * M_PI; \
  }


// FIXME maybe vz = -climb for NED??
#define EstimatorSetSpeedCart(vx, vy, vz) { \
  estimator_vx = vx; \
  estimator_vy = vy; \
  estimator_vz = vz; \
}
//  estimator_hspeed_mod = sqrt( estimator_vx * estimator_vx + estimator_vy * estimator_vy);
//  estimator_hspeed_dir = atan2(estimator_vy, estimator_vx);


//FIXME is this true ?? estimator_vx = estimator_hspeed_mod * cos(estimator_hspeed_dir);
//FIXME is this true ?? estimator_vy = estimator_hspeed_mod * sin(estimator_hspeed_dir);

inline void estimator_update_lls( void );

void estimator_init( void ) {

  EstimatorSetPos (0., 0., 0.);

  EstimatorSetAtt (0., 0., 0);

  EstimatorSetSpeedPol ( 0., 0., 0.);

  estimator_flight_time = 0;
}


#ifdef IMU_3DMG
#include "inter_mcu.h"
void estimator_update_state_3DMG( void ) {
  estimator_phi = from_fbw.euler[0];
  estimator_psi = from_fbw.euler[1];
  estimator_theta = from_fbw.euler[2];
}
#elif defined IMU_ANALOG && defined AHRS
#include "ahrs.h"
void estimator_update_state_ANALOG( void ) {
//ahrs.h is in NED but estimator in NWU if i remember
//perhaps this transform is not enough, i'm tired ;-) 
  estimator_phi = ahrs_euler[0];
  estimator_theta = -ahrs_euler[1];
  estimator_psi = -ahrs_euler[2];
}
#else //NO_IMU
#endif

void estimator_propagate_state( void ) {
  
}
