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
#include "flight_plan.h"


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

float estimator_rad_of_ir, estimator_ir, estimator_rad;


/* array of horizon angles computed for a given height (at flight_plan.h generation) */

int8_t angles[NB_HEIGHTS] = HEIGHTS;


/* (aircraft axis, ir axis) angle */

#define aircraft_ir_angle ( M_PI / 2)



#define height_index_coeff (NB_HEIGHTS / ( 2 * M_PI ))

#define NORM_RAD_ANGLE2(x) { \
    while (x > 2 * M_PI) x -= 2 * M_PI; \
    while (x < 0 ) x += 2 * M_PI; \
  }

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

inline void estimator_update_lls( void );

void estimator_init( void ) {

  EstimatorSetPos (0., 0., 0.);

  EstimatorSetAtt (0., 0., 0);

  EstimatorSetSpeedPol ( 0., 0., 0.);

  estimator_flight_time = 0;

  estimator_rad_of_ir = ir_rad_of_ir;
}

#define EstimatorIrGainIsCorrect() (TRUE)

#ifdef SECTION_IMU_3DMG
#include "link_fbw.h"
void estimator_update_state_3DMG( void ) {
  estimator_phi = from_fbw.euler[0];
  estimator_psi = from_fbw.euler[1];
  estimator_theta = from_fbw.euler[2];
}
//#elif defined SECTION_IMU_ANALOG
//#include "ahrs.h"
//void estimator_update_state_ANALOG( void ) {
//  estimator_phi = ahrs_euler[0];
//  estimator_theta = ahrs_euler[1];
//  estimator_psi = ahrs_euler[2];
//}
#else //NO_IMU

float ir_roll_neutral  = RadOfDeg(IR_ROLL_NEUTRAL_DEFAULT);
/** Initialized to \a IR_PITCH_NEUTRAL_DEFAULT.
 *  Changed with @@@@@ EST-CE QUE CA CHANGE @@@@@ */
float ir_pitch_neutral = RadOfDeg(IR_PITCH_NEUTRAL_DEFAULT);

void estimator_update_state_infrared( void ) {
  float rad_of_ir = (ir_estim_mode == IR_ESTIM_MODE_ON && EstimatorIrGainIsCorrect()) ? 
    estimator_rad_of_ir : ir_rad_of_ir;

  /* phi correction because of the relief effect on ir data */

  /*** int8_t index_left, index_right;

  float angle =  gps_fcourse - aircraft_ir_angle;

  NORM_RAD_ANGLE2(angle);

  index_left = angle * height_index_coeff ;

  angle =  gps_fcourse + aircraft_ir_angle;

  NORM_RAD_ANGLE2(angle);

  index_right = angle * height_index_coeff ;

  int8_t angle_left = angles[index_left]; 

  int8_t angle_right = angles[index_right];


  float correction =  angle_left - angle_right;

  printf(" degres_left %d angle_left %d degres_right %d angle_right %d correction %.2f \n", (index_left*15), angle_left, (index_right*15), angle_right, correction); ***/
   
  estimator_phi  = rad_of_ir * ir_roll - ir_roll_neutral /*** + RadOfDeg( correction ) ***/;

  estimator_theta = rad_of_ir * ir_pitch - ir_pitch_neutral;
}
#endif

#define INIT_WEIGHT 100. /* The number of times the initial value has to be taken */
#define RHO 0.995 /* The higher, the slower the estimation is changing */

#define g 9.81


void estimator_update_ir_estim( void ) {
  static float last_hspeed_dir;
  static uint32_t last_t; /* ms */
  static bool_t initialized = FALSE;
  static float sum_xy, sum_xx;

  if (initialized) {
    float dt = (float)(gps_itow - last_t) / 1e3;
    if (dt > 0.1) { // Against division by zero
      float dpsi = (estimator_hspeed_dir - last_hspeed_dir); 
      NORM_RAD_ANGLE(dpsi);
      estimator_rad = dpsi/dt*NOMINAL_AIRSPEED/g; /* tan linearized */
      NORM_RAD_ANGLE(estimator_rad);
      estimator_ir = (float)ir_roll;
      float absphi = fabs(estimator_rad);
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
  last_t = gps_itow;
}


void estimator_update_state_gps( void ) {
  if (GPS_FIX_VALID(gps_mode)) {
    float gps_east = gps_utm_east / 100. - NAV_UTM_EAST0;
    float gps_north = gps_utm_north / 100. - NAV_UTM_NORTH0;
    float falt = gps_alt / 100.;
    EstimatorSetPos(gps_east, gps_north, falt);
    float fspeed = gps_gspeed / 100.;
    float fclimb = gps_climb / 100.;
    float fcourse = RadOfDeg(gps_course / 10.);
    EstimatorSetSpeedPol(fspeed, fcourse, fclimb);
    
    if (estimator_flight_time)
      estimator_update_ir_estim();
  }
}

void estimator_propagate_state( void ) {
  
}
