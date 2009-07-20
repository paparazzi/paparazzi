/*
 * $Id$
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

#include "booz2_guidance_h.h"

#include "booz_ahrs.h"
#include "booz_stabilization.h"
#include "booz2_fms.h"
#include "booz2_ins.h"
#include "booz2_navigation.h"

#include "airframe.h"

uint8_t booz2_guidance_h_mode;

struct Int32Vect2 booz2_guidance_h_pos_sp;
int32_t            booz2_guidance_h_psi_sp;

struct Int32Vect2 booz2_guidance_h_pos_err;
struct Int32Vect2 booz2_guidance_h_speed_err;
struct Int32Vect2 booz2_guidance_h_pos_err_sum;

struct Int32Eulers booz2_guidance_h_rc_sp;
struct Int32Vect2  booz2_guidance_h_command_earth;
struct Int32Vect2  booz2_guidance_h_stick_earth_sp;
struct Int32Eulers booz2_guidance_h_command_body;

int32_t booz2_guidance_h_pgain;
int32_t booz2_guidance_h_dgain;
int32_t booz2_guidance_h_igain;


static inline void booz2_guidance_h_hover_run(void);
static inline void booz2_guidance_h_hover_enter(void);
static inline void booz2_guidance_h_nav_enter(void);


void booz2_guidance_h_init(void) {

  booz2_guidance_h_mode = BOOZ2_GUIDANCE_H_MODE_KILL;
  booz2_guidance_h_psi_sp = 0;
  INT_VECT2_ZERO(booz2_guidance_h_pos_sp);
  INT_VECT2_ZERO(booz2_guidance_h_pos_err_sum);
  INT_EULERS_ZERO(booz2_guidance_h_rc_sp);
  INT_EULERS_ZERO(booz2_guidance_h_command_body);
  booz2_guidance_h_pgain = BOOZ2_GUIDANCE_H_PGAIN;
  booz2_guidance_h_igain = BOOZ2_GUIDANCE_H_IGAIN;
  booz2_guidance_h_dgain = BOOZ2_GUIDANCE_H_DGAIN;

}


void booz2_guidance_h_mode_changed(uint8_t new_mode) {
  if (new_mode == booz2_guidance_h_mode)
    return;

  switch ( booz2_guidance_h_mode ) {
	//      case BOOZ2_GUIDANCE_H_MODE_RATE:
	//	booz_stabilization_rate_exit();
	//	break;
  }
   
  switch (new_mode) {

  case BOOZ2_GUIDANCE_H_MODE_ATTITUDE:
    booz_stabilization_attitude_enter();
    break;
    
  case BOOZ2_GUIDANCE_H_MODE_HOVER:
    booz2_guidance_h_hover_enter();
    break;

  case BOOZ2_GUIDANCE_H_MODE_NAV:
    booz2_guidance_h_nav_enter();
    break;

  }

  booz2_guidance_h_mode = new_mode;
  
}

void booz2_guidance_h_read_rc(bool_t  in_flight) {
  
  switch ( booz2_guidance_h_mode ) {

  case BOOZ2_GUIDANCE_H_MODE_RATE:
    booz_stabilization_rate_read_rc();
    break;
    
  case BOOZ2_GUIDANCE_H_MODE_ATTITUDE:
    booz_stabilization_attitude_read_rc(in_flight);
#ifdef USE_FMS
    if (booz_fms_on)
      BOOZ_STABILIZATION_ATTITUDE_ADD_SP(booz_fms_input.h_sp.attitude);
#endif
    break;
  
  case BOOZ2_GUIDANCE_H_MODE_HOVER:
#ifdef USE_FMS
    if (booz_fms_on && booz_fms_input.h_mode >= BOOZ2_GUIDANCE_H_MODE_HOVER)
      BOOZ2_FMS_SET_POS_SP(booz2_guidance_h_pos_sp,booz_stabilization_att_sp.psi);
#endif
    BOOZ_STABILIZATION_ATTITUDE_READ_RC(booz2_guidance_h_rc_sp, in_flight);
    break;
  
  case BOOZ2_GUIDANCE_H_MODE_NAV:
    BOOZ_STABILIZATION_ATTITUDE_READ_RC(booz2_guidance_h_rc_sp, in_flight);
    booz2_guidance_h_rc_sp.psi = 0;
    break;
  }

}


void booz2_guidance_h_run(bool_t  in_flight) {
  switch ( booz2_guidance_h_mode ) {

  case BOOZ2_GUIDANCE_H_MODE_RATE:
    booz_stabilization_rate_run();
    break;

  case BOOZ2_GUIDANCE_H_MODE_ATTITUDE:
    booz_stabilization_attitude_run(in_flight);
    break;
    
  case BOOZ2_GUIDANCE_H_MODE_HOVER:
    booz2_guidance_h_hover_run();
    booz_stabilization_attitude_run(in_flight);
    break;
    
  case BOOZ2_GUIDANCE_H_MODE_NAV:
    {
      if (horizontal_mode == HORIZONTAL_MODE_ATTITUDE) {
        booz_stabilization_att_sp.phi = 0;
        booz_stabilization_att_sp.theta = 0;
      }
      else {
        INT32_VECT2_NED_OF_ENU(booz2_guidance_h_pos_sp, booz2_navigation_carrot);
        booz2_guidance_h_psi_sp = (nav_heading << (ANGLE_REF_RES - INT32_ANGLE_FRAC));
        booz2_guidance_h_hover_run();
      }
      booz_stabilization_attitude_run(in_flight);
      break;
    }
    
  }



}

#define MAX_POS_ERR   POS_BFP_OF_REAL(16.)
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)
#define MAX_POS_ERR_SUM ((int32_t)(MAX_POS_ERR)<< 12)

// 15 degres
//#define MAX_BANK (65536)
#define MAX_BANK (98000)

static inline void  booz2_guidance_h_hover_run(void) {

  /* compute position error    */
  VECT2_DIFF(booz2_guidance_h_pos_err, booz_ins_ltp_pos, booz2_guidance_h_pos_sp);
  /* saturate it               */
  VECT2_STRIM(booz2_guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  VECT2_COPY(booz2_guidance_h_speed_err, booz_ins_ltp_speed);
  /* saturate it               */
  VECT2_STRIM(booz2_guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);
  
  /* update pos error integral */
  VECT2_ADD(booz2_guidance_h_pos_err_sum, booz2_guidance_h_pos_err);
  /* saturate it               */
  VECT2_STRIM(booz2_guidance_h_pos_err_sum, -MAX_POS_ERR_SUM, MAX_POS_ERR_SUM);
		    
  /* run PID */
  // cmd_earth < 15.17
  booz2_guidance_h_command_earth.x = (booz2_guidance_h_pgain<<1)  * booz2_guidance_h_pos_err.x +
                                     booz2_guidance_h_dgain * (booz2_guidance_h_speed_err.x>>9) +
                                      booz2_guidance_h_igain * (booz2_guidance_h_pos_err_sum.x >> 12); 
  booz2_guidance_h_command_earth.y = (booz2_guidance_h_pgain<<1)  * booz2_guidance_h_pos_err.y +
                                     booz2_guidance_h_dgain *( booz2_guidance_h_speed_err.y>>9) +
		                      booz2_guidance_h_igain * (booz2_guidance_h_pos_err_sum.y >> 12); 

  VECT2_STRIM(booz2_guidance_h_command_earth, -MAX_BANK, MAX_BANK);

  /* Rotate to body frame */
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, booz_ahrs.ltp_to_body_euler.psi);	
  PPRZ_ITRIG_COS(c_psi, booz_ahrs.ltp_to_body_euler.psi);	


  // INT32_TRIG_FRAC - 2: 100mm erreur, gain 100 -> 10000 command | 2 degres = 36000, so multiply by 4
  booz2_guidance_h_command_body.phi = 
      ( - s_psi * booz2_guidance_h_command_earth.x + c_psi * booz2_guidance_h_command_earth.y)
    >> (INT32_TRIG_FRAC - 2);
  booz2_guidance_h_command_body.theta = 
    - ( c_psi * booz2_guidance_h_command_earth.x + s_psi * booz2_guidance_h_command_earth.y)
    >> (INT32_TRIG_FRAC - 2);


  booz2_guidance_h_command_body.phi   += booz2_guidance_h_rc_sp.phi;
  booz2_guidance_h_command_body.theta += booz2_guidance_h_rc_sp.theta;
  booz2_guidance_h_command_body.psi    = booz2_guidance_h_psi_sp + booz2_guidance_h_rc_sp.psi;
  ANGLE_REF_NORMALIZE(booz2_guidance_h_command_body.psi);

  EULERS_COPY(booz_stabilization_att_sp, booz2_guidance_h_command_body);

}

static inline void booz2_guidance_h_hover_enter(void) {

  VECT2_COPY(booz2_guidance_h_pos_sp, booz_ins_ltp_pos);

  BOOZ_STABILIZATION_ATTITUDE_RESET_PSI_REF( booz2_guidance_h_rc_sp );

  INT_VECT2_ZERO(booz2_guidance_h_pos_err_sum);

#ifdef USE_FMS
  BOOZ2_FMS_POS_INIT(booz2_guidance_h_pos_sp,booz2_guidance_h_rc_sp.psi);
#endif
}

static inline void booz2_guidance_h_nav_enter(void) {

  INT32_VECT2_NED_OF_ENU(booz2_guidance_h_pos_sp, booz2_navigation_carrot);

  struct Int32Eulers tmp_sp;
  BOOZ_STABILIZATION_ATTITUDE_RESET_PSI_REF( tmp_sp );
  booz2_guidance_h_psi_sp = tmp_sp.psi;
  nav_heading = (booz2_guidance_h_psi_sp >> (ANGLE_REF_RES - INT32_ANGLE_FRAC));
  booz2_guidance_h_rc_sp.psi = 0;

  INT_VECT2_ZERO(booz2_guidance_h_pos_err_sum);

}

