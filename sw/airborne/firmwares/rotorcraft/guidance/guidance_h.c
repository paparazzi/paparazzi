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

#define GUIDANCE_H_C
//#define GUIDANCE_H_USE_REF
#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "subsystems/ahrs.h"
#include "firmwares/rotorcraft/stabilization.h"
// #include "booz_fms.h" FIXME
#include "subsystems/ins.h"
#include "firmwares/rotorcraft/navigation.h"

#include "generated/airframe.h"

uint8_t guidance_h_mode;

struct Int32Vect2 guidance_h_pos_sp;
int32_t           guidance_h_psi_sp;
struct Int32Vect2 guidance_h_pos_ref;
struct Int32Vect2 guidance_h_speed_ref;
struct Int32Vect2 guidance_h_accel_ref;

struct Int32Vect2 guidance_h_pos_err;
struct Int32Vect2 guidance_h_speed_err;
struct Int32Vect2 guidance_h_pos_err_sum;
struct Int32Vect2 guidance_h_nav_err;

struct Int32Eulers guidance_h_rc_sp;
struct Int32Vect2  guidance_h_command_earth;
struct Int32Vect2  guidance_h_stick_earth_sp;
struct Int32Eulers guidance_h_command_body;

int32_t guidance_h_pgain;
int32_t guidance_h_dgain;
int32_t guidance_h_igain;
int32_t guidance_h_ngain;
int32_t guidance_h_again;

#ifndef GUIDANCE_H_NGAIN
#define GUIDANCE_H_NGAIN 0
#endif

#ifndef GUIDANCE_H_AGAIN
#define GUIDANCE_H_AGAIN 0
#endif

static inline void guidance_h_hover_run(void);
static inline void guidance_h_nav_run(bool_t in_flight);
static inline void guidance_h_hover_enter(void);
static inline void guidance_h_nav_enter(void);

#define GuidanceHSetRef(_pos, _speed, _accel) { \
    b2_gh_set_ref(_pos, _speed, _accel); \
    VECT2_COPY(guidance_h_pos_ref,   _pos); \
    VECT2_COPY(guidance_h_speed_ref, _speed); \
    VECT2_COPY(guidance_h_accel_ref, _accel); \
  }


void guidance_h_init(void) {

  guidance_h_mode = GUIDANCE_H_MODE_KILL;
  guidance_h_psi_sp = 0;
  INT_VECT2_ZERO(guidance_h_pos_sp);
  INT_VECT2_ZERO(guidance_h_pos_err_sum);
  INT_EULERS_ZERO(guidance_h_rc_sp);
  INT_EULERS_ZERO(guidance_h_command_body);
  guidance_h_pgain = GUIDANCE_H_PGAIN;
  guidance_h_igain = GUIDANCE_H_IGAIN;
  guidance_h_dgain = GUIDANCE_H_DGAIN;
  guidance_h_ngain = GUIDANCE_H_NGAIN;
  guidance_h_again = GUIDANCE_H_AGAIN;

}


void guidance_h_mode_changed(uint8_t new_mode) {
  if (new_mode == guidance_h_mode)
    return;

  switch ( guidance_h_mode ) {
	//      case GUIDANCE_H_MODE_RATE:
	//	stabilization_rate_exit();
	//	break;
  default:
    break;
  }

  switch (new_mode) {

  case GUIDANCE_H_MODE_RATE:
    stabilization_rate_enter();
    break;

  case GUIDANCE_H_MODE_ATTITUDE:
    stabilization_attitude_enter();
    break;

  case GUIDANCE_H_MODE_HOVER:
    guidance_h_hover_enter();
    break;

  case GUIDANCE_H_MODE_NAV:
    guidance_h_nav_enter();
    break;
  default:
    break;
  }

  guidance_h_mode = new_mode;

}


void guidance_h_read_rc(bool_t  in_flight) {

  switch ( guidance_h_mode ) {

  case GUIDANCE_H_MODE_RATE:
    stabilization_rate_read_rc();
    break;

  case GUIDANCE_H_MODE_ATTITUDE:
    stabilization_attitude_read_rc(in_flight);
    break;

  case GUIDANCE_H_MODE_HOVER:
    STABILIZATION_ATTITUDE_READ_RC(guidance_h_rc_sp, in_flight);
    break;

  case GUIDANCE_H_MODE_NAV:
    if (radio_control.status == RC_OK) {
      STABILIZATION_ATTITUDE_READ_RC(guidance_h_rc_sp, in_flight);
      guidance_h_rc_sp.psi = 0;
    }
    else {
      INT_EULERS_ZERO(guidance_h_rc_sp);
    }
    break;
  default:
    break;
  }

}


void guidance_h_run(bool_t  in_flight) {
  switch ( guidance_h_mode ) {

  case GUIDANCE_H_MODE_RATE:
    stabilization_rate_run(in_flight);
    break;

  case GUIDANCE_H_MODE_ATTITUDE:
    stabilization_attitude_run(in_flight);
    break;

  case GUIDANCE_H_MODE_HOVER:
    guidance_h_hover_run();
    stabilization_attitude_run(in_flight);
    break;

  case GUIDANCE_H_MODE_NAV:
    {
      if (!in_flight) guidance_h_nav_enter();

      if (horizontal_mode == HORIZONTAL_MODE_ATTITUDE) {
#ifndef STABILISATION_ATTITUDE_TYPE_FLOAT
        stab_att_sp_euler.phi = nav_roll << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
        stab_att_sp_euler.theta = nav_pitch << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
#endif
      }
      else {
        INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_carrot);
#ifdef GUIDANCE_H_USE_REF
        b2_gh_update_ref_from_pos_sp(guidance_h_pos_sp);
#endif
#ifndef STABILISATION_ATTITUDE_TYPE_FLOAT
        guidance_h_psi_sp = (nav_heading << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
#endif
        //guidance_h_hover_run();
        guidance_h_nav_run(in_flight);
      }
      stabilization_attitude_run(in_flight);
      break;
    }
  default:
    break;
  }



}

#define MAX_POS_ERR   POS_BFP_OF_REAL(16.)
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)
#define MAX_POS_ERR_SUM ((int32_t)(MAX_POS_ERR)<< 12)

// 15 degres
//#define MAX_BANK (65536)
#define MAX_BANK (98000)

__attribute__ ((always_inline)) static inline void  guidance_h_hover_run(void) {

  /* compute position error    */
  VECT2_DIFF(guidance_h_pos_err, ins_ltp_pos, guidance_h_pos_sp);
  /* saturate it               */
  VECT2_STRIM(guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  VECT2_COPY(guidance_h_speed_err, ins_ltp_speed);
  /* saturate it               */
  VECT2_STRIM(guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

  /* update pos error integral */
  VECT2_ADD(guidance_h_pos_err_sum, guidance_h_pos_err);
  /* saturate it               */
  VECT2_STRIM(guidance_h_pos_err_sum, -MAX_POS_ERR_SUM, MAX_POS_ERR_SUM);

  /* run PID */
  // cmd_earth < 15.17
  guidance_h_command_earth.x = (guidance_h_pgain<<1)  * guidance_h_pos_err.x +
                                     guidance_h_dgain * (guidance_h_speed_err.x>>9) +
                                      guidance_h_igain * (guidance_h_pos_err_sum.x >> 12);
  guidance_h_command_earth.y = (guidance_h_pgain<<1)  * guidance_h_pos_err.y +
                                     guidance_h_dgain *( guidance_h_speed_err.y>>9) +
		                      guidance_h_igain * (guidance_h_pos_err_sum.y >> 12);

  VECT2_STRIM(guidance_h_command_earth, -MAX_BANK, MAX_BANK);

  /* Rotate to body frame */
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, ahrs.ltp_to_body_euler.psi);
  PPRZ_ITRIG_COS(c_psi, ahrs.ltp_to_body_euler.psi);


  // INT32_TRIG_FRAC - 2: 100mm erreur, gain 100 -> 10000 command | 2 degres = 36000, so multiply by 4
  guidance_h_command_body.phi =
      ( - s_psi * guidance_h_command_earth.x + c_psi * guidance_h_command_earth.y)
    >> (INT32_TRIG_FRAC - 2);
  guidance_h_command_body.theta =
    - ( c_psi * guidance_h_command_earth.x + s_psi * guidance_h_command_earth.y)
    >> (INT32_TRIG_FRAC - 2);


  guidance_h_command_body.phi   += guidance_h_rc_sp.phi;
  guidance_h_command_body.theta += guidance_h_rc_sp.theta;
  guidance_h_command_body.psi    = guidance_h_psi_sp + guidance_h_rc_sp.psi;
#ifndef STABILISATION_ATTITUDE_TYPE_FLOAT
  ANGLE_REF_NORMALIZE(guidance_h_command_body.psi);
#endif /* STABILISATION_ATTITUDE_TYPE_FLOAT */

  EULERS_COPY(stab_att_sp_euler, guidance_h_command_body);

}

// 20 degres -> 367002 (0.35 << 20)
#define NAV_MAX_BANK BFP_OF_REAL(0.35,REF_ANGLE_FRAC)
#define HOLD_DISTANCE POS_BFP_OF_REAL(10.)

__attribute__ ((always_inline)) static inline void  guidance_h_nav_run(bool_t in_flight) {

  /* convert our reference to generic representation */
#ifdef GUIDANCE_H_USE_REF
  INT32_VECT2_RSHIFT(guidance_h_pos_ref,   b2_gh_pos_ref,   (B2_GH_POS_REF_FRAC - INT32_POS_FRAC));
  INT32_VECT2_LSHIFT(guidance_h_speed_ref, b2_gh_speed_ref, (INT32_SPEED_FRAC - B2_GH_SPEED_REF_FRAC));
  INT32_VECT2_LSHIFT(guidance_h_accel_ref, b2_gh_accel_ref, (INT32_ACCEL_FRAC - B2_GH_ACCEL_REF_FRAC));
#else
  VECT2_COPY(guidance_h_pos_ref, guidance_h_pos_sp);
  INT_VECT2_ZERO(guidance_h_speed_ref);
  INT_VECT2_ZERO(guidance_h_accel_ref);
#endif

  /* compute position error    */
  VECT2_DIFF(guidance_h_pos_err, ins_ltp_pos, guidance_h_pos_ref);
  /* saturate it               */
  VECT2_STRIM(guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  //VECT2_COPY(guidance_h_speed_err, ins_ltp_speed);
  VECT2_DIFF(guidance_h_speed_err, ins_ltp_speed, guidance_h_speed_ref);
  /* saturate it               */
  VECT2_STRIM(guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

  int32_t dist;
  INT32_VECT2_NORM(dist, guidance_h_pos_err);
  if ( dist < HOLD_DISTANCE ) { // Hold position
    /* update pos error integral */
    VECT2_ADD(guidance_h_pos_err_sum, guidance_h_pos_err);
    /* saturate it               */
    VECT2_STRIM(guidance_h_pos_err_sum, -MAX_POS_ERR_SUM, MAX_POS_ERR_SUM);
  }
  else { // Tracking algorithm, no integral
    int32_t vect_prod = 0;
    int32_t scal_prod = ins_ltp_speed.x * guidance_h_pos_err.x + ins_ltp_speed.y * guidance_h_pos_err.y;
    // compute vectorial product only if angle < pi/2 (scalar product > 0)
    if (scal_prod >= 0) {
      vect_prod = ((ins_ltp_speed.x * guidance_h_pos_err.y) >> (INT32_POS_FRAC + INT32_SPEED_FRAC - 10))
                - ((ins_ltp_speed.y * guidance_h_pos_err.x) >> (INT32_POS_FRAC + INT32_SPEED_FRAC - 10));
    }
    // multiply by vector orthogonal to speed
    VECT2_ASSIGN(guidance_h_nav_err,
        vect_prod * (-ins_ltp_speed.y),
        vect_prod * ins_ltp_speed.x);
    // divide by 2 times dist ( >> 16 )
    VECT2_SDIV(guidance_h_nav_err, guidance_h_nav_err, dist*dist);
    // *2 ??
  }
  if (!in_flight) { INT_VECT2_ZERO(guidance_h_pos_err_sum); }

  /* run PID */
  guidance_h_command_earth.x =
    guidance_h_pgain * (guidance_h_pos_err.x << (10 - INT32_POS_FRAC)) +
    guidance_h_dgain * (guidance_h_speed_err.x >> (INT32_SPEED_FRAC - 10)) +
    guidance_h_igain * (guidance_h_pos_err_sum.x >> (12 + INT32_POS_FRAC - 10)) +
    guidance_h_ngain * guidance_h_nav_err.x +
    guidance_h_again * guidance_h_accel_ref.x; /* feedforward gain */
  guidance_h_command_earth.y =
    guidance_h_pgain * (guidance_h_pos_err.y << (10 - INT32_POS_FRAC)) +
    guidance_h_dgain * (guidance_h_speed_err.y >> (INT32_SPEED_FRAC - 10)) +
    guidance_h_igain * (guidance_h_pos_err_sum.y >> (12 + INT32_POS_FRAC - 10)) +
    guidance_h_ngain * guidance_h_nav_err.y +
    guidance_h_again * guidance_h_accel_ref.y; /* feedforward gain */

  VECT2_STRIM(guidance_h_command_earth, -NAV_MAX_BANK, NAV_MAX_BANK);
  INT32_VECT2_RSHIFT(guidance_h_command_earth, guidance_h_command_earth, REF_ANGLE_FRAC - 16); // Reduice to 16 for trigo operation

  /* Rotate to body frame */
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, ahrs.ltp_to_body_euler.psi);
  PPRZ_ITRIG_COS(c_psi, ahrs.ltp_to_body_euler.psi);

  // Restore angle ref resolution after rotation
  guidance_h_command_body.phi =
      ( - s_psi * guidance_h_command_earth.x + c_psi * guidance_h_command_earth.y) >> (INT32_TRIG_FRAC - (REF_ANGLE_FRAC - 16));
  guidance_h_command_body.theta =
    - ( c_psi * guidance_h_command_earth.x + s_psi * guidance_h_command_earth.y) >> (INT32_TRIG_FRAC - (REF_ANGLE_FRAC - 16));

  // Add RC setpoint
  guidance_h_command_body.phi   += guidance_h_rc_sp.phi;
  guidance_h_command_body.theta += guidance_h_rc_sp.theta;
  guidance_h_command_body.psi    = guidance_h_psi_sp + guidance_h_rc_sp.psi;
  ANGLE_REF_NORMALIZE(guidance_h_command_body.psi);

  // Set attitude setpoint
  EULERS_COPY(stab_att_sp_euler, guidance_h_command_body);

}

__attribute__ ((always_inline)) static inline void guidance_h_hover_enter(void) {

  VECT2_COPY(guidance_h_pos_sp, ins_ltp_pos);

  STABILIZATION_ATTITUDE_RESET_PSI_REF( guidance_h_rc_sp );

  INT_VECT2_ZERO(guidance_h_pos_err_sum);

}

__attribute__ ((always_inline)) static inline void guidance_h_nav_enter(void) {

  INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_carrot);
  struct Int32Vect2 pos,speed,zero;
  INT_VECT2_ZERO(zero);
  VECT2_COPY(pos, ins_ltp_pos);
  VECT2_COPY(speed, ins_ltp_speed);
  GuidanceHSetRef(pos, speed, zero);

  struct Int32Eulers tmp_sp;
  STABILIZATION_ATTITUDE_RESET_PSI_REF( tmp_sp );
  guidance_h_psi_sp = tmp_sp.psi;
#ifndef STABILISATION_ATTITUDE_TYPE_FLOAT
  nav_heading = (guidance_h_psi_sp >> (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
#endif /* STABILISATION_ATTITUDE_TYPE_FLOAT */
  guidance_h_rc_sp.psi = 0;

  INT_VECT2_ZERO(guidance_h_pos_err_sum);

}
