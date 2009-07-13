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

#include "booz2_stabilization_attitude.h"

#include "booz2_stabilization_attitude_ref_traj_euler.h"
#include "booz2_stabilization.h"
#include "booz_ahrs.h"
#include "airframe.h"
#include "radio_control.h"

struct booz_ieuler booz_stabilization_att_sp;

struct booz_ieuler booz_stabilization_att_ref;
struct booz_ivect  booz_stabilization_rate_ref;
struct booz_ivect  booz_stabilization_accel_ref;

struct booz_ivect  booz_stabilization_pgain;
struct booz_ivect  booz_stabilization_dgain;
struct booz_ivect  booz_stabilization_ddgain;
struct booz_ivect  booz_stabilization_igain;
struct booz_ieuler booz_stabilization_att_sum_err;

static inline void booz_stabilization_update_ref(void);


void booz2_stabilization_attitude_init(void) {

  BOOZ_IEULER_ZERO(booz_stabilization_att_sp);

  BOOZ_IEULER_ZERO(booz_stabilization_att_ref);
  BOOZ_IVECT_ZERO(booz_stabilization_rate_ref);
  BOOZ_IVECT_ZERO(booz_stabilization_accel_ref);

  BOOZ_IVECT_ASSIGN(booz_stabilization_pgain,
		    BOOZ_STABILIZATION_ATTITUDE_PHI_THETA_PGAIN,
		    BOOZ_STABILIZATION_ATTITUDE_PHI_THETA_PGAIN,
		    BOOZ_STABILIZATION_ATTITUDE_PSI_PGAIN);
  
  BOOZ_IVECT_ASSIGN(booz_stabilization_dgain,
		    BOOZ_STABILIZATION_ATTITUDE_PHI_THETA_DGAIN,
		    BOOZ_STABILIZATION_ATTITUDE_PHI_THETA_DGAIN,
		    BOOZ_STABILIZATION_ATTITUDE_PSI_DGAIN);

  BOOZ_IVECT_ASSIGN(booz_stabilization_ddgain,
		    BOOZ_STABILIZATION_ATTITUDE_PHI_THETA_DDGAIN,
		    BOOZ_STABILIZATION_ATTITUDE_PHI_THETA_DDGAIN,
		    BOOZ_STABILIZATION_ATTITUDE_PSI_DDGAIN);

  BOOZ_IVECT_ASSIGN(booz_stabilization_igain,
		    BOOZ_STABILIZATION_ATTITUDE_PHI_THETA_IGAIN,
		    BOOZ_STABILIZATION_ATTITUDE_PHI_THETA_IGAIN,
		    BOOZ_STABILIZATION_ATTITUDE_PSI_IGAIN);

  BOOZ_IEULER_ZERO( booz_stabilization_att_sum_err );

}


void booz2_stabilization_attitude_read_rc(bool_t in_flight) {

  BOOZ2_STABILIZATION_ATTITUDE_READ_RC(booz_stabilization_att_sp, in_flight);

}


void booz2_stabilization_attitude_enter(void) {

  BOOZ2_STABILIZATION_ATTITUDE_RESET_PSI_REF(  booz_stabilization_att_sp );
  BOOZ_IEULER_ZERO( booz_stabilization_att_sum_err );
  
}


#define MAX_SUM_ERR 4000000

void booz2_stabilization_attitude_run(bool_t  in_flight) {

  booz_stabilization_update_ref();

  /* compute attitude error            */
  const struct booz_ieuler att_ref_scaled = {
    booz_stabilization_att_ref.phi   >> (ANGLE_REF_RES - IANGLE_RES),
    booz_stabilization_att_ref.theta >> (ANGLE_REF_RES - IANGLE_RES),
    booz_stabilization_att_ref.psi   >> (ANGLE_REF_RES - IANGLE_RES) };
  struct booz_ieuler att_err;
  BOOZ_IEULER_DIFF(att_err, booz_ahrs.ltp_to_body_euler, att_ref_scaled);
  BOOZ_ANGLE_NORMALIZE(att_err.psi);

  if (in_flight) {
    /* update integrator */
    BOOZ_IEULER_SUM(booz_stabilization_att_sum_err, booz_stabilization_att_sum_err, att_err);
    const struct booz_ieuler _MIN_SUM_ERR = {-MAX_SUM_ERR, -MAX_SUM_ERR, -MAX_SUM_ERR};
    const struct booz_ieuler _MAX_SUM_ERR = { MAX_SUM_ERR,  MAX_SUM_ERR,  MAX_SUM_ERR};
    BOOZ_IEULER_BOUND(booz_stabilization_att_sum_err, booz_stabilization_att_sum_err, _MIN_SUM_ERR, _MAX_SUM_ERR);
  }
  else {
    BOOZ_IEULER_ZERO(booz_stabilization_att_sum_err);
  }
  
  /* compute rate error                */
  const struct Int32Rates rate_ref_scaled = {
    booz_stabilization_rate_ref.x >> (RATE_REF_RES - IRATE_RES),
    booz_stabilization_rate_ref.y >> (RATE_REF_RES - IRATE_RES),
    booz_stabilization_rate_ref.z >> (RATE_REF_RES - IRATE_RES) };
  struct Int32Rates rate_err;
  RATES_DIFF(rate_err, booz_ahrs.body_rate, rate_ref_scaled);

  /* compute PID loop                  */
  booz2_stabilization_cmd[COMMAND_ROLL] = booz_stabilization_pgain.x    * att_err.phi +
    booz_stabilization_dgain.x    * rate_err.p +
    ((booz_stabilization_ddgain.x * booz_stabilization_accel_ref.x) >> 5) +
    ((booz_stabilization_igain.x  * booz_stabilization_att_sum_err.phi) >> 10);
  booz2_stabilization_cmd[COMMAND_ROLL] = booz2_stabilization_cmd[COMMAND_ROLL] >> 16;

  booz2_stabilization_cmd[COMMAND_PITCH] = booz_stabilization_pgain.y    * att_err.theta +
    booz_stabilization_dgain.y    * rate_err.q +
    ((booz_stabilization_ddgain.y * booz_stabilization_accel_ref.y) >> 5) +
    ((booz_stabilization_igain.y  * booz_stabilization_att_sum_err.theta) >> 10);
  booz2_stabilization_cmd[COMMAND_PITCH] = booz2_stabilization_cmd[COMMAND_PITCH] >> 16;
  
  booz2_stabilization_cmd[COMMAND_YAW] = booz_stabilization_pgain.z    * att_err.psi +
    booz_stabilization_dgain.z    * rate_err.r +
    ((booz_stabilization_ddgain.z * booz_stabilization_accel_ref.z) >> 5) +
    ((booz_stabilization_igain.z  * booz_stabilization_att_sum_err.psi) >> 10);
  booz2_stabilization_cmd[COMMAND_YAW] = booz2_stabilization_cmd[COMMAND_YAW] >> 16;
  
}


/* 

  generation of a saturated linear second order reference trajectory

  roll/pitch
  omega : 1100 deg s-1
  zeta : 0.85
  max rotational accel : 128 rad.s-2  ( ~7300 deg s-2 )
  max rotational speed :   8 rad/s    ( ~ 458 deg s-1 )

  yaw
  omega : 500 deg s-1
  zeta : 0.85
  max rotational accel : 32 rad.s-2  ( ~1833 deg s-2 )
  max rotational speed :  4 rad/s    ( ~ 230 deg s-1 )

  representation
  accel : 20.12
  speed : 16.16 
  angle : 12.20

*/

#define USE_REF 1

static inline void booz_stabilization_update_ref(void) {

#ifdef USE_REF
  BOOZ_STABILIZATION_ATTITUDE_REF_TRAJ_EULER_UPDATE();
#else
  BOOZ_IEULER_COPY(booz_stabilization_att_ref, booz_stabilization_att_sp);
  BOOZ_IVECT_ZERO(booz_stabilization_rate_ref);
  BOOZ_IVECT_ZERO(booz_stabilization_accel_ref);
#endif


}


