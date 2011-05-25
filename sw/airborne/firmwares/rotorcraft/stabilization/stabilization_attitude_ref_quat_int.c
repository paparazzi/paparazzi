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

/** \file stabilization_attitude_ref_int.c
 *  \brief Booz attitude reference generation (quaternion int version)
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "subsystems/ahrs.h"

#include "stabilization_attitude_ref_int.h"
//#include "quat_setpoint.h"

#define REF_ACCEL_MAX_P BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_PDOT, REF_ACCEL_FRAC)
#define REF_ACCEL_MAX_Q BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_QDOT, REF_ACCEL_FRAC)
#define REF_ACCEL_MAX_R BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_RDOT, REF_ACCEL_FRAC)

#define REF_RATE_MAX_P BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_P, REF_RATE_FRAC)
#define REF_RATE_MAX_Q BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_Q, REF_RATE_FRAC)
#define REF_RATE_MAX_R BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_R, REF_RATE_FRAC)

#define OMEGA_P   STABILIZATION_ATTITUDE_REF_OMEGA_P 
#define ZETA_P    STABILIZATION_ATTITUDE_REF_ZETA_P
#define ZETA_OMEGA_P_RES 10
#define ZETA_OMEGA_P BFP_OF_REAL((ZETA_P*OMEGA_P), ZETA_OMEGA_P_RES)
#define OMEGA_2_P_RES 7
#define OMEGA_2_P    BFP_OF_REAL((OMEGA_P*OMEGA_P), OMEGA_2_P_RES)

#define OMEGA_Q   STABILIZATION_ATTITUDE_REF_OMEGA_Q
#define ZETA_Q    STABILIZATION_ATTITUDE_REF_ZETA_Q
#define ZETA_OMEGA_Q_RES 10
#define ZETA_OMEGA_Q BFP_OF_REAL((ZETA_Q*OMEGA_Q), ZETA_OMEGA_Q_RES)
#define OMEGA_2_Q_RES 7
#define OMEGA_2_Q    BFP_OF_REAL((OMEGA_Q*OMEGA_Q), OMEGA_2_Q_RES)

#define OMEGA_R   STABILIZATION_ATTITUDE_REF_OMEGA_R
#define ZETA_R    STABILIZATION_ATTITUDE_REF_ZETA_R
#define ZETA_OMEGA_R_RES 10
#define ZETA_OMEGA_R BFP_OF_REAL((ZETA_R*OMEGA_R), ZETA_OMEGA_R_RES)
#define OMEGA_2_R_RES 7
#define OMEGA_2_R    BFP_OF_REAL((OMEGA_R*OMEGA_R), OMEGA_2_R_RES)


struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;
struct Int32Eulers stab_att_ref_euler;
struct Int32Quat   stab_att_ref_quat;
struct Int32Rates  stab_att_ref_rate;
struct Int32Rates  stab_att_ref_accel;

struct Int32RefModel stab_att_ref_model = {
  {STABILIZATION_ATTITUDE_REF_OMEGA_P, STABILIZATION_ATTITUDE_REF_OMEGA_Q, STABILIZATION_ATTITUDE_REF_OMEGA_R},
  {STABILIZATION_ATTITUDE_REF_ZETA_P, STABILIZATION_ATTITUDE_REF_ZETA_Q, STABILIZATION_ATTITUDE_REF_ZETA_R}
};

/*
static const float omega_p[] = STABILIZATION_ATTITUDE_REF_OMEGA_P;
static const float zeta_p[] = STABILIZATION_ATTITUDE_REF_ZETA_P;
static const float omega_q[] = STABILIZATION_ATTITUDE_REF_OMEGA_Q;
static const float zeta_q[] = STABILIZATION_ATTITUDE_REF_ZETA_Q;
static const float omega_r[] = STABILIZATION_ATTITUDE_REF_OMEGA_R;
static const float zeta_r[] = STABILIZATION_ATTITUDE_REF_ZETA_R;
*/

static void reset_psi_ref_from_body(void) {
    stab_att_ref_euler.psi = ahrs.ltp_to_body_euler.psi;
    stab_att_ref_rate.r = 0;
    stab_att_ref_accel.r = 0;
}

static void update_ref_quat_from_eulers(void) {
    struct Int32RMat ref_rmat;

#ifdef STICKS_RMAT312
    INT32_RMAT_OF_EULERS_312(ref_rmat, stab_att_ref_euler);
#else
    INT32_RMAT_OF_EULERS_321(ref_rmat, stab_att_ref_euler);
#endif
    INT32_QUAT_OF_RMAT(stab_att_ref_quat, ref_rmat);
    INT32_QUAT_WRAP_SHORTEST(stab_att_ref_quat);
}

void stabilization_attitude_ref_init(void) {

  INT_EULERS_ZERO(stab_att_sp_euler);
  INT32_QUAT_ZERO(  stab_att_sp_quat);
  INT_EULERS_ZERO(stab_att_ref_euler);
  INT32_QUAT_ZERO(  stab_att_ref_quat);
  INT_RATES_ZERO( stab_att_ref_rate);
  INT_RATES_ZERO( stab_att_ref_accel);

  /*
  for (int i = 0; i < STABILIZATION_ATTITUDE_GAIN_NB; i++) {
    RATES_ASSIGN(stab_att_ref_model[i].omega, omega_p[i], omega_q[i], omega_r[i]);
    RATES_ASSIGN(stab_att_ref_model[i].zeta, zeta_p[i], zeta_q[i], zeta_r[i]);
  }
  */

}

void stabilization_attitude_ref_enter()
{
  reset_psi_ref_from_body();
  stabilization_attitude_sp_enter();
  memcpy(&stab_att_ref_quat, &stab_att_sp_quat, sizeof(struct Int32Quat));
  memset(&stab_att_ref_accel, 0, sizeof(struct Int32Rates));
  memset(&stab_att_ref_rate, 0, sizeof(struct Int32Rates));
  //update_ref_quat_from_eulers();
}

/*
 * Reference
 */
#define DT_UPDATE (1./512.)
#define F_UPDATE_RES 9

#include "messages.h"
#include "mcu_periph/uart.h"
#include "downlink.h"

void stabilization_attitude_ref_update() {

  /* integrate reference attitude            */
  struct Int32Quat qdot;
  INT32_QUAT_DERIVATIVE(qdot, stab_att_ref_rate, stab_att_ref_quat);
  //QUAT_SMUL(qdot, qdot, RATE_BFP_OF_REAL(DT_UPDATE));
  QUAT_SMUL(qdot, qdot, 4);
  QUAT_SMUL(qdot, qdot, DT_UPDATE);
  QUAT_ADD(stab_att_ref_quat, qdot);
  INT32_QUAT_NORMALIZE(stab_att_ref_quat);

  /* integrate reference rotational speeds   */
  const struct Int32Rates delta_rate = {
         stab_att_ref_accel.p >> ( F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC),
         stab_att_ref_accel.q >> ( F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC),
         stab_att_ref_accel.r >> ( F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC)};

  //RATES_SMUL(delta_rate, stab_att_ref_accel, RATE_BFP_OF_REAL(DT_UPDATE));
  //RATES_SMUL(delta_rate, stab_att_ref_accel, DT_UPDATE);
  RATES_ADD(stab_att_ref_rate, delta_rate);

  /* compute reference angular accelerations */
  struct Int32Quat err;
  /* compute reference attitude error        */
  INT32_QUAT_INV_COMP(err, stab_att_sp_quat, stab_att_ref_quat);
  /* wrap it in the shortest direction       */
  INT32_QUAT_WRAP_SHORTEST(err);
  /* propagate the 2nd order linear model    */

  const struct Int32Rates accel_rate = {
    ((int32_t)(-2.*ZETA_OMEGA_P) * (stab_att_ref_rate.p >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (ZETA_OMEGA_P_RES),
    ((int32_t)(-2.*ZETA_OMEGA_Q) * (stab_att_ref_rate.q >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (ZETA_OMEGA_Q_RES),
    ((int32_t)(-2.*ZETA_OMEGA_R) * (stab_att_ref_rate.r >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (ZETA_OMEGA_R_RES) };

  const struct Int32Rates accel_angle = {
    ((int32_t)(-OMEGA_2_P)* (err.qx   >> (REF_ANGLE_FRAC - REF_ACCEL_FRAC))) >> (OMEGA_2_P_RES),
    ((int32_t)(-OMEGA_2_Q)* (err.qy   >> (REF_ANGLE_FRAC - REF_ACCEL_FRAC))) >> (OMEGA_2_Q_RES),
    ((int32_t)(-OMEGA_2_R)* (err.qz   >> (REF_ANGLE_FRAC - REF_ACCEL_FRAC))) >> (OMEGA_2_R_RES) };

  RATES_SUM(stab_att_ref_accel, accel_rate, accel_angle);


        /*
  stab_att_ref_accel.p = -2.*stab_att_ref_model.zeta.p*stab_att_ref_model.omega.p*stab_att_ref_rate.p
    - stab_att_ref_model.omega.p*stab_att_ref_model.omega.p*err.qx;
  stab_att_ref_accel.q = -2.*stab_att_ref_model.zeta.q*stab_att_ref_model.omega.q*stab_att_ref_rate.q
    - stab_att_ref_model.omega.q*stab_att_ref_model.omega.q*err.qy;
  stab_att_ref_accel.r = -2.*stab_att_ref_model.zeta.r*stab_att_ref_model.omega.r*stab_att_ref_rate.r
    - stab_att_ref_model.omega.r*stab_att_ref_model.omega.r*err.qz;
    */

  /*	saturate acceleration */
  //const struct Int32Rates MIN_ACCEL = { -REF_ACCEL_MAX_P, -REF_ACCEL_MAX_Q, -REF_ACCEL_MAX_R };
  //const struct Int32Rates MAX_ACCEL = {  REF_ACCEL_MAX_P,  REF_ACCEL_MAX_Q,  REF_ACCEL_MAX_R };
  //RATES_BOUND_BOX(stab_att_ref_accel, MIN_ACCEL, MAX_ACCEL);

  /* compute ref_euler */
  INT32_EULERS_OF_QUAT(stab_att_ref_euler, stab_att_ref_quat);

}
