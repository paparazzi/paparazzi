/*
 * Copyright (C) 2015 Jean-Philippe Condomines, Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ahrs/ahrs_float_invariant.c
 * @author Jean-Philippe Condomines <jp.condomines@gmail.com>
 *
 * AHRS using invariant filter.
 *
 */

#include "modules/ahrs/ahrs_float_invariant.h"

#include "modules/ahrs/ahrs_int_utils.h"

#include "generated/airframe.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_rk_float.h"

#if SEND_INVARIANT_FILTER
#include "modules/datalink/telemetry.h"
#endif

/*---------Invariant Observers-----------
 *
 *            State vector :
 *  x = [q0 q1 q2 q3 wb1 wb2 wb3 as cs]
 *
 *
 *            Dynamic model (dim = 9) :
 *  x_qdot     = 0.5 * x_quat * ( x_rates - x_bias );
 *  x_bias_dot = 0;
 *  x_asdot    = 0;
 *  x_csdot    = 0;
 *
 *            Observation model (dim = 6) :
 *  ya = as * (q)-1 * A * q; (A : accelerometers)
 *  yc = cs * (q)-1 * C * q; (C = A x B (cross product), B : magnetometers)
 *
 *------------------------------------------*/

// Default values for the tuning gains
// Tuning parameter of accel and mag on attitude
#ifndef AHRS_INV_LX
#define AHRS_INV_LX 2. * (0.06 + 0.1)
#endif
// Tuning parameter of accel and mag on attitude
#ifndef AHRS_INV_LY
#define AHRS_INV_LY 2. * (0.06 + 0.06)
#endif
// Tuning parameter of accel and mag on attitude
#ifndef AHRS_INV_LZ
#define AHRS_INV_LZ 2. * (0.1 + 0.06)
#endif
// Tuning parameter of accel and mag on gyro bias
#ifndef AHRS_INV_MX
#define AHRS_INV_MX 2. * 0.05 * (0.06 + 0.1)
#endif
// Tuning parameter of accel and mag on gyro bias
#ifndef AHRS_INV_MY
#define AHRS_INV_MY 2. * 0.05 * (0.06 + 0.06)
#endif
// Tuning parameter of accel and mag on gyro bias
#ifndef AHRS_INV_MZ
#define AHRS_INV_MZ 2. * 0.05 * (0.1 + 0.06)
#endif
// Tuning parameter of accel and mag on accel bias
#ifndef AHRS_INV_N
#define AHRS_INV_N 0.25
#endif
// Tuning parameter of accel and mag on mag bias
#ifndef AHRS_INV_O
#define AHRS_INV_O 0.5
#endif

struct AhrsFloatInv ahrs_float_inv;

/* earth gravity model */
static const struct FloatVect3 A = { 0.f, 0.f, -9.81f };

/* earth magnetic model */
#define B ahrs_float_inv.mag_h

/* error computation */
static inline void error_output(struct AhrsFloatInv *_ins);

/* propagation model (called by runge-kutta library) */
static inline void invariant_model(float *o, const float *x, const int n, const float *u, const int m);


/** Right multiplication by a quaternion.
 * vi * q
 */
void float_quat_vmul_right(struct FloatQuat *mright, const struct FloatQuat *q,
                           struct FloatVect3 *vi);

/* init state and measurements */
static inline void init_invariant_state(void)
{
  // init state
  float_quat_identity(&ahrs_float_inv.state.quat);
  FLOAT_RATES_ZERO(ahrs_float_inv.state.bias);
  ahrs_float_inv.state.as = 1.0f;
  ahrs_float_inv.state.cs = 1.0f;

  // init measures
  FLOAT_VECT3_ZERO(ahrs_float_inv.meas.accel);
  FLOAT_VECT3_ZERO(ahrs_float_inv.meas.mag);

}

void ahrs_float_invariant_init(void)
{
  // init magnetometers

  ahrs_float_inv.mag_h.x = AHRS_H_X;
  ahrs_float_inv.mag_h.y = AHRS_H_Y;
  ahrs_float_inv.mag_h.z = AHRS_H_Z;

  // init state and measurements
  init_invariant_state();

  // init gains
  ahrs_float_inv.gains.lx  = AHRS_INV_LX;
  ahrs_float_inv.gains.ly  = AHRS_INV_LY;
  ahrs_float_inv.gains.lz  = AHRS_INV_LZ;
  ahrs_float_inv.gains.mx  = AHRS_INV_MX;
  ahrs_float_inv.gains.my  = AHRS_INV_MY;
  ahrs_float_inv.gains.mz  = AHRS_INV_MZ;
  ahrs_float_inv.gains.n   = AHRS_INV_N;
  ahrs_float_inv.gains.o   = AHRS_INV_O;

  ahrs_float_inv.is_aligned = false;
  ahrs_float_inv.reset = false;
}

void ahrs_float_invariant_align(struct FloatRates *lp_gyro,
                                struct FloatVect3 *lp_accel,
                                struct FloatVect3 *lp_mag)
{
  /* Compute an initial orientation from accel and mag directly as quaternion */
  ahrs_float_get_quat_from_accel_mag(&ahrs_float_inv.state.quat, lp_accel, lp_mag);

  /* use average gyro as initial value for bias */
  ahrs_float_inv.state.bias = *lp_gyro;

  // ins and ahrs are now running
  ahrs_float_inv.is_aligned = true;
}

void ahrs_float_invariant_propagate(struct FloatRates* gyro, float dt)
{
  // realign all the filter if needed
  // a complete init cycle is required
  if (ahrs_float_inv.reset) {
    ahrs_float_inv.reset = false;
    ahrs_float_inv.is_aligned = false;
    init_invariant_state();
  }

  // fill command vector
  RATES_COPY(ahrs_float_inv.cmd.rates, *gyro);

  // update correction gains
  error_output(&ahrs_float_inv);

  // propagate model
  struct inv_state new_state;
  runge_kutta_4_float((float *)&new_state,
                      (float *)&ahrs_float_inv.state, INV_STATE_DIM,
                      (float *)&ahrs_float_inv.cmd, INV_COMMAND_DIM,
                      invariant_model, dt);
  ahrs_float_inv.state = new_state;

  // normalize quaternion
  float_quat_normalize(&ahrs_float_inv.state.quat);

  //------------------------------------------------------------//

#if SEND_INVARIANT_FILTER
  struct FloatEulers eulers;
  float foo = 0.f;
  float_eulers_of_quat(&eulers, &ahrs_float_inv.state.quat);
  RunOnceEvery(3,
      pprz_msg_send_INV_FILTER(&(DefaultChannel).trans_tx, &(DefaultDevice).device,
        AC_ID,
        &ahrs_float_inv.state.quat.qi,
        &eulers.phi,
        &eulers.theta,
        &eulers.psi,
        &foo,
        &foo,
        &foo,
        &foo,
        &foo,
        &foo,
        &ahrs_float_inv.state.bias.p,
        &ahrs_float_inv.state.bias.q,
        &ahrs_float_inv.state.bias.r,
        &ahrs_float_inv.state.as,
        &ahrs_float_inv.state.cs,
        &foo,
        &foo);
      );
#endif

}

void ahrs_float_invariant_update_accel(struct FloatVect3* accel)
{
  ahrs_float_inv.meas.accel = *accel;
}

// assume mag is dead when values are not moving anymore
#define MAG_FROZEN_COUNT 30

void ahrs_float_invariant_update_mag(struct FloatVect3* mag)
{
  static uint32_t mag_frozen_count = MAG_FROZEN_COUNT;
  static int32_t last_mx = 0;

  if (last_mx == mag->x) {
    mag_frozen_count--;
    if (mag_frozen_count == 0) {
      // if mag is dead, better set measurements to zero
      FLOAT_VECT3_ZERO(ahrs_float_inv.meas.mag);
      mag_frozen_count = MAG_FROZEN_COUNT;
    }
  } else {
    // new values in body frame
    VECT3_COPY(ahrs_float_inv.meas.mag, *mag);
    // reset counter
    mag_frozen_count = MAG_FROZEN_COUNT;
  }
  last_mx = mag->x;
}

/** Compute dynamic mode
 *
 * x_dot = evolution_model + (gain_matrix * error)
 */
static inline void invariant_model(float *o, const float *x, const int n, const float *u,
                                   const int m __attribute__((unused)))
{

#pragma GCC diagnostic push // require GCC 4.6
#pragma GCC diagnostic ignored "-Wcast-qual"
  struct inv_state *s = (struct inv_state *)x;
  struct inv_command *c = (struct inv_command *)u;
#pragma GCC diagnostic pop // require GCC 4.6
  struct inv_state s_dot;
  struct FloatRates rates_unbiased;
  struct FloatVect3 tmp_vect;
  struct FloatQuat tmp_quat;

  // test accel sensitivity
  if (fabs(s->as) < 0.1) {
    // too small, return x_dot = 0 to avoid division by 0
    float_vect_zero(o, n);
    // TODO set ins state to error
    return;
  }

  /* dot_q = 0.5 * q * (x_rates - x_bias) + LE * q + (1 - ||q||^2) * q */
  RATES_DIFF(rates_unbiased, c->rates, s->bias);
  /* qd = 0.5 * q * rates_unbiased = -0.5 * rates_unbiased * q */
  float_quat_derivative(&s_dot.quat, &rates_unbiased, &(s->quat));

  float_quat_vmul_right(&tmp_quat, &(s->quat), &ahrs_float_inv.corr.LE);
  QUAT_ADD(s_dot.quat, tmp_quat);

  float norm2_r = 1. - FLOAT_QUAT_NORM2(s->quat);
  QUAT_SMUL(tmp_quat, s->quat, norm2_r);
  QUAT_ADD(s_dot.quat, tmp_quat);

  /* bias_dot = q-1 * (ME) * q */
  float_quat_vmult(&tmp_vect, &(s->quat), &ahrs_float_inv.corr.ME);
  RATES_ASSIGN(s_dot.bias, tmp_vect.x, tmp_vect.y, tmp_vect.z);

  /* as_dot = as * NE */
  s_dot.as = (s->as) * (ahrs_float_inv.corr.NE);

  /* cs_dot = OE */
  s_dot.cs = ahrs_float_inv.corr.OE;

  // set output
  memcpy(o, &s_dot, n * sizeof(float));
}

/** Compute correction vectors
 * E = ( ŷ - y )
 * LE, ME, NE, OE : ( gain matrix * error )
 */
static inline void error_output(struct AhrsFloatInv *_ahrs)
{

  struct FloatVect3 YAt, C, YC, YCt, Ec, Ea;

  // test accel sensitivity
  if (fabs(_ahrs->state.as) < 0.1) {
    // too small, don't do anything to avoid division by 0
    return;
  }

  /* C = A X B Cross product */
  VECT3_CROSS_PRODUCT(C, A, B);
  /* YC = YA X YB Cross product */
  VECT3_CROSS_PRODUCT(YC, _ahrs->meas.accel, _ahrs->meas.mag);
  /* YCt = (1 / cs) * q * YC * q-1 => invariant output on magnetometers */
  struct FloatQuat q_b2n;
  float_quat_invert(&q_b2n, &(_ahrs->state.quat));
  float_quat_vmult(&YCt, &q_b2n, &YC);
  VECT3_SMUL(YCt, YCt, 1. / (_ahrs->state.cs));

  /* YAt = q * yA * q-1 => invariant output on accelerometers  */
  float_quat_vmult(&YAt, &q_b2n, &(_ahrs->meas.accel));
  VECT3_SMUL(YAt, YAt, 1. / (_ahrs->state.as));

  /*--------- E = ( ŷ - y ) ----------*/

  /* EC = ( C - YCt ) */
  VECT3_DIFF(Ec, C, YCt);
  /* EA = ( A - YAt ) */
  VECT3_DIFF(Ea, A, YAt);

  /*-------------- Gains --------------*/

  /**** LaEa + LcEc *****/
  _ahrs->corr.LE.x = (-_ahrs->gains.lx * Ea.y) / (2.f * A.z);
  _ahrs->corr.LE.y = (_ahrs->gains.ly * Ea.x) / (2.f * A.z);
  _ahrs->corr.LE.z = (-_ahrs->gains.lz * Ec.x) / (B.x * 2.f * A.z);

  /***** MaEa + McEc ********/
  _ahrs->corr.ME.x = (_ahrs->gains.mx * Ea.y) / (2.f * A.z);
  _ahrs->corr.ME.y = (-_ahrs->gains.my * Ea.x)/(2.f * A.z);
  _ahrs->corr.ME.z = (_ahrs->gains.mz * Ec.x) / (B.x * 2.f * A.z);

  /****** NaEa + NcEc ********/
  _ahrs->corr.NE = (-_ahrs->gains.n * Ea.z) / A.z;

  /****** OaEa + OcEc ********/
  _ahrs->corr.OE = (-_ahrs->gains.o * Ec.y) / (B.x * A.z);
}


void float_quat_vmul_right(struct FloatQuat *mright, const struct FloatQuat *q,
    struct FloatVect3 *vi)
{
  struct FloatVect3 qvec, v1, v2;
  float qi;

  FLOAT_QUAT_EXTRACT(qvec, *q);
  qi = - VECT3_DOT_PRODUCT(*vi, qvec);
  VECT3_CROSS_PRODUCT(v1, *vi, qvec);
  VECT3_SMUL(v2, *vi, q->qi);
  VECT3_ADD(v2, v1);
  QUAT_ASSIGN(*mright, qi, v2.x, v2.y, v2.z);
}
