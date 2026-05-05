/*
 * Copyright (C) 2026 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 *
 */

/** @file filters/quaternion_filter.h
 *  @brief Quaternion second order filter
 *
 */

#ifndef QUATERNION_FILTER_H
#define QUATERNION_FILTER_H

#include "std.h"
#include "math/pprz_algebra_float.h"

/** Quaternion second order filter model (float) */
struct QuatSecondOrderLowPass {
  struct FloatQuat  quat;
  struct FloatRates rate;
  struct FloatRates accel;

  float two_zeta_omega;
  float two_omega2;
  float dt;
};

/** Init second order quaternion low pass filter.
 *
 * @param filter second order low pass filter structure
 * @param omega cutoff frequency of the second order low pass filter
 * @param damp damping coefficient of the second order low pass filter
 * @param sample_time sampling period of the signal
 * @param q0 initial orientation of the filter
 */
static inline void init_quat_second_order_low_pass(
    struct QuatSecondOrderLowPass *filter,
    const float omega,
    const float damp,
    const float sample_time,
    const struct FloatQuat q0)
{
  filter->quat = q0;
  FLOAT_RATES_ZERO(filter->rate);
  FLOAT_RATES_ZERO(filter->accel);
  filter->two_zeta_omega = 2.f * damp * omega;
  filter->two_omega2 = 2.f * omega * omega;
  filter->dt = sample_time;
}

/** Reset the second order quaternion low-pass filter to a specific value.
 *
 * @param filter filter structure
 * @param q0 quaternion to reset the filter to
 */
static inline void reset_quat_second_order_low_pass(
    struct QuatSecondOrderLowPass *filter,
    const struct FloatQuat q0)
{
  filter->quat = q0;
  FLOAT_RATES_ZERO(filter->rate);
  FLOAT_RATES_ZERO(filter->accel);
}

/** Update second order quaterion low pass filter state with a new value.
 *
 * @param filter filter structure
 * @param q quaternion input value of the filter
 * @return new filtered quaternion
 */
static inline struct FloatQuat update_quat_second_order_low_pass(
    struct QuatSecondOrderLowPass *filter,
    const struct FloatQuat q)
{
  /* integrate quaternion */
  struct FloatQuat qdot;
  float_quat_derivative(&qdot, &filter->rate, &filter->quat);
  QUAT_SMUL(qdot, qdot, filter->dt);
  QUAT_ADD(filter->quat, qdot);
  float_quat_normalize(&filter->quat);

  /* integrate rotational speed */
  struct FloatRates delta_rate;
  RATES_SMUL(delta_rate, filter->accel, filter->dt);
  RATES_ADD(filter->rate, delta_rate);

  /* compute angular accelerations */
  struct FloatQuat err;
  /* attitude error */
  float_quat_inv_comp(&err, &q, &filter->quat);
  /* wrap it in the shortest direction */
  float_quat_wrap_shortest(&err);
  /* propagate the 2nd order linear model: xdotdot = -2*zeta*omega*xdot - omega^2*x */
  /* since error quaternion contains the half-angles we get 2*omega^2*err */
  filter->accel.p = -filter->two_zeta_omega * filter->rate.p - filter->two_omega2 * err.qx;
  filter->accel.q = -filter->two_zeta_omega * filter->rate.q - filter->two_omega2 * err.qy;
  filter->accel.r = -filter->two_zeta_omega * filter->rate.r - filter->two_omega2 * err.qz;

  return filter->quat;
}

/** Get current value of the second order quaterion low pass filter.
 *
 * @param filter filter structure
 * @return current orientation of the filter
 */
static inline struct FloatQuat get_quat_second_order_low_pass(
    const struct QuatSecondOrderLowPass *filter)
{
  return filter->quat;
}


/** Second order Butterworth low pass filter.
 */
typedef struct QuatSecondOrderLowPass QuatButterworthLowPass;

/** Init a second order quaternion Butterworth filter.
 *
 * based on the generic second order filter
 * with zeta = 0.7071 = 1/sqrt(2)
 *
 * @param filter second order quaternion Butterworth low pass filter structure
 * @param omega cutoff frequency of the second order low pass filter
 * @param sample_time sampling period of the signal
 * @param q0 initial orientation of the filter
 */
static inline void init_quat_butterworth_low_pass(
    QuatButterworthLowPass *filter,
    const float omega,
    const float sample_time,
    const struct FloatQuat q0)
{
  init_quat_second_order_low_pass((struct QuatSecondOrderLowPass *)filter, omega, 0.7071f, sample_time, q0);
}

/** Update second order quaternion Butterworth low pass filter state with a new value.
 *
 * @param filter second order quaternion Butterworth low pass filter structure
 * @param q new input orientation of the filter
 * @return new filtered quaternion
 */
static inline struct FloatQuat update_quat_butterworth_low_pass(
    QuatButterworthLowPass *filter,
    const struct FloatQuat quat)
{
  return update_quat_second_order_low_pass((struct QuatSecondOrderLowPass *)filter, quat);
}

/**
 * @brief Reset a quaternion Butterworth low-pass filter to a specific value.
 *
 * @param[out] filter QuatButterworth2LowPass filter instance to reset.
 * @param[in] quat Value to reset the filter to.
 */
static inline void reset_quat_butterworth_low_pass(
    QuatButterworthLowPass *filter,
    const struct FloatQuat quat)
{
  reset_quat_second_order_low_pass((struct QuatSecondOrderLowPass *)filter, quat);
}

/** Get current value of the second order quaternion Butterworth low pass filter.
 *
 * @param filter second order quaternion Butterworth low pass filter structure
 * @return current orientation of the filter
 */
static inline struct FloatQuat get_quat_butterworth_low_pass(
    const QuatButterworthLowPass *filter)
{
  return get_quat_second_order_low_pass((const struct QuatSecondOrderLowPass *)filter);
}

#endif

