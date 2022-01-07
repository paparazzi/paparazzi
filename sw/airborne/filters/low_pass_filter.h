/*
 * Copyright (C) 2013 Gautier Hattenberger
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

/** @file filters/low_pass_filter.h
 *  @brief Simple first order low pass filter with bilinear transform
 *
 */

#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#include "std.h"
#include "math/pprz_algebra_int.h"

#define INT32_FILT_FRAC  8

/** First order low pass filter structure.
 *
 * using bilinear z transform
 */
struct FirstOrderLowPass {
  float time_const;
  float last_in;
  float last_out;
};

/** Init first order low pass filter.
 *
 * Laplace transform in continious time:
 *            1
 * H(s) = ---------
 *        1 + tau*s
 *
 * @param filter first order low pass filter structure
 * @param tau time constant of the first order low pass filter
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
static inline void init_first_order_low_pass(struct FirstOrderLowPass *filter, float tau, float sample_time,
    float value)
{
  filter->last_in = value;
  filter->last_out = value;
  filter->time_const = 2.0f * tau / sample_time;
}

/** Update first order low pass filter state with a new value.
 *
 * @param filter first order low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
static inline float update_first_order_low_pass(struct FirstOrderLowPass *filter, float value)
{
  float out = (value + filter->last_in + (filter->time_const - 1.0f) * filter->last_out) / (1.0f + filter->time_const);
  filter->last_in = value;
  filter->last_out = out;
  return out;
}

/** Get current value of the first order low pass filter.
 *
 * @param filter first order low pass filter structure
 * @return current value of the filter
 */
static inline float get_first_order_low_pass(struct FirstOrderLowPass *filter)
{
  return filter->last_out;
}

/** Update time constant (tau parameter) for first order low pass filter
 *
 * @param filter first order low pass filter structure
 * @param tau time constant of the first order low pass filter
 * @param sample_time sampling period of the signal
 */
static inline void update_first_order_low_pass_tau(struct FirstOrderLowPass *filter, float tau, float sample_time)
{
  filter->time_const = 2.0f * tau / sample_time;
}

/** Second order low pass filter structure.
 *
 * using biquad filter with bilinear z transform
 *
 * http://en.wikipedia.org/wiki/Digital_biquad_filter
 * http://www.earlevel.com/main/2003/03/02/the-bilinear-z-transform
 *
 * Laplace continious form:
 *
 *                 1
 * H(s) = -------------------
 *        s^2/w^2 + s/w*Q + 1
 *
 *
 * Polynomial discrete form:
 *
 *        b0 + b1 z^-1 + b2 z^-2
 * H(z) = ----------------------
 *        a0 + a1 z^-1 + a2 z^-2
 *
 * with:
 *  a0 = 1
 *  a1 = 2*(K^2 - 1) / (K^2 + K/Q + 1)
 *  a2 = (K^2 - K/Q + 1) / (K^2 + K/Q + 1)
 *  b0 = K^2 / (K^2 + K/Q + 1)
 *  b1 = 2*b0
 *  b2 = b0
 *  K = tan(pi*Fc/Fs) ~ pi*Fc/Fs = Ts/(2*tau)
 *  Fc: cutting frequency
 *  Fs: sampling frequency
 *  Ts: sampling period
 *  tau: time constant (tau = 1/(2*pi*Fc))
 *  Q: gain at cutoff frequency
 *
 * Note that b[0]=b[2], so we don't need to save b[2]
 */
struct SecondOrderLowPass {
  float a[2]; ///< denominator gains
  float b[2]; ///< numerator gains
  float i[2]; ///< input history
  float o[2]; ///< output history
};


/** Init second order low pass filter.
 *
 * @param filter second order low pass filter structure
 * @param tau time constant of the second order low pass filter
 * @param Q Q value of the second order low pass filter
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
static inline void init_second_order_low_pass(struct SecondOrderLowPass *filter, float tau, float Q, float sample_time,
    float value)
{
  float K = tanf(sample_time / (2.0f * tau));
  float poly = K * K + K / Q + 1.0f;
  filter->a[0] = 2.0f * (K * K - 1.0f) / poly;
  filter->a[1] = (K * K - K / Q + 1.0f) / poly;
  filter->b[0] = K * K / poly;
  filter->b[1] = 2.0f * filter->b[0];
  filter->i[0] = filter->i[1] = filter->o[0] = filter->o[1] = value;
}

/** Update second order low pass filter state with a new value.
 *
 * @param filter second order low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
static inline float update_second_order_low_pass(struct SecondOrderLowPass *filter, float value)
{
  float out = filter->b[0] * value
              + filter->b[1] * filter->i[0]
              + filter->b[0] * filter->i[1]
              - filter->a[0] * filter->o[0]
              - filter->a[1] * filter->o[1];
  filter->i[1] = filter->i[0];
  filter->i[0] = value;
  filter->o[1] = filter->o[0];
  filter->o[0] = out;
  return out;
}

/** Get current value of the second order low pass filter.
 *
 * @param filter second order low pass filter structure
 * @return current value of the filter
 */
static inline float get_second_order_low_pass(struct SecondOrderLowPass *filter)
{
  return filter->o[0];
}

struct SecondOrderLowPass_int {
  int32_t a[2]; ///< denominator gains
  int32_t b[2]; ///< numerator gains
  int32_t i[2]; ///< input history
  int32_t o[2]; ///< output history
  int32_t loop_gain; ///< loop gain
};

/** Init second order low pass filter(fixed point version).
 *
 * @param filter second order low pass filter structure
 * @param cut_off Cutoff frequency of the filter with -3dB
 * @param Q Q value of the second order low pass filter
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
static inline void init_second_order_low_pass_int(struct SecondOrderLowPass_int *filter, float cut_off, float Q,
    float sample_time, int32_t value)
{
  struct SecondOrderLowPass filter_temp;
  float tau = 7.0f / (44.0f * cut_off);
  float K = sample_time / (2.0f * tau);
  float poly = K * K + K / Q + 1.0f;
  float loop_gain_f;

  filter_temp.a[0] = 2.0f * (K * K - 1.0f) / poly;
  filter_temp.a[1] = (K * K - K / Q + 1.0f) / poly;
  filter_temp.b[0] = K * K / poly;
  filter_temp.b[1] = 2.0f * filter_temp.b[0];
  loop_gain_f = 1.0f / filter_temp.b[0];

  filter->a[0] = BFP_OF_REAL((filter_temp.a[0] * loop_gain_f), INT32_FILT_FRAC);
  filter->a[1] = BFP_OF_REAL((filter_temp.a[1] * loop_gain_f), INT32_FILT_FRAC);
  filter->b[0] = BFP_OF_REAL(1, INT32_FILT_FRAC);
  filter->b[1] = 2 * filter->b[0];
  filter->i[0] = filter->i[1] = filter->o[0] = filter->o[1] = value;
  filter->loop_gain = BFP_OF_REAL(loop_gain_f, INT32_FILT_FRAC);
}

/** Update second order low pass filter state with a new value(fixed point version).
 *
 * @param filter second order low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
static inline int32_t update_second_order_low_pass_int(struct SecondOrderLowPass_int *filter, int32_t value)
{
  int32_t out = filter->b[0] * value
                + filter->b[1] * filter->i[0]
                + filter->b[0] * filter->i[1]
                - filter->a[0] * filter->o[0]
                - filter->a[1] * filter->o[1];

  filter->i[1] = filter->i[0];
  filter->i[0] = value;
  filter->o[1] = filter->o[0];
  filter->o[0] = out / (filter->loop_gain);
  return filter->o[0];
}

/** Get current value of the second order low pass filter(fixed point version).
 *
 * @param filter second order low pass filter structure
 * @return current value of the filter
 */
static inline int32_t get_second_order_low_pass_int(struct SecondOrderLowPass_int *filter)
{
  return filter->o[0];
}

/** Second order Butterworth low pass filter.
 */
typedef struct SecondOrderLowPass Butterworth2LowPass;

/** Init a second order Butterworth filter.
 *
 * based on the generic second order filter
 * with Q = 0.7071 = 1/sqrt(2)
 *
 * http://en.wikipedia.org/wiki/Butterworth_filter
 *
 * @param filter second order Butterworth low pass filter structure
 * @param tau time constant of the second order low pass filter
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
static inline void init_butterworth_2_low_pass(Butterworth2LowPass *filter, float tau, float sample_time, float value)
{
  init_second_order_low_pass((struct SecondOrderLowPass *)filter, tau, 0.7071, sample_time, value);
}

/** Update second order Butterworth low pass filter state with a new value.
 *
 * @param filter second order Butterworth low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
static inline float update_butterworth_2_low_pass(Butterworth2LowPass *filter, float value)
{
  return update_second_order_low_pass((struct SecondOrderLowPass *)filter, value);
}

/** Get current value of the second order Butterworth low pass filter.
 *
 * @param filter second order Butterworth low pass filter structure
 * @return current value of the filter
 */
static inline float get_butterworth_2_low_pass(Butterworth2LowPass *filter)
{
  return filter->o[0];
}

/** Second order Butterworth low pass filter(fixed point version).
 */
typedef struct SecondOrderLowPass_int Butterworth2LowPass_int;

/** Init a second order Butterworth filter.
 *
 * based on the generic second order filter
 * with Q = 0.7071 = 1/sqrt(2)
 *
 * http://en.wikipedia.org/wiki/Butterworth_filter
 *
 * @param filter second order Butterworth low pass filter structure
 * @param cut_off Cutoff frequency of the filter with -3dB
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
static inline void init_butterworth_2_low_pass_int(Butterworth2LowPass_int *filter, float cut_off, float sample_time,
    int32_t value)
{
  init_second_order_low_pass_int((struct SecondOrderLowPass_int *)filter, cut_off, 0.7071, sample_time, value);
}

/** Update second order Butterworth low pass filter state with a new value(fixed point version).
 *
 * @param filter second order Butterworth low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
static inline int32_t update_butterworth_2_low_pass_int(Butterworth2LowPass_int *filter, int32_t value)
{
  return update_second_order_low_pass_int((struct SecondOrderLowPass_int *)filter, value);
}

/** Get current value of the second order Butterworth low pass filter(fixed point version).
 *
 * @param filter second order Butterworth low pass filter structure
 * @return current value of the filter
 */
static inline int32_t get_butterworth_2_low_pass_int(Butterworth2LowPass_int *filter)
{
  return filter->o[0];
}

/** Fourth order Butterworth low pass filter.
 *
 * using two cascaded second order filters
 */
typedef struct {
  struct SecondOrderLowPass lp1;
  struct SecondOrderLowPass lp2;
} Butterworth4LowPass;

/** Init a fourth order Butterworth filter.
 *
 * based on two generic second order filters
 * with Q1 = 1.30651
 *  and Q2 = 0.541184
 *
 * http://en.wikipedia.org/wiki/Butterworth_filter
 *
 * @param filter fourth order Butterworth low pass filter structure
 * @param tau time constant of the fourth order low pass filter
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
static inline void init_butterworth_4_low_pass(Butterworth4LowPass *filter, float tau, float sample_time, float value)
{
  init_second_order_low_pass(&filter->lp1, tau, 1.30651, sample_time, value);
  init_second_order_low_pass(&filter->lp2, tau, 0.51184, sample_time, value);
}

/** Update fourth order Butterworth low pass filter state with a new value.
 *
 * using two cascaded second order filters
 *
 * @param filter fourth order Butterworth low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
static inline float update_butterworth_4_low_pass(Butterworth4LowPass *filter, float value)
{
  float tmp = update_second_order_low_pass(&filter->lp1, value);
  return update_second_order_low_pass(&filter->lp2, tmp);
}

/** Get current value of the fourth order Butterworth low pass filter.
 *
 * @param filter fourth order Butterworth low pass filter structure
 * @return current value of the filter
 */
static inline float get_butterworth_4_low_pass(Butterworth4LowPass *filter)
{
  return filter->lp2.o[0];
}

/** Fourth order Butterworth low pass filter(fixed point version).
 *
 * using two cascaded second order filters
 */
typedef struct {
  struct SecondOrderLowPass_int lp1;
  struct SecondOrderLowPass_int lp2;
} Butterworth4LowPass_int;

/** Init a fourth order Butterworth filter(fixed point version).
 *
 * based on two generic second order filters
 * with Q1 = 1.30651
 *  and Q2 = 0.541184
 *
 * http://en.wikipedia.org/wiki/Butterworth_filter
 *
 * @param filter fourth order Butterworth low pass filter structure
 * @param cut_off Cutoff frequency of the filter with -3dB
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
static inline void init_butterworth_4_low_pass_int(Butterworth4LowPass_int *filter, float cut_off, float sample_time,
    int32_t value)
{
  init_second_order_low_pass_int(&filter->lp1, cut_off, 1.30651, sample_time, value);
  init_second_order_low_pass_int(&filter->lp2, cut_off, 0.51184, sample_time, value);
}

/** Update fourth order Butterworth low pass filter state with a new value(fixed point version).
 *
 * using two cascaded second order filters
 *
 * @param filter fourth order Butterworth low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
static inline int32_t update_butterworth_4_low_pass_int(Butterworth4LowPass_int *filter, int32_t value)
{
  int32_t tmp = update_second_order_low_pass_int(&filter->lp1, value);
  return update_second_order_low_pass_int(&filter->lp2, tmp);
}

/** Get current value of the fourth order Butterworth low pass filter(fixed point version).
 *
 * @param filter fourth order Butterworth low pass filter structure
 * @return current value of the filter
 */
static inline int32_t get_butterworth_4_low_pass_int(Butterworth4LowPass_int *filter)
{
  return filter->lp2.o[0];
}

#endif

