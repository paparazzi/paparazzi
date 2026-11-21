/*
 * Copyright (C) 2026 Paparazzi contributors
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/** @file filters/notch_filter_tpt.h
 * Weighted second-order notch using a topology-preserving transform (TPT).
 *
 * The analogue prototype is H(s) = 1 - weight * (s / Q) / (s*s + s / Q + 1).
 * The bilinear transform uses g = tan(pi * frequency / sample_frequency).
 * Q is dimensionless and remains constant when retuning with the same Q.
 * Digital bandwidth is warped near Nyquist; frequency / Q is an approximation.
 *
 * Coefficients can be shared by sensor axes with the same sampling frequency.
 * Each axis, sensor and cascaded notch needs a separate state. Reconfigure
 * coefficients at the tracking rate; apply the filter to every sensor sample.
 * RPM smoothing, harmonics, frequency limits and fade scheduling belong to
 * the caller. Gyro and accel streams at different rates need separate coefficients.
 *
 * Example (one axis):
 *   struct TptNotchCoefficients coeff;
 *   struct TptNotchState state;
 *   tpt_notch_init(&coeff, 100.0f, 5.0f, 1000.0f, 1.0f);
 *   tpt_notch_reset(&state, initial_sample);
 *   output = tpt_notch_apply(&coeff, &state, input);
 */

#ifndef NOTCH_FILTER_TPT_H
#define NOTCH_FILTER_TPT_H

#include "std.h"

struct TptNotchCoefficients {
  float g;
  float a1;
  float a2;
  float weighted_inv_q;
};

struct TptNotchState {
  float ic1;
  float ic2;
};

/** Retune without resetting state. Invalid arguments leave coefficients unchanged.
 *
 * @param frequency centre frequency [Hz], strictly between zero and Nyquist
 * @param q positive, dimensionless quality factor
 * @param sample_frequency positive sensor sampling frequency [Hz]
 * @param weight attenuation weight in [0, 1]: zero bypasses, one gives a full notch
 * @return true on success, false for invalid or unrepresentable coefficients
 *
 * The caller must synchronize coefficient updates with sample processing.
 */
static inline bool tpt_notch_set_parameters(struct TptNotchCoefficients *coeff,
    float frequency, float q, float sample_frequency, float weight)
{
  if (!isfinite(frequency) || !isfinite(q) || !isfinite(sample_frequency) || !isfinite(weight) ||
      frequency <= 0.0f || q <= 0.0f || sample_frequency <= 0.0f || weight < 0.0f || weight > 1.0f) {
    return false;
  }
  const float normalized_frequency = frequency / sample_frequency;
  if (normalized_frequency <= 0.0f || normalized_frequency >= 0.5f) {
    return false;
  }
  const float g = tanf((float)M_PI * normalized_frequency);
  const float inv_q = 1.0f / q;
  const float denominator = 1.0f + g * (g + inv_q);
  if (!isfinite(inv_q) || !isfinite(denominator)) {
    return false;
  }
  coeff->g = g;
  coeff->a1 = 1.0f / denominator;
  coeff->a2 = g * coeff->a1;
  coeff->weighted_inv_q = weight * inv_q;
  return true;
}

/** Initialize coefficients. Invalid parameters initialize an identity filter.
 * @return true if the supplied parameters were accepted
 */
static inline bool tpt_notch_init(struct TptNotchCoefficients *coeff,
    float frequency, float q, float sample_frequency, float weight)
{
  *coeff = (struct TptNotchCoefficients) {0};
  return tpt_notch_set_parameters(coeff, frequency, q, sample_frequency, weight);
}

/** Initialize or reset an axis at a finite, steady input value (zero if unknown). */
static inline void tpt_notch_reset(struct TptNotchState *state, float value)
{
  state->ic1 = 0.0f;
  state->ic2 = value;
}

/** Process one finite sample. State continues tracking even at zero weight,
 * so fading back in does not resume a stale state. No trigonometry is used here.
 */
static inline float tpt_notch_apply(const struct TptNotchCoefficients *coeff,
    struct TptNotchState *state, float input)
{
  const float v3 = input - state->ic2;
  const float v1 = coeff->a1 * state->ic1 + coeff->a2 * v3;
  const float v2 = state->ic2 + coeff->g * v1;
  state->ic1 = 2.0f * v1 - state->ic1;
  state->ic2 = 2.0f * v2 - state->ic2;
  return input - coeff->weighted_inv_q * v1;
}

#endif // NOTCH_FILTER_TPT_H
