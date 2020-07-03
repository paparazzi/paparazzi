/*
 * Copyright (C) Joost Meulenbeld
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
 */
/**
 * @file "modules/system_identification/pprz_chirp.c"
 * @author Joost Meulenbeld
 * Mathematical implementation of the chirp
 */
#include "pprz_chirp.h"
#include "std.h"

// Values for exponential chirp (See ref [2] in the header file). C2 is based on C1 s.t. the frequency range exactly covers the required range
#define CHIRP_C1 4.0f
#define CHIRP_C2 1.0f / (expf(CHIRP_C1) - 1)



void chirp_init(struct chirp_t *chirp, float f0_hz, float f1_hz, float length_s, float current_time_s,
                bool exponential_chirp, bool fade_in)
{
  chirp->f0_hz = f0_hz;
  chirp->f1_hz = f1_hz;

  chirp->length_s = length_s;
  if (fade_in) { // The fade-in takes two of the longest wave-lengths, total_length is including that time
    chirp->total_length_s = length_s + 2 / f0_hz;
  } else {
    chirp->total_length_s = length_s;
  }

  chirp->start_time_s = current_time_s;
  chirp->exponential_chirp = exponential_chirp;
  chirp->fade_in = fade_in;

  chirp->current_frequency_hz = 0;
  chirp->current_value = 0;
  chirp->percentage_done = 0;
}

void chirp_reset(struct chirp_t *chirp, float current_time_s)
{
  chirp->current_time_s = current_time_s;
  chirp->start_time_s = current_time_s;
  chirp->current_frequency_hz = 0;
  chirp->current_value = 0;
  chirp->percentage_done = 0;
}

bool chirp_is_running(struct chirp_t *chirp, float current_time_s)
{
  float t = current_time_s - chirp->start_time_s;
  return (t >= 0) && (t <= chirp->total_length_s);
}

float chirp_update(struct chirp_t *chirp, float current_time_s)
{
  if (!chirp_is_running(chirp, current_time_s)) { // Outside the chirp interval, return 0
    chirp->current_value = 0.0f;
    return 0;
  }

  float t = current_time_s - chirp->start_time_s; // Time since the start of the chirp
  chirp->current_time_s = current_time_s;
  // Protect against divide by zero
  if (chirp->total_length_s <= 0) {
    chirp->total_length_s = 0.01;
  }
  chirp->percentage_done = t / chirp->total_length_s;

  if (chirp->fade_in && t < 2 / chirp->f0_hz) { // Fade-in is two times the wavelength of f0
    chirp->current_frequency_hz = chirp->f0_hz;

    // First wavelength increases linearly in amplitude, second wavelength has unity amplitude
    chirp->current_value = sinf(t * 2 * M_PI * chirp->f0_hz) * Min(1, t * chirp->f0_hz);
    return chirp->current_value;
  }

  // If the fade-in is finished, the current time t is the time since the fade-in stopped
  if (chirp->fade_in) {
    t -= 2 / chirp->f0_hz;
  }

  if (chirp->exponential_chirp) { // See the book referenced in the header for the equations
    float exponential = expf(CHIRP_C1 * t / chirp->length_s);
    float K = CHIRP_C2 * (exponential - 1);

    chirp->current_frequency_hz = chirp->f0_hz + K * (chirp->f1_hz - chirp->f0_hz);

    float theta = 2 * M_PI * (chirp->f0_hz * t
                              + (chirp->f1_hz - chirp->f0_hz) * (chirp->length_s / CHIRP_C1 * K - CHIRP_C2 * t));

    chirp->current_value = sinf(theta);
  } else { // linear-time chirp
    float k = (chirp->f1_hz - chirp->f0_hz) / chirp->length_s;

    chirp->current_frequency_hz = k * t;
    chirp->current_value = sinf(2 * M_PI * t * (chirp->f0_hz + chirp->current_frequency_hz / 2));
  }

  return chirp->current_value;
}
