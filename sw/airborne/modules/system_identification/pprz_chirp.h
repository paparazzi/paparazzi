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
 *
 * Mathematical implementation of the chirp
 * A "chirp" or frequency sweep is a sine wave with in time increasing frequency, and can be
 * used for system identification purposes. This registers a broad frequency spectrum.
 *
 * Chirps can be made with frequency increasing linearly or exponentially with time. This is set using an argument in
 * the chirp_init method. The latter one is better for system identification, according to [2].
 *
 * The time length of the chirp is best put at a minimum of 4 / f_minimum such that low-frequency effects
 * can be correctly discovered.
 *
 * The fade_in argument determines if the chirp fades in with increasing amplitude at
 * constant frequency (the lowest frequency). This makes it much easier to maintain the UAV around the trim position.
 *
 * Usage example:
 * // Initialize a chirp between 1 and 5 Hz during 2 seconds (for sys. id. use minimum of 4 * 1/f0)
 * struct chirp_t chirp;
 * chirp_init(&chirp, 1, 5, 2, get_current_time());
 *
 * // During flight loop, add chirp value to stick input
 * while (chirp_is_running(chirp, get_current_time())) {
 *     control[YAW] = stick_control + chirp_update(chirp, get_current_time());
 * }
 *
 * [1] https://en.wikipedia.org/wiki/Chirp for a derivation of the linear chirp
 * [2] Aircraft and Rotorcraft System Identification, 2nd edition by M. Tischler for a derivation of exponential chirp
 */

#ifndef PPRZ_CHIRP_H
#define PPRZ_CHIRP_H

#include "std.h"

/**
 * Initialize with chirp_init
 */
struct chirp_t {
  float f0_hz;
  float f1_hz;
  float start_time_s;
  float length_s; // Amount of seconds of the chirp, excluding fade-in if applicable
  float total_length_s; // Amount of seconds of the chirp, including fade-in if applicable

  float current_frequency_hz;
  float current_value; // Value is [-1, 1]
  float current_time_s;
  float percentage_done; // t / total_length: [0, 1]

  bool exponential_chirp;
  bool fade_in;
};

/**
 * Allocate and initialize a new chirp struct. set start_time to the current time
 * @param f0_hz: Minimum frequency of the chirp in Hz
 * @param f1_hz: Maximum frequency of the chirp in Hz
 * @param length_s: Time interval in s (starting from start_time_s) in which the chirp should be carried out, excluding fade-in time
 * @param current_time_s: Current time in s, starting point of the chirp
 * @param exponential_chirp: If true, use exponential-time chirp, otherwise use linear-time chirp (see wikipedia)
 * @param fade_in: If true, begin the chirp with 2 wavelengths of the lowest frequency, with increasing amplitude
 */
void chirp_init(struct chirp_t *chirp, float f0_hz, float f1_hz, float length_s, float current_time_s,
                bool exponential_chirp, bool fade_in);

/**
 * Reset the time of the chirp
 * @param chirp: The chirp struct pointer to reset
 * @param current_time_s: The time to set the chirp start at
 **/
void chirp_reset(struct chirp_t *chirp, float current_time_s);

/**
 * Return if the current_time is within the chirp manoeuvre
 */
bool chirp_is_running(struct chirp_t *chirp, float current_time_s);

/**
 * Calculate the value at current_time_s and update the struct with current frequency and value
 * @return Current value chirp->current_value
 */
float chirp_update(struct chirp_t *chirp, float current_time_s);

#endif
