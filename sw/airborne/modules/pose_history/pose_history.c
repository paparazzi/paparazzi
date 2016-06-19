/*
 * Copyright (C) Roland Meertens
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
 * @file "modules/pose_history/pose_history.c"
 * @author Roland Meertens
 * Ask this module for the pose the drone had closest to a given timestamp
 */

#include "modules/pose_history/pose_history.h"
#include <sys/time.h>
#include "mcu_periph/sys_time.h"
#include "state.h"
#if LINUX
#include <pthread.h>
#endif

#ifndef POSE_HISTORY_SIZE
#define POSE_HISTORY_SIZE 1024
#endif

typedef struct {
  uint32_t ring_index;
  uint32_t ring_size;
  timeAndRotation ring_data[POSE_HISTORY_SIZE];
} rotationHistoryRingBuffer;

rotationHistoryRingBuffer location_history;

#if LINUX
pthread_mutex_t pose_mutex;
#endif



void float_eulers_copy(struct FloatEulers *to, struct FloatEulers *from);
void float_eulers_copy(struct FloatEulers *to, struct FloatEulers *from)
{
  to->phi = from->phi;
  to->theta = from->theta;
  to->psi = from->psi;
}

void float_rates_copy(struct FloatRates *to, struct FloatRates *from);
void float_rates_copy(struct FloatRates *to, struct FloatRates *from)
{
  to->p = from->p;
  to->q = from->q;
  to->r = from->r;
}

/**
 * Given a pprz timestamp in used (obtained with get_sys_time_usec) we return the pose in FloatEulers closest to that time.
 */
timeAndRotation get_rotation_at_timestamp(uint32_t timestamp)
{
#if LINUX
  pthread_mutex_lock(&pose_mutex);
#endif
  uint32_t index_history = 0;
  uint32_t closestTimeDiff = abs(timestamp - location_history.ring_data[0].timestamp);
  uint32_t closestIndex = 0;

  // Seach for the timestamp closes to the timestamp argument of this function.
  for (index_history = 0; index_history < location_history.ring_size; index_history++) {
    uint32_t diff = abs(timestamp - location_history.ring_data[index_history].timestamp);
    if (diff < closestTimeDiff) {
      closestIndex = index_history;
      closestTimeDiff = diff;
    }
  }

  // Save the pose closest to the given timestamp and return this
  timeAndRotation closest_pose;
  float_eulers_copy(&closest_pose.rotation, &location_history.ring_data[closestIndex].rotation);
  float_rates_copy(&closest_pose.rates, &location_history.ring_data[closestIndex].rates);

#if LINUX
  pthread_mutex_unlock(&pose_mutex);
#endif
  return closest_pose;
}

/**
 * Initialises the pose history
 */
void pose_init()
{
  location_history.ring_index = 0;
  location_history.ring_size = POSE_HISTORY_SIZE;
}


/**
 * Records the pose history 512 times per second. Time gets saved in pprz usec, obtained with get_sys_time_usec();
 */
void pose_periodic()
{
  uint32_t now_ts = get_sys_time_usec();
#if LINUX
  pthread_mutex_lock(&pose_mutex);
#endif
  timeAndRotation *current_time_and_rotation = &location_history.ring_data[location_history.ring_index];
  float_eulers_copy(&current_time_and_rotation->rotation, stateGetNedToBodyEulers_f());
  float_rates_copy(&current_time_and_rotation->rates, stateGetBodyRates_f());

  current_time_and_rotation->timestamp = now_ts;

  // increase index location history
  location_history.ring_index = (location_history.ring_index + 1) % location_history.ring_size;

#if LINUX
  pthread_mutex_unlock(&pose_mutex);
#endif
}


