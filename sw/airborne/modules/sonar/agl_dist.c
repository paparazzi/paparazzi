/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi

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

/**
 * @file modules/sonar/agl_dist.h
 *
 * Bind to agl ABI message and provide a filtered value to be used in flight plans
 *
 */

#include "modules/sonar/agl_dist.h"
#include "modules/core/abi.h"
#include "generated/airframe.h"

float agl_dist_valid;
float agl_dist_value;
float agl_dist_value_filtered;
float agl_measurement_time;

/** default sonar */
#ifndef AGL_DIST_ID
#define AGL_DIST_ID ABI_BROADCAST
#endif
#ifndef AGL_DIST_MAX_RANGE
#define AGL_DIST_MAX_RANGE 5.0f
#endif
#ifndef AGL_DIST_MIN_RANGE
#define AGL_DIST_MIN_RANGE 0.001f
#endif
#ifndef AGL_DIST_FILTER
#define AGL_DIST_FILTER 0.1f
#endif

abi_event agl_ev;

static void agl_cb(uint8_t sender_id, uint32_t stamp, float distance);

void agl_dist_init(void)
{
  agl_dist_valid = false;
  agl_dist_value = 0.f;
  agl_dist_value_filtered = 0.f;
  agl_measurement_time = get_sys_time_float();

  // Bind to AGL message
  AbiBindMsgAGL(AGL_DIST_ID, &agl_ev, agl_cb);
}

static void agl_cb(uint8_t __attribute__((unused)) sender_id, uint32_t __attribute__((unused)) stamp, float distance)
{
  if (distance < AGL_DIST_MAX_RANGE && distance > AGL_DIST_MIN_RANGE) {
    agl_dist_value = distance;
    agl_dist_valid = true;
    float now = get_sys_time_float();
    float dt = now - agl_measurement_time;
    agl_measurement_time = now;

    // update multirate exponentially weighted moving average filter
    agl_dist_value_filtered += (agl_dist_value - agl_dist_value_filtered) * dt / (AGL_DIST_FILTER + dt);
  } else {
    agl_dist_valid = false;
  }
}
