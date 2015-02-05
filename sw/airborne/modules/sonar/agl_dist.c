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
 * Bind to sonar ABI message and provide a filtered value to be used in flight plans
 *
 */

#include "modules/sonar/agl_dist.h"
#include "subsystems/abi.h"
#include "generated/airframe.h"

float agl_dist_valid;
float agl_dist_value;
float agl_dist_value_filtered;

/** default sonar */
#ifndef AGL_DIST_SONAR_ID
#define AGL_DIST_SONAR_ID ABI_BROADCAST
#endif
#ifndef AGL_DIST_SONAR_MAX_RANGE
#define AGL_DIST_SONAR_MAX_RANGE 5.0
#endif
#ifndef AGL_DIST_SONAR_MIN_RANGE
#define AGL_DIST_SONAR_MIN_RANGE 0.001
#endif
#ifndef AGL_DIST_SONAR_FILTER
#define AGL_DIST_SONAR_FILTER 5
#endif

abi_event sonar_ev;

static void sonar_cb(uint8_t sender_id, float distance);

void agl_dist_init(void)
{
  agl_dist_valid = FALSE;
  agl_dist_value = 0.;
  agl_dist_value_filtered = 0.;

  // Bind to AGL message
  AbiBindMsgAGL(AGL_DIST_SONAR_ID, &sonar_ev, sonar_cb);
}


static void sonar_cb(uint8_t __attribute__((unused)) sender_id, float distance)
{
  if (distance < AGL_DIST_SONAR_MAX_RANGE && distance > AGL_DIST_SONAR_MIN_RANGE) {
    agl_dist_value = distance;
    agl_dist_valid = TRUE;
    agl_dist_value_filtered = (AGL_DIST_SONAR_FILTER * agl_dist_value_filtered + agl_dist_value) /
                              (AGL_DIST_SONAR_FILTER + 1);
  } else {
    agl_dist_valid = FALSE;
  }
}

