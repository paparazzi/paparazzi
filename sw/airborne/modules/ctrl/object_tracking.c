/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/ctrl/object_tracking.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Control a rotorcraft heading to track an object detected by a camera
 */

#include "modules/ctrl/object_tracking.h"

#include "firmwares/rotorcraft/navigation.h"
#include "modules/core/abi.h"
#include "generated/airframe.h"
#include "generated/modules.h"

// ABI message binding ID
#ifndef OBJECT_TRACKING_ID
#define OBJECT_TRACKING_ID ABI_BROADCAST
#endif

// Timout in seconds before entering search mode
#ifndef OBJECT_TRACKING_TIMEOUT
#define OBJECT_TRACKING_TIMEOUT 3.0f
#endif

// Turn rate in tracking mode (rad/s)
#ifndef OBJECT_TRACKING_RATE
#define OBJECT_TRACKING_RATE RadOfDeg(10)
#endif

// Turn rate in search mode (rad/s)
#ifndef OBJECT_TRACKING_SEARCH_RATE
#define OBJECT_TRACKING_SEARCH_RATE RadOfDeg(20)
#endif

// Send debug message
#ifndef OBJECT_TRACKING_DEBUG
#define OBJECT_TRACKING_DEBUG FALSE
#endif

#if OBJECT_TRACKING_DEBUG
#include "modules/datalink/downlink.h"
#include "pprzlink/messages.h"
#include "mcu_periph/uart.h"
#endif

float object_tracking_rate; // in rad/s
float object_tracking_search_rate; // in rad/s

static uint8_t object_frame; // relative or global coordinates
static float object_bearing;
static float object_height;
static float timeout;

abi_event object_ev;

static const float nav_dt = 1.f / NAVIGATION_FREQUENCY;

// callback on follow target message
static void get_object(uint8_t sender_id __attribute__((unused)),
    uint32_t id __attribute__((unused)),
    uint8_t frame, float bearing, float height,
    float distance __attribute__((unused)))
{
  object_frame = frame;
  object_bearing = bearing;
  object_height = height;
  timeout = 0.f;
}

void object_tracking_init(void)
{
  object_tracking_search_rate = OBJECT_TRACKING_SEARCH_RATE;
  object_tracking_rate = OBJECT_TRACKING_RATE;

  object_frame = 0; // relative coordinates
  object_bearing = 0.f;
  object_height = 0.f;
  timeout = OBJECT_TRACKING_TIMEOUT; // start in search mode

  // Bind to camera message
  AbiBindMsgFOLLOW_TARGET(OBJECT_TRACKING_ID, &object_ev, get_object);
}

void object_tracking_run(void)
{
  if (timeout < OBJECT_TRACKING_TIMEOUT) {
    timeout += nav_dt;
    // compute expected heading
    float target_heading = object_bearing;
    if (!bit_is_set(object_frame, 0)) {
      target_heading += stateGetNedToBodyEulers_f()->psi; // relative frame
    }
    float diff = target_heading - nav.heading;
    FLOAT_ANGLE_NORMALIZE(diff);
    BoundAbs(diff, object_tracking_rate * nav_dt)
    nav_heading += ANGLE_BFP_OF_REAL(diff);
#if OBJECT_TRACKING_DEBUG
    float msg[] = {
      target_heading,
      object_bearing,
      object_height,
      timeout
    };
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 4, msg);
#endif
  } else {
    nav_heading += ANGLE_BFP_OF_REAL(object_tracking_search_rate * nav_dt);
  }
  INT32_COURSE_NORMALIZE(nav_heading);
}

