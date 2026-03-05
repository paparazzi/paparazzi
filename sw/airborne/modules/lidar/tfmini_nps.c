/*
 * Copyright (C) 2025 Alejandro Rochas <alrochas@ucm.es>
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
 *
 */

/** @file modules/lidar/tfmini_nps.c
 *  @brief simulation of a lidar
 *
 */

#include "tfmini.h"
#include "state.h"

// Messages
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "modules/core/abi.h"

#define LIDAR_MIN_RANGE 0.1
#define LIDAR_MAX_RANGE 12.0

struct TFMini tfmini;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

/**
 * Downlink message lidar
 */
static void tfmini_send_lidar(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t status = (uint8_t) tfmini.parse_status;
  pprz_msg_send_LIDAR(trans, dev, AC_ID,
                      &tfmini.distance,
                      &tfmini.mode,
                      &status);
}

#endif



/**
 * Initialization function
 */
void tfmini_init(void)
{
  tfmini.distance = 0;
  tfmini.device = &((TFMINI_PORT).device);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LIDAR, tfmini_send_lidar);
#endif
}


void tfmini_event(void)
{
  sim_overwrite_lidar();
}



// If you want to use the lidar simulation, you need to use the lidar correction module
void sim_overwrite_lidar(void)
{
  if (!stateIsLocalCoordinateValid()) { return; }

  // Convert GPS position to NED coordinates
  struct FloatVect2 pos = {0.0f, 0.0f};
  struct NedCoor_i ned = {0.0f, 0.0f, 0.0f};
  ned_of_lla_point_i(&ned, stateGetNedOrigin_i(), &gps.lla_pos);
  pos.x = (float)(ned.y / 100.0f);
  pos.y = (float)(ned.x / 100.0f);

  // Calculate the angle of the rover (and servo if used)
  float theta = M_PI / 2 - (stateGetNedToBodyEulers_f()->psi);
#ifdef USE_SERVO_LIDAR
  theta = theta - servoLidar.angle * M_PI / 180;
#endif

  float min_distance = FLT_MAX;

  // Traverse the wall array and store the minimum distance (!= 0)
  for (uint8_t w = 0; w < wall_system.wall_count; w++) {
    struct Wall *wall = &wall_system.walls[w];
    for (uint8_t p = 0; p < wall->count - 1; p++) {
      struct FloatVect2 p1 = wall->points_ltp[p];
      struct FloatVect2 p2 = wall->points_ltp[p + 1];
      float distance = distance_to_wall(theta, &pos, &p1, &p2);
      if (distance < min_distance && distance > 0) {
        min_distance = distance;
      }
    }
  }

  // Store that data in the variable tfmini.distance
  setLidarDistance_f(min_distance);
}


/**
 * Set the distance of the lidar
 * This function is used in NPS to set the distance of the lidar
 */
void setLidarDistance_f(float distance)
{
  if (distance < LIDAR_MIN_RANGE || distance > LIDAR_MAX_RANGE) {
    distance = 0;
  }
  tfmini.distance = distance;
  tfmini_send_abi();
}


// Send the lidar message (if requested, OBSTACLE_DETECTION)
void tfmini_send_abi(void)
{
#ifndef USE_SERVO_LIDAR
  //send message (if there is not servo module)
  AbiSendMsgOBSTACLE_DETECTION(AGL_LIDAR_TFMINI_ID, tfmini.distance, 0, 0);
#endif
}
