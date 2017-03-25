/*
 * Copyright (C) 2016  2017 Michal Podhradsky <http://github.com/podhrmic>
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
/**
 * @file "modules/cartography/isaac.c"
 *
 *  ISaAC: The Intelligent Safety and Airworthiness Co-Pilot module
 *  Based on paper "A Payload Verification and Management Framework
 *  for Small UAV-based Personal Remote Sensing Systems" by Cal Coopmans
 *  and Chris Coffin. Link: http://ieeexplore.ieee.org/abstract/document/6309316/
 *
 *  ISaAC is intented mainly for mapping applications.
 *
 *  This module processes messages from ISaAC, and either forwards them to the GCS
 *  (such as CAMERA_SNAPSHOT or CAMERA_PAYLOAD messages), or responds to them necessary
 *  (such as MOVE_WP).
 *
 *  The module assumes the source of the messages is trusted (i.e. not authentication besides
 *  AC_ID check is performed).
 */

#include "modules/cartography/isaac.h"
#include "subsystems/datalink/telemetry.h"
#include <string.h>

bool send_cam_snapshot;
bool send_cam_payload;

struct CameraPayload cam_payload;
struct CameraSnapshot cam_snapshot;

/** Init function */
void isaac_init(void)
{
  send_cam_snapshot = false;
  send_cam_payload = false;

  memset(&cam_payload, 0, sizeof(cam_payload));
  memset(&cam_snapshot, 0, sizeof(cam_snapshot));
}

/** Periodic function */
void isaac_periodic(void)
{
  if (send_cam_snapshot)
  {
    // send down to GCS
    DOWNLINK_SEND_CAMERA_SNAPSHOT(DefaultChannel, DefaultDevice,
        &cam_snapshot.cam_id,
        &cam_snapshot.cam_state,
        &cam_snapshot.snapshot_num,
        &cam_snapshot.snapshot_valid,
        &cam_snapshot.lens_temp);

    send_cam_snapshot = false;
  }

  if (send_cam_payload)
  {
    // send down to GCS
    DOWNLINK_SEND_CAMERA_PAYLOAD(DefaultChannel, DefaultDevice,
        &cam_payload.timestamp,
        &cam_payload.used_mem,
        &cam_payload.used_disk,
        &cam_payload.door_status,
        &cam_payload.error_code);

    send_cam_payload = false;
  }
}

/** Message processing functions
 * It would be nice to be able to have the dl buffer passed as
 * an argument to the parsing function, but for now we assume that
 * the message always comes from the extra_dl_buffer
 *
 * In case of multiple cameras, it is up to the payload computer to send
 * CAMERA_SNAPSHOT messages for each camera at proper interval, so the values
 * don't get overwritten.
 *
 * TODO: protect with mutexes?
 * */
void isaac_parse_cam_snapshot_dl(void)
{
  // copy CAMERA_SNAPSHOT message and mark it to be sent
  cam_snapshot.cam_id = DL_CAMERA_SNAPSHOT_camera_id(extra_dl_buffer);
  cam_snapshot.cam_state = DL_CAMERA_SNAPSHOT_camera_state(extra_dl_buffer);
  cam_snapshot.snapshot_num = DL_CAMERA_SNAPSHOT_snapshot_image_number(extra_dl_buffer);
  cam_snapshot.snapshot_valid = DL_CAMERA_SNAPSHOT_valid_snapshot(extra_dl_buffer);
  cam_snapshot.lens_temp = DL_CAMERA_SNAPSHOT_lens_temp(extra_dl_buffer);
}

void isaac_parse_cam_payload_dl(void){
  // copy CAMERA_PAYLOAD message and mark it to be sent
  cam_payload.timestamp = DL_CAMERA_PAYLOAD_timestamp(extra_dl_buffer);
  cam_payload.used_mem = DL_CAMERA_PAYLOAD_used_memory(extra_dl_buffer);
  cam_payload.used_disk = DL_CAMERA_PAYLOAD_used_disk(extra_dl_buffer);
  cam_payload.door_status = DL_CAMERA_PAYLOAD_door_status(extra_dl_buffer);
  cam_payload.error_code = DL_CAMERA_PAYLOAD_error_code(extra_dl_buffer);

  send_cam_payload = true;
}
