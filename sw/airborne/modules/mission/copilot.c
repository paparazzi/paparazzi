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
 * @file "modules/mission/copilot.c"
 *
 *  Mission Computer module, interfacing the mission computer (also known as Copilot),
 *  based losely on
 *  ISaAC: The Intelligent Safety and Airworthiness Co-Pilot module
 *  Based on paper "A Payload Verification and Management Framework
 *  for Small UAV-based Personal Remote Sensing Systems" by Cal Coopmans
 *  and Chris Coffin. Link: http://ieeexplore.ieee.org/abstract/document/6309316/
 *
 *  More info can be found on http://wiki.paparazziuav.org/wiki/Mission_computer
 *
 *  Copilot is intended mainly for mapping applications.
 *
 *   This module processes messages from Copilot, and either forwards them to the GCS
 *  (such as CAMERA_SNAPSHOT or CAMERA_PAYLOAD messages), or responds to them as necessary
 *  (such as MOVE_WP).
 *
 *  The module assumes the source of the messages is trusted (i.e. not authentication besides
 *  AC_ID check is performed).
 */

#include "modules/mission/copilot.h"
#include "subsystems/datalink/telemetry.h"
#include <string.h>

#include "pprz_mutex.h"

// needed for WP_MOVED confirmation
#include "firmwares/fixedwing/nav.h"
#include "subsystems/navigation/common_nav.h"
#include "math/pprz_geodetic_float.h"

bool send_cam_snapshot;
bool send_cam_payload;
bool send_copilot_status;

struct CameraPayload cam_payload;
struct CameraSnapshot cam_snapshot;
struct CopilotStatus copilot_status;

PPRZ_MUTEX(copilot_cam_snapshot_mtx);
PPRZ_MUTEX(copilot_cam_payload_mtx);
PPRZ_MUTEX(copilot_status_mtx);

/** Init function */
void copilot_init(void)
{
  send_cam_snapshot = false;
  send_cam_payload = false;
  send_copilot_status = false;

  memset(&cam_payload, 0, sizeof(cam_payload));
  memset(&cam_snapshot, 0, sizeof(cam_snapshot));
  memset(&copilot_status, 0, sizeof(copilot_status));

  PPRZ_MUTEX_INIT(copilot_cam_snapshot_mtx);
  PPRZ_MUTEX_INIT(copilot_cam_payload_mtx);
  PPRZ_MUTEX_INIT(copilot_status_mtx);
}

/** Periodic function */
void copilot_periodic(void)
{
  PPRZ_MUTEX_LOCK(copilot_cam_snapshot_mtx);
  if (send_cam_snapshot)
  {
    // send down to GCS
    DOWNLINK_SEND_CAMERA_SNAPSHOT(DefaultChannel, DefaultDevice,
        &cam_snapshot.cam_id,
        &cam_snapshot.cam_state,
        &cam_snapshot.snapshot_num,
        &cam_snapshot.snapshot_valid,
        &cam_snapshot.lens_temp,
        &cam_snapshot.array_temp);

    send_cam_snapshot = false;
  }
  PPRZ_MUTEX_UNLOCK(copilot_cam_snapshot_mtx);


  PPRZ_MUTEX_LOCK(copilot_cam_payload_mtx);
  if (send_cam_payload)
  {
    // NOTE: to send the message over the EXTRA_DL port
    // use "DOWNLINK_SEND_CAMERA_PAYLOAD(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE,"

    // send down to GCS
    DOWNLINK_SEND_CAMERA_PAYLOAD(DefaultChannel, DefaultDevice,
        &cam_payload.timestamp,
        &cam_payload.used_mem,
        &cam_payload.used_disk,
        &cam_payload.door_status,
        &cam_payload.error_code);

    send_cam_payload = false;
  }
  PPRZ_MUTEX_UNLOCK(copilot_cam_payload_mtx);

  PPRZ_MUTEX_LOCK(copilot_status_mtx);
  // send down to GCS
  if (send_copilot_status)
  {
    DOWNLINK_SEND_COPILOT_STATUS(DefaultChannel, DefaultDevice,
        &copilot_status.timestamp,
        &copilot_status.used_mem,
        &copilot_status.used_disk,
        &copilot_status.status,
        &copilot_status.error_code);

    send_copilot_status = false;
  }
  PPRZ_MUTEX_UNLOCK(copilot_status_mtx);

}

/**
 *
 * copy CAMERA_SNAPSHOT message and mark it to be sent
 *
 * In case of multiple cameras, it is up to the payload computer to send
 * CAMERA_SNAPSHOT messages for each camera at proper interval, so the values
 * don't get overwritten.
 *
 */
void copilot_parse_cam_snapshot_dl(uint8_t *buf)
{
  PPRZ_MUTEX_LOCK(copilot_cam_snapshot_mtx);

  // copy CAMERA_SNAPSHOT message and mark it to be sent
  cam_snapshot.cam_id = DL_CAMERA_SNAPSHOT_DL_camera_id(buf);
  cam_snapshot.cam_state = DL_CAMERA_SNAPSHOT_DL_camera_state(buf);
  cam_snapshot.snapshot_num = DL_CAMERA_SNAPSHOT_DL_snapshot_image_number(buf);
  cam_snapshot.snapshot_valid = DL_CAMERA_SNAPSHOT_DL_snapshot_valid(buf);
  cam_snapshot.lens_temp = DL_CAMERA_SNAPSHOT_DL_lens_temp(buf);
  cam_snapshot.array_temp = DL_CAMERA_SNAPSHOT_DL_array_temp(buf);

  send_cam_snapshot = true;

  PPRZ_MUTEX_UNLOCK(copilot_cam_snapshot_mtx);
}

/**
 * copy CAMERA_PAYLOAD message and mark it to be sent
 */
void copilot_parse_cam_payload_dl(uint8_t *buf)
{
  PPRZ_MUTEX_LOCK(copilot_cam_payload_mtx);

  cam_payload.timestamp = DL_CAMERA_PAYLOAD_DL_timestamp(buf);
  cam_payload.used_mem = DL_CAMERA_PAYLOAD_DL_used_memory(buf);
  cam_payload.used_disk = DL_CAMERA_PAYLOAD_DL_used_disk(buf);
  cam_payload.door_status = DL_CAMERA_PAYLOAD_DL_door_status(buf);
  cam_payload.error_code = DL_CAMERA_PAYLOAD_DL_error_code(buf);

  send_cam_payload = true;

  PPRZ_MUTEX_UNLOCK(copilot_cam_payload_mtx);
}

/**
 * copy COPILOT_STATUS message and mark it to be sent
 */
void copilot_parse_copilot_status_dl(uint8_t *buf)
{
  PPRZ_MUTEX_LOCK(copilot_status_mtx);

  copilot_status.timestamp = DL_COPILOT_STATUS_DL_timestamp(buf);
  copilot_status.used_mem = DL_COPILOT_STATUS_DL_used_memory(buf);
  copilot_status.used_disk = DL_COPILOT_STATUS_DL_used_disk(buf);
  copilot_status.status = DL_COPILOT_STATUS_DL_status(buf);
  copilot_status.error_code = DL_COPILOT_STATUS_DL_error_code(buf);

  send_copilot_status = true;

  PPRZ_MUTEX_UNLOCK(copilot_status_mtx);
}

/**
 * If MOVE_WP from GCS
 *  - processed in  firmware_parse_msg(dev, trans, buf); with regular buffer
 *  - reponse over telemetry (regular buffer)
 *  - here send WP_MOVED over extra_dl
 *
 *  If MOVE_WP from extra_dl
 *  - processed in firmware_parse_msg(dev, trans, buf); with extra buffer
 *  - response over extra_dl
 *  - send an update to GCS
 *
 *  In both cases, the MOVE_WP message was already processed in firmware_parse
 *  here we are taking care only about propagating the change
 *
 */
void copilot_parse_move_wp_dl(uint8_t *buf)
{
  if (DL_MOVE_WP_ac_id(buf) == AC_ID) {
    uint8_t wp_id = DL_MOVE_WP_wp_id(buf);

    /* Computes from (lat, long) in the referenced UTM zone */
    struct LlaCoor_f lla;
    lla.lat = RadOfDeg((float)(DL_MOVE_WP_lat(buf) / 1e7));
    lla.lon = RadOfDeg((float)(DL_MOVE_WP_lon(buf) / 1e7));
    lla.alt = ((float)(DL_MOVE_WP_alt(buf)))/1000.;
    struct UtmCoor_f utm;
    utm.zone = nav_utm_zone0;
    utm_of_lla_f(&utm, &lla);

    // Waypoint range is limited. Computes the UTM pos back from the relative
    // coordinates */
    utm.east = waypoints[wp_id].x + nav_utm_east0;
    utm.north = waypoints[wp_id].y + nav_utm_north0;

    if (buf == extra_dl_buffer) {
       // MOVE_WP came from extra_dl, respond over telemetry
      DOWNLINK_SEND_WP_MOVED(DefaultChannel, DefaultDevice,
                             &wp_id, &utm.east, &utm.north, &utm.alt, &nav_utm_zone0);
    }

    if (buf == dl_buffer) {
      // MOVE_WP came over telemetry, respond over extra_dl
      DOWNLINK_SEND_WP_MOVED(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE,
                             &wp_id, &utm.east, &utm.north, &utm.alt, &nav_utm_zone0);
    }

  }
}
