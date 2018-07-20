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
 * @file "modules/mission/copilot.h"
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

#ifndef COPILOT_H
#define COPILOT_H

#include "subsystems/datalink/datalink.h"
#include "modules/datalink/extra_pprz_dl.h"
#include "pprz_mutex.h"

PPRZ_MUTEX_DECL(copilot_cam_snapshot_mtx);
PPRZ_MUTEX_DECL(copilot_cam_payload_mtx);
PPRZ_MUTEX_DECL(copilot_status_mtx);

struct CameraPayload {
  float timestamp;
  uint8_t used_mem;
  uint8_t used_disk;
  uint8_t door_status;
  uint8_t error_code;
};

struct CameraSnapshot {
  uint16_t cam_id;
  uint8_t cam_state;
  uint16_t snapshot_num;
  uint8_t snapshot_valid;
  float lens_temp;
  float array_temp;
};

struct CopilotStatus {
  float timestamp;
  uint8_t used_mem;
  uint8_t used_disk;
  uint8_t status;
  uint8_t error_code;
};

extern bool send_cam_snapshot;
extern bool send_cam_payload;
extern bool send_copilot_status;

/** Init function */
void copilot_init(void);

/** Periodic function */
void copilot_periodic(void);

/** Message processing functions */
void copilot_parse_cam_snapshot_dl(uint8_t *buf);
void copilot_parse_cam_payload_dl(uint8_t *buf);
void copilot_parse_copilot_status_dl(uint8_t *buf);
void copilot_parse_move_wp_dl(uint8_t *buf);
void copilot_parse_payload_command_dl(uint8_t *buf);

#endif /* COPILOT_H */

