/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file modules/digital_cam/pprzlink_cam_ctrl.h
 *  @brief Digital Camera Control with PPRZLINK messages
 */

#include "modules/digital_cam/pprzlink_cam_ctrl.h"
#include "generated/airframe.h"
#include "generated/modules.h"

// Include Standard Camera Control Interface
#include "dc.h"

float digital_cam_exposure;

void pprzlink_cam_ctrl_init(void)
{
  digital_cam_exposure = 0.f; // auto expo
}

void pprzlink_cam_ctrl_periodic(void)
{
  // Common DC Periodic task
  dc_periodic();
}

/* Command The Camera */
void dc_send_command(uint8_t cmd)
{

  if (cmd == DC_SHOOT) {
    dc_send_shot_position();
  }

  // call command send_command function
  dc_send_command_common(cmd);
}

#include "modules/datalink/downlink.h"
#include "modules/datalink/extra_pprz_dl.h"

void pprzlink_cam_ctrl_set_expo(float expo)
{
  digital_cam_exposure = expo;
  uint8_t tab[2];
  tab[0] = 'e';
  tab[1] = (uint8_t)(expo * 10.f);
  uint8_t dst_id = 0;
  DOWNLINK_SEND_PAYLOAD_COMMAND(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE, &dst_id, 2, tab);
}

void dc_expo_cb(uint8_t* buf) {
  if (DL_PAYLOAD_COMMAND_ac_id(buf) != AC_ID) { return; }

  // feedback from camera
  if (DL_PAYLOAD_COMMAND_command_length(buf) == 2 && DL_PAYLOAD_COMMAND_command(buf)[0] == 'e') {
    digital_cam_exposure = DL_PAYLOAD_COMMAND_command(buf)[1] / 10.0;
  }
}

