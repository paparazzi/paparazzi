/*
 * Copyright (C) 2017 Hector Garcia de Marina
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

#include <math.h>
#include <std.h>
#include <stdio.h>

#include "modules/multi/ctc/ctc_target.h"
//#include "subsystems/datalink/datalink.h" // dl_buffer
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/navigation.h"
#include "autopilot.h"

/*! Max expected number of aircraft */
#ifndef CTC_MAX_AC
#define CTC_MAX_AC 4
#endif

int16_t tableNei[CTC_MAX_AC][6];

void ctc_target_init(void)
{
  for (int i = 0; i < CTC_MAX_AC; i++) {
    tableNei[i][0] = -1;
  }
}

void ctc_target_send_info_to_nei(void)
{
  struct EnuCoor_f *v = stateGetSpeedEnu_f();
  struct EnuCoor_f *p = stateGetPositionEnu_f();

  float vx = v->x;
  float vy = v->y;
  float px = p->x;
  float py = p->y;

  struct pprzlink_msg msg;

  for (int i = 0; i < CTC_MAX_AC; i++)
    if (tableNei[i][0] != -1) {
      msg.trans = &(DefaultChannel).trans_tx;
      msg.dev = &(DefaultDevice).device;
      msg.sender_id = AC_ID;
      msg.receiver_id = tableNei[i][0];
      msg.component_id = 0;
      pprzlink_msg_send_CTC_INFO_FROM_TARGET(&msg, &px, &py, &vx, &vy);
    }
}

void parse_ctc_target_RegTable(void)
{
  uint8_t ac_id = DL_CTC_REG_TABLE_ac_id(dl_buffer);
  if (ac_id == AC_ID) {
    uint8_t nei_id = DL_CTC_REG_TABLE_nei_id(dl_buffer);
    for (int i = 0; i < CTC_MAX_AC; i++)
      if (tableNei[i][0] == -1) {
        tableNei[i][0] = (int16_t)nei_id;
        return;
      }
  }
}

void parse_ctc_target_CleanTable(void)
{
  uint8_t ac_id = DL_CTC_REG_TABLE_ac_id(dl_buffer);
  if (ac_id == AC_ID)
    for (int i = 0; i < CTC_MAX_AC; i++) {
      tableNei[i][0] = -1;
    }
}

