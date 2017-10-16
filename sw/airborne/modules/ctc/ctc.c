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

#include "modules/ctc/ctc.h"
#include "subsystems/datalink/datalink.h" // dl_buffer
#include "subsystems/datalink/telemetry.h"
#include "subsystems/navigation/common_nav.h"
#include "autopilot.h"

#if PERIODIC_TELEMETRY
static void send_ctc(struct transport_tx *trans, struct link_device *dev)
{
  // pprz_msg_send_CTC(trans, dev, AC_ID, error);
}
#endif // PERIODIC TELEMETRY

// Control
/*! Default gain k for the algorithm */
#ifndef CTC_GAIN_K
#define CTC_GAIN_K 10
#endif
/*! Default timeout for the neighbors' information */
#ifndef CTC_TIMEOUT
#define CTC_TIMEOUT 1500
#endif

ctc_con ctc_control = {CTC_GAIN_K, CTC_TIMEOUT, 0, 0};

/*! Max expected number of aircraft */
#ifndef CTC_MAX_AC
#define CTC_MAX_AC 4
#endif
int16_t tableNei[CTC_MAX_AC][4];
uint32_t last_info[CTC_MAX_AC];
uint32_t last_transmision = 0;

void ctc_init(void)
{
  for (int i = 0; i < CTC_MAX_AC; i++) {
    tableNei[i][0] = -1;
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DCF, send_ctc);
#endif
}

bool collective_tracking_control()
{
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float x = p->x;
  float y = p->y;
  float u = 0;

  uint32_t now = get_sys_time_msec();

  for (int i = 0; i < CTC_MAX_AC; i++) {
    if (tableNei[i][0] != -1) {
      uint32_t timeout = now - last_theta[i];
      if (timeout > ctc_control.timeout) {
        tableNei[i][3] = ctc_control.timeout;
      } else {
        tableNei[i][3] = (uint16_t)timeout;

       // u += e;
      }
    }
  }

  if ((now - last_transmision > 200) && (autopilot_get_mode() == AP_MODE_AUTO2)) {
    send_theta_and_vel_to_nei();
    last_transmision = now;
  }

  return true;
}

void send_theta_and_vel_to_nei(void)
{
  struct pprzlink_msg msg;

  for (int i = 0; i < CTC_MAX_NEIGHBORS; i++)
    if (tableNei[i][0] != -1) {
      msg.trans = &(DefaultChannel).trans_tx;
      msg.dev = &(DefaultDevice).device;
      msg.sender_id = AC_ID;
      msg.receiver_id = tableNei[i][0];
      msg.component_id = 0;
      //pprzlink_msg_send_CTC_THETA_AND_VEL(&msg, &(dcf_control.theta));
    }
}

void parseRegTable(void)
{
  uint8_t ac_id = DL_DCF_REG_TABLE_ac_id(dl_buffer);
  if (ac_id == AC_ID) {
    uint8_t nei_id = DL_DCF_REG_TABLE_nei_id(dl_buffer);
    int16_t desired_sigma = DL_DCF_REG_TABLE_desired_sigma(dl_buffer);

    for (int i = 0; i < DCF_MAX_NEIGHBORS; i++)
      if (tableNei[i][0] == (int16_t)nei_id) {
        tableNei[i][0] = (int16_t)nei_id;
        tableNei[i][2] = desired_sigma;
        return;
      }

    for (int i = 0; i < DCF_MAX_NEIGHBORS; i++)
      if (tableNei[i][0] == -1) {
        tableNei[i][0] = (int16_t)nei_id;
        tableNei[i][2] = desired_sigma;
        return;
      }
  }
}

void parseCleanTable(void)
{
  uint8_t ac_id = DL_DCF_REG_TABLE_ac_id(dl_buffer);
  if (ac_id == AC_ID)
    for (int i = 0; i < DCF_MAX_NEIGHBORS; i++)
      tableNei[i][0] = -1;
}

void parseThetaAndVelTable(void)
{
  int16_t sender_id = (int16_t)(SenderIdOfPprzMsg(dl_buffer));
  for (int i = 0; i < DCF_MAX_NEIGHBORS; i++)
    if (tableNei[i][0] == sender_id) {
      last_theta[i] = get_sys_time_msec();
      tableNei[i][1] = (int16_t)((DL_DCF_THETA_theta(dl_buffer)) * 1800 / M_PI);
      break;
    }
}
