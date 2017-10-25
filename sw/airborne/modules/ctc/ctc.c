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
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "autopilot.h"

/*! Max expected number of aircraft */
#ifndef CTC_MAX_AC
#define CTC_MAX_AC 4
#endif

#if PERIODIC_TELEMETRY
static void send_ctc(struct transport_tx *trans, struct link_device *dev)
{
   pprz_msg_send_CTC(trans, dev, AC_ID, 4*CTC_MAX_AC, &(tableNei[0][0]));
}
#endif // PERIODIC TELEMETRY

// Control
/*! Default gain k for the algorithm */
#ifndef CTC_GAIN_K
#define CTC_GAIN_K 0.05
#endif
/*! Default timeout in ms for the neighbors' information */
#ifndef CTC_TIMEOUT
#define CTC_TIMEOUT 1500
#endif
/*! Default time in ms for broadcasting information */
#ifndef CTC_TIME_BROAD
#define CTC_TIME_BROAD 200
#endif

ctc_con ctc_control = {CTC_GAIN_K, CTC_TIMEOUT, 0, 0, 0, 0, 0, 0, 0, 0, CTC_TIME_BROAD};

int16_t tableNei[CTC_MAX_AC][6];
uint32_t last_info[CTC_MAX_AC];
uint32_t last_transmision = 0;
float ctc_error_to_target = 0;

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
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct EnuCoor_f *v = stateGetSpeedEnu_f();
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float vx = v->x;
  float vy = v->y;
  float p_centroid_x = p->x;
  float p_centroid_y = p->y;

  ctc_control.speed = sqrtf(vx*vx + vy*vy);
  ctc_control.theta = atan2f(vy, vx);
  ctc_control.px = p->x;
  ctc_control.py = p->y;

  float u_vel = 0;
  float u_spa = 0;

  uint32_t now = get_sys_time_msec();

  int num_neighbors = 0;

  for (int i = 0; i < CTC_MAX_AC; i++) {
    if (tableNei[i][0] != -1) {
      uint32_t timeout = now - last_info[i];
      if (timeout > ctc_control.timeout) {
        tableNei[i][5] = ctc_control.timeout;
      } else {
        tableNei[i][5] = (uint16_t)timeout;
        num_neighbors++;
        float speed_nei = tableNei[i][1] / 100;
        float theta_nei = tableNei[i][2] * M_PI / 1800;
        float px_nei = tableNei[i][3] / 100;
        float py_nei = tableNei[i][4] / 100;
        u_vel += speed_nei*sinf(theta_nei - ctc_control.theta);
        p_centroid_x += px_nei;
        p_centroid_y += py_nei;
      }
    }
  }

  if(num_neighbors != 0){
      u_vel /= num_neighbors;
      p_centroid_x /= num_neighbors;
      p_centroid_y /= num_neighbors;
      
      float error_target_x = ctc_control.target_px - p_centroid_x;
      float error_target_y = ctc_control.target_py - p_centroid_y;
      ctc_error_to_target = sqrtf(error_target_x*error_target_x + error_target_y*error_target_y);
      float aux = (1-expf(-0.001*ctc_error_to_target)) / ctc_error_to_target;
      float v_ref_x = ctc_control.target_vx + aux*error_target_x;
      float v_ref_y = ctc_control.target_vy + aux*error_target_y;

      float speed_ref = sqrtf(v_ref_x*v_ref_x + v_ref_y*v_ref_y);
      float theta_ref = atan2f(v_ref_y, v_ref_x);

      u_vel -= speed_ref*sinf(theta_ref - ctc_control.theta);
      u_vel *= -(ctc_control.k*ctc_control.speed);
  }

  float u = u_vel + u_spa;

  printf("%i Total u: %f\n", AC_ID, u*180/M_PI);

  if (autopilot_get_mode() == AP_MODE_AUTO2) {
    h_ctl_roll_setpoint =
      -atanf(u * ctc_control.speed / 9.8 / cosf(att->theta));
    BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);

    lateral_mode = LATERAL_MODE_ROLL;
  }

  if (((now - last_transmision) > ctc_control.time_broad) && (autopilot_get_mode() == AP_MODE_AUTO2)) {
    ctc_send_info_to_nei();
    last_transmision = now;
  }

  return true;
}

void ctc_send_info_to_nei(void)
{
  struct pprzlink_msg msg;

  for (int i = 0; i < CTC_MAX_AC; i++)
    if (tableNei[i][0] != -1) {
      msg.trans = &(DefaultChannel).trans_tx;
      msg.dev = &(DefaultDevice).device;
      msg.sender_id = AC_ID;
      msg.receiver_id = tableNei[i][0];
      msg.component_id = 0;
      pprzlink_msg_send_CTC_INFO_TO_NEI(&msg, &ctc_control.theta, &ctc_control.speed, &ctc_control.px, &ctc_control.py);
    }
}

void parse_ctc_RegTable(void)
{
  uint8_t ac_id = DL_CTC_REG_TABLE_ac_id(dl_buffer);
  if (ac_id == AC_ID) {
    uint8_t nei_id = DL_CTC_REG_TABLE_nei_id(dl_buffer);
    for (int i = 0; i < DCF_MAX_NEIGHBORS; i++)
      if (tableNei[i][0] == -1) {
        tableNei[i][0] = (int16_t)nei_id;
        return;
      }
  }
}

void parse_ctc_CleanTable(void)
{
  uint8_t ac_id = DL_CTC_REG_TABLE_ac_id(dl_buffer);
  if (ac_id == AC_ID)
    for (int i = 0; i < DCF_MAX_NEIGHBORS; i++)
      tableNei[i][0] = -1;
}

void parse_ctc_NeiInfoTable(void)
{
  int16_t sender_id = (int16_t)(SenderIdOfPprzMsg(dl_buffer));
  for (int i = 0; i < DCF_MAX_NEIGHBORS; i++)
    if (tableNei[i][0] == sender_id) {
      last_info[i] = get_sys_time_msec();
      tableNei[i][1] = (int16_t)(DL_CTC_INFO_TO_NEI_speed(dl_buffer)*100);
      tableNei[i][2] = (int16_t)((DL_CTC_INFO_TO_NEI_theta(dl_buffer)) * 1800 / M_PI);
      tableNei[i][3] = (int16_t)(DL_CTC_INFO_TO_NEI_px(dl_buffer)*100);
      tableNei[i][4] = (int16_t)(DL_CTC_INFO_TO_NEI_py(dl_buffer)*100);
      break;
    }
}
