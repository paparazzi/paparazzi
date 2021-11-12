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

#include "modules/multi/ctc/ctc.h"
//#include "modules/datalink/datalink.h" // dl_buffer
#include "modules/datalink/telemetry.h"
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
  pprz_msg_send_CTC(trans, dev, AC_ID, 6 * CTC_MAX_AC, &(tableNei[0][0]));
}

static void send_ctc_control(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_CTC_CONTROL(trans, dev, AC_ID, &ctc_control.v_centroid_x, &ctc_control.v_centroid_y,
                            &ctc_control.target_vx, &ctc_control.target_vy, &ctc_control.target_px, &ctc_control.target_py,
                            &ctc_control.ref_px, &ctc_control.ref_py);
}
#endif // PERIODIC TELEMETRY

// Control
/*! Default gains for the algorithm */
#ifndef CTC_GAIN_K1
#define CTC_GAIN_K1 0.001
#endif
#ifndef CTC_GAIN_K2
#define CTC_GAIN_K2 0.001
#endif
#ifndef CTC_GAIN_ALPHA
#define CTC_GAIN_ALPHA 0.01
#endif
/*! Default timeout in ms for the neighbors' information */
#ifndef CTC_TIMEOUT
#define CTC_TIMEOUT 1500
#endif
/*! Default angular velocity around target in rad/sec */
#ifndef CTC_OMEGA
#define CTC_OMEGA 0.25
#endif
/*! Default time in ms for broadcasting information */
#ifndef CTC_TIME_BROAD
#define CTC_TIME_BROAD 100
#endif

ctc_con ctc_control = {CTC_GAIN_K1, CTC_GAIN_K2, CTC_GAIN_ALPHA, CTC_TIMEOUT,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, CTC_OMEGA, CTC_TIME_BROAD
                      };

int16_t tableNei[CTC_MAX_AC][6];
uint32_t last_info[CTC_MAX_AC];
uint32_t last_transmision = 0;
uint32_t before = 0;

uint32_t time_init_table;

float moving_target_px = 0;
float moving_target_py = 0;
float moving_target_vx = 0;
float moving_target_vy = 0;

bool ctc_gogo = false;
bool ctc_first_time = true;
uint32_t starting_time;

void ctc_init(void)
{
  for (int i = 0; i < CTC_MAX_AC; i++) {
    tableNei[i][0] = -1;
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CTC, send_ctc);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CTC_CONTROL, send_ctc_control);
#endif
}

bool collective_tracking_vehicle()
{
  ctc_control.target_px = moving_target_px;
  ctc_control.target_py = moving_target_py;
  ctc_control.target_vx = moving_target_vx;
  ctc_control.target_vy = moving_target_vy;

  collective_tracking_control();

  return true;
}

bool collective_tracking_waypoint(uint8_t wp)
{
  ctc_control.target_px = waypoints[wp].x;
  ctc_control.target_py = waypoints[wp].y;
  ctc_control.target_vx = 0;
  ctc_control.target_vy = 0;

  collective_tracking_control();

  return true;
}

bool collective_tracking_point(float x, float y)
{
  ctc_control.target_px = x;
  ctc_control.target_py = y;
  ctc_control.target_vx = 0;
  ctc_control.target_vy = 0;

  collective_tracking_control();

  return true;
}

void collective_tracking_control()
{
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct EnuCoor_f *v = stateGetSpeedEnu_f();
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float vx = v->x;
  float vy = v->y;
  float px = p->x;
  float py = p->y;

  ctc_control.vx = vx;
  ctc_control.vy = vy;
  ctc_control.px = px;
  ctc_control.py = py;

  ctc_control.v_centroid_x = vx;
  ctc_control.v_centroid_y = vy;
  ctc_control.p_centroid_x = px;
  ctc_control.p_centroid_y = py;


  float u_vel = 0;
  float u_spa = 0;

  if (ctc_first_time) {
    starting_time = get_sys_time_msec();
    ctc_first_time = false;
  }

  uint32_t now = get_sys_time_msec();
  float dt = (now - before) / 1000.0;
  before = now;

  int num_neighbors = 0;

  int num_neighbors = 0;

  for (int i = 0; i < CTC_MAX_AC; i++) {
    if (tableNei[i][0] != -1) {
      uint32_t timeout = now - last_info[i];
      if (timeout > ctc_control.timeout) {
        tableNei[i][5] = ctc_control.timeout;
      } else {
        tableNei[i][5] = (uint16_t)timeout;
        num_neighbors++;
        float vx_nei = tableNei[i][1] / 100.0;
        float vy_nei = tableNei[i][2] / 100.0;
        float px_nei = tableNei[i][3] / 100.0;
        float py_nei = tableNei[i][4] / 100.0;

        ctc_control.v_centroid_x += vx_nei;
        ctc_control.v_centroid_y += vy_nei;
        ctc_control.p_centroid_x += px_nei;
        ctc_control.p_centroid_y += py_nei;
      }
    }
  }
  else{

      int num_neighbors = 0;

      for (int i = 0; i < CTC_MAX_AC; i++) {
          if (tableNei[i][0] != -1) {
              uint32_t timeout = now - last_info[i];
              if (timeout > ctc_control.timeout) {
                  tableNei[i][5] = ctc_control.timeout;
              } else {
                  tableNei[i][5] = (uint16_t)timeout;
                  num_neighbors++;
                  float vx_nei = tableNei[i][1] / 100.0;
                  float vy_nei = tableNei[i][2] / 100.0;
                  float px_nei = tableNei[i][3] / 100.0;
                  float py_nei = tableNei[i][4] / 100.0;

                  ctc_control.v_centroid_x += vx_nei;
                  ctc_control.v_centroid_y += vy_nei;
                  ctc_control.p_centroid_x += px_nei;
                  ctc_control.p_centroid_y += py_nei;
              }
          }
      }
    }

    if (num_neighbors != 0) {
      ctc_control.v_centroid_x /= (num_neighbors + 1);
      ctc_control.v_centroid_y /= (num_neighbors + 1);
      ctc_control.p_centroid_x /= (num_neighbors + 1);
      ctc_control.p_centroid_y /= (num_neighbors + 1);

      float error_target_x = ctc_control.target_px - ctc_control.p_centroid_x;
      float error_target_y = ctc_control.target_py - ctc_control.p_centroid_y;
      float error_target_ref_x = ctc_control.target_px - ctc_control.ref_px;
      float error_target_ref_y = ctc_control.target_py - ctc_control.ref_py;

  if(num_neighbors != 0){
      ctc_control.v_centroid_x /= (num_neighbors + 1);
      ctc_control.v_centroid_y /= (num_neighbors + 1);
      ctc_control.p_centroid_x /= (num_neighbors + 1);
      ctc_control.p_centroid_y /= (num_neighbors + 1);

      float error_target_x = ctc_control.target_px - ctc_control.p_centroid_x;
      float error_target_y = ctc_control.target_py - ctc_control.p_centroid_y;
      float error_target_ref_x = ctc_control.target_px - ctc_control.ref_px;
      float error_target_ref_y = ctc_control.target_py - ctc_control.ref_py;

      float distance_target_ref = sqrtf(error_target_ref_x*error_target_ref_x + error_target_ref_y*error_target_ref_y);
      if(distance_target_ref < 0.1)
          distance_target_ref = 0.1;
      float aux = (1-expf(-ctc_control.alpha*distance_target_ref)) / distance_target_ref;
      float v_ref_x = ctc_control.target_vx + aux*(error_target_x);
      float v_ref_y = ctc_control.target_vy + aux*(error_target_y);

      ctc_control.ref_px += v_ref_x*dt;
      ctc_control.ref_py += v_ref_y*dt;

      float error_v_x = ctc_control.v_centroid_x - v_ref_x;
      float error_v_y = ctc_control.v_centroid_y - v_ref_y;
      u_vel = -ctc_control.k1*(-error_v_x*vy + error_v_y*vx);


      float error_ref_x = px - ctc_control.ref_px;
      float error_ref_y = py - ctc_control.ref_py;
      u_spa = ctc_control.omega*(1 + ctc_control.k2*(error_ref_x*vx + error_ref_y*vy));
  }

  float u = u_vel + u_spa;
  
  if (autopilot_get_mode() == AP_MODE_AUTO2) {
    h_ctl_roll_setpoint =
      -atanf(u * (sqrtf(vx * vx + vy * vy)) / 9.8 / cosf(att->theta));
    BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);

    lateral_mode = LATERAL_MODE_ROLL;
  }

  if (((now - last_transmision) > ctc_control.time_broad) && (autopilot_get_mode() == AP_MODE_AUTO2)) {
    ctc_send_info_to_nei();
    last_transmision = now;
  }
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
      pprzlink_msg_send_CTC_INFO_TO_NEI(&msg, &ctc_control.vx, &ctc_control.vy, &ctc_control.px, &ctc_control.py);
    }
}

void parse_ctc_RegTable(uint8_t *buf)
{
  uint8_t ac_id = DL_CTC_REG_TABLE_ac_id(buf);
  if (ac_id == AC_ID) {
    uint8_t nei_id = DL_CTC_REG_TABLE_nei_id(buf);
    for (int i = 0; i < CTC_MAX_AC; i++)
      if (tableNei[i][0] == -1) {
        tableNei[i][0] = (int16_t)nei_id;
        return;
      }
  }
}

void parse_ctc_CleanTable(uint8_t *buf)
{
  uint8_t ac_id = DL_CTC_REG_TABLE_ac_id(uint8_t *buf);
  if (ac_id == AC_ID)
    for (int i = 0; i < CTC_MAX_AC; i++) {
      tableNei[i][0] = -1;
    }

  // Reset the control variables as well
  ctc_control.p_centroid_x = 0;
  ctc_control.p_centroid_y = 0;
  ctc_control.v_centroid_x = 0;
  ctc_control.v_centroid_y = 0;
  ctc_control.target_px = 0;
  ctc_control.target_py = 0;
  ctc_control.target_vx = 0;
  ctc_control.target_vy = 0;
  ctc_control.vx = 0;
  ctc_control.vy = 0;
  ctc_control.px = 0;
  ctc_control.py = 0;
  ctc_control.ref_px = 0;
  ctc_control.ref_py = 0;

  // We force again 2 seconds of waiting before the algorithm starts, so all the aircraft have transmitted the necessary information to their neighbors
  ctc_gogo = false;
}

void parse_ctc_NeiInfoTable(uint8_t *buf)
{
  int16_t sender_id = (int16_t)(SenderIdOfPprzMsg(buf));
  for (int i = 0; i < CTC_MAX_AC; i++)
    if (tableNei[i][0] == sender_id) {
      last_info[i] = get_sys_time_msec();
      tableNei[i][1] = (int16_t)(DL_CTC_INFO_TO_NEI_vx(buf) * 100);
      tableNei[i][2] = (int16_t)(DL_CTC_INFO_TO_NEI_vy(buf) * 100);
      tableNei[i][3] = (int16_t)(DL_CTC_INFO_TO_NEI_px(buf) * 100);
      tableNei[i][4] = (int16_t)(DL_CTC_INFO_TO_NEI_py(buf) * 100);
      break;
    }
}

void parse_ctc_TargetInfo(uint8_t *buf)
{
  moving_target_px = DL_CTC_INFO_FROM_TARGET_px(buf);
  moving_target_py = DL_CTC_INFO_FROM_TARGET_py(buf);
  moving_target_vx = DL_CTC_INFO_FROM_TARGET_vx(buf);
  moving_target_vy = DL_CTC_INFO_FROM_TARGET_vy(buf);
}
