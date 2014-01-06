/*
 * Copyright (C) 2010  Gautier Hattenberger
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

#include "cam_track.h"

#include "subsystems/ins.h"
#include "state.h"

#if USE_HFF
#include "subsystems/ins/hf_float.h"
#endif

#include "subsystems/datalink/telemetry.h"

struct FloatVect3 target_pos_ned;
struct FloatVect3 target_speed_ned;
struct FloatVect3 target_accel_ned;

struct FloatVect3 last_pos_ned;

#define CAM_DATA_LEN  (3*4)
#define CAM_START_1   0xFF
#define CAM_START_2   0xFE
#define CAM_END       0xF0

#define UNINIT        0
#define GOT_START_1   1
#define GOT_START_2   2
#define GOT_LEN       3
#define GOT_DATA      4
#define GOT_END       5

#include "messages.h"
#include "subsystems/datalink/downlink.h"

volatile uint8_t cam_msg_received;
uint8_t cam_status;
uint8_t cam_data_len;

static void send_cam_track(void) {
  DOWNLINK_SEND_NPS_SPEED_POS(DefaultChannel, DefaultDevice,
      &target_accel_ned.x, &target_accel_ned.y, &target_accel_ned.z,
      &target_speed_ned.x, &target_speed_ned.y, &target_speed_ned.z,
      &target_pos_ned.x, &target_pos_ned.y, &target_pos_ned.z);
}

void track_init(void) {
  ins_impl.ltp_initialized = TRUE; // ltp is initialized and centered on the target
  ins_update_on_agl = TRUE;   // use sonar to update agl (assume flat ground)

  cam_status = UNINIT;
  cam_data_len = CAM_DATA_LEN;

  register_periodic_telemetry(DefaultPeriodic, "CAM_TRACK", send_cam_track);
}

#include <stdio.h>
void track_periodic_task(void) {
  char cmd_msg[256];
  uint8_t c = 0;

  cmd_msg[c++] = 'A';
  cmd_msg[c++] = ' ';
  struct FloatEulers* att = stateGetNedToBodyEulers_f();
  float phi = att->phi;
  if (phi > 0) cmd_msg[c++] = ' ';
  else { cmd_msg[c++] = '-'; phi = -phi; }
  cmd_msg[c++] = '0' + ((unsigned int) phi % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (10*phi) % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (100*phi) % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (1000*phi) % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (10000*phi) % 10);
  cmd_msg[c++] = ' ';
  float theta = att->theta;
  if (theta > 0) cmd_msg[c++] = ' ';
  else { cmd_msg[c++] = '-'; theta = -theta; }
  cmd_msg[c++] = '0' + ((unsigned int) theta % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (10*theta) % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (100*theta) % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (1000*theta) % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (10000*theta) % 10);
  cmd_msg[c++] = ' ';
  float psi = att->psi;
  if (psi > 0) cmd_msg[c++] = ' ';
  else { cmd_msg[c++] = '-'; psi = -psi; }
  cmd_msg[c++] = '0' + ((unsigned int) psi % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (10*psi) % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (100*psi) % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (1000*psi) % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (10000*psi) % 10);
  cmd_msg[c++] = ' ';
  float alt = stateGetPositionEnu_f()->z;
  //alt = 0.40;
  if (alt > 0) cmd_msg[c++] = ' ';
  else { cmd_msg[c++] = '-'; alt = -alt; }
  cmd_msg[c++] = '0' + ((unsigned int) (alt/10) % 10);
  cmd_msg[c++] = '0' + ((unsigned int) alt % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (10*alt) % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (100*alt) % 10);
  cmd_msg[c++] = '0' + ((unsigned int) (1000*alt) % 10);
  cmd_msg[c++] = ' ';
  cmd_msg[c++] = '\n';;

  int i;
  for (i = 0; i < c; i++) {
    CamUartSend1(cmd_msg[i]);
  }
  //DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice,c,cmd_msg);

}

void track_event(void) {
  if (!ins_impl.ltp_initialized) {
    ins_impl.ltp_initialized = TRUE;
    ins.hf_realign = TRUE;
  }

#if USE_HFF
  if (ins.hf_realign) {
    ins.hf_realign = FALSE;
    struct FloatVect2 pos, zero;
    pos.x = -target_pos_ned.x;
    pos.y = -target_pos_ned.y;
    ins_realign_h(pos, zero);
  }
  const stuct FlotVect2 measuremet_noise = { 10.0, 10.0 };
  b2_hff_update_pos(-target_pos_ned, measurement_noise);
  ins_impl.ltp_accel.x = ACCEL_BFP_OF_REAL(b2_hff_state.xdotdot);
  ins_impl.ltp_accel.y = ACCEL_BFP_OF_REAL(b2_hff_state.ydotdot);
  ins_impl.ltp_speed.x = SPEED_BFP_OF_REAL(b2_hff_state.xdot);
  ins_impl.ltp_speed.y = SPEED_BFP_OF_REAL(b2_hff_state.ydot);
  ins_impl.ltp_pos.x   = POS_BFP_OF_REAL(b2_hff_state.x);
  ins_impl.ltp_pos.y   = POS_BFP_OF_REAL(b2_hff_state.y);

  INS_NED_TO_STATE();
#else
  // store pos in ins
  ins_impl.ltp_pos.x = -(POS_BFP_OF_REAL(target_pos_ned.x));
  ins_impl.ltp_pos.y = -(POS_BFP_OF_REAL(target_pos_ned.y));
  // compute speed from last pos
  // TODO get delta T
  // store last pos
  VECT3_COPY(last_pos_ned, target_pos_ned);

  stateSetPositionNed_i(&ins_impl.ltp_pos);
#endif

  b2_hff_lost_counter = 0;
}

#define CAM_MAX_PAYLOAD 254
uint8_t cam_data_buf[CAM_MAX_PAYLOAD];
uint8_t cam_data_idx;

void parse_cam_msg( void ) {
  uint8_t* ptr;
  // pos x
  ptr = (uint8_t*)(&(target_pos_ned.x));
  *ptr = cam_data_buf[0];
  ptr++;
  *ptr = cam_data_buf[1];
  ptr++;
  *ptr = cam_data_buf[2];
  ptr++;
  *ptr = cam_data_buf[3];
  // pos y
  ptr = (uint8_t*)(&(target_pos_ned.y));
  *ptr = cam_data_buf[4];
  ptr++;
  *ptr = cam_data_buf[5];
  ptr++;
  *ptr = cam_data_buf[6];
  ptr++;
  *ptr = cam_data_buf[7];
  // pos z
  ptr = (uint8_t*)(&(target_pos_ned.z));
  *ptr = cam_data_buf[8];
  ptr++;
  *ptr = cam_data_buf[9];
  ptr++;
  *ptr = cam_data_buf[10];
  ptr++;
  *ptr = cam_data_buf[11];

  //DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice,12,cam_data_buf);
}

void parse_cam_buffer( uint8_t c ) {
  char bla[1];
  bla[1] = c;
  //DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice,1,bla);
  switch (cam_status) {
  case UNINIT:
    if (c != CAM_START_1)
      goto error;
    cam_status++;
    break;
  case GOT_START_1:
    if (c != CAM_START_2)
      goto error;
    cam_status++;
    break;
  case GOT_START_2:
    cam_data_len = c;
    if (cam_data_len > CAM_MAX_PAYLOAD)
      goto error;
    cam_data_idx = 0;
    cam_status++;
    break;
  case GOT_LEN:
    cam_data_buf[cam_data_idx] = c;
    cam_data_idx++;
    if (cam_data_idx >= cam_data_len)
      cam_status++;
    break;
  case GOT_DATA:
    if (c != CAM_END)
      goto error;
    cam_msg_received = TRUE;
    goto restart;
    break;
  }
  return;
 error:
 restart:
  cam_status = UNINIT;
  return;
}

