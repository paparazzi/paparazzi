/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
/**
 * @file "modules/sensors/cameras/jevois.c"
 * @author Gautier Hattenberger
 * Decoder for standardized messages from the JEVOIS smart camera (http://jevois.org)
 */

#include "modules/sensors/cameras/jevois.h"

#include "std.h"
#include "mcu_periph/uart.h"
#include "subsystems/abi.h"
#include "math/pprz_algebra_float.h"
#include "subsystems/datalink/downlink.h"
#include <string.h>
#include <stdio.h>

int jevois_mapping_setting;
bool jevois_stream_setting;

// max string length
#define JEVOIS_MAX_LEN 32
// max number of coordinates
#define JEVOIS_MAX_COORD 18
// check delimiter
#define JEVOIS_CHECK_DELIM(_c) (_c == ' ' || _c == '\n' || _c == '\r' || _c == '\0')

// generic JEVOIS message structure
struct jevois_msg_t {
  uint8_t type;
  char id[JEVOIS_MAX_LEN];
  uint8_t nb;
  int16_t coord[JEVOIS_MAX_COORD];
  uint16_t dim[3];
  struct FloatQuat quat;
  char extra[JEVOIS_MAX_LEN];
};

// decoder state
enum jevois_state {
  JV_SYNC = 0,
  JV_TYPE,
  JV_ID,
  JV_SIZE,
  JV_COORD,
  JV_DIM,
  JV_QUAT,
  JV_EXTRA,
  JV_SEND_MSG
};

// jevois struct
struct jevois_t {
  enum jevois_state state; // decoder state
  char buf[JEVOIS_MAX_LEN]; // temp buffer
  uint8_t idx; // temp buffer index
  uint8_t n; // temp coordinates/dimension index
  struct jevois_msg_t msg; // last decoded message
  bool data_available; // new data to report
};

struct jevois_t jevois;

// reporting function, send telemetry message
void jevois_report(void)
{
  if (jevois.data_available == false) {
    // no new data, return
    return;
  }

  float quat[4] = {
    jevois.msg.quat.qi,
    jevois.msg.quat.qx,
    jevois.msg.quat.qy,
    jevois.msg.quat.qz
  };
  uint8_t len = strlen(jevois.msg.id);
  char none[] = "None";
  char *id = jevois.msg.id;
  if (len == 0) {
    id = none;
    len = 4;
  }
  DOWNLINK_SEND_JEVOIS(DefaultChannel, DefaultDevice,
      &jevois.msg.type,
      len, id,
      &jevois.msg.nb,
      Max(jevois.msg.nb,1), jevois.msg.coord,
      jevois.msg.dim,
      quat);
  jevois.data_available = false;
}

// initialization
void jevois_init(void)
{
  // dummy settings
  jevois_mapping_setting = 0;
  jevois_stream_setting = false;

  jevois.state = JV_SYNC;
  jevois.idx = 0;
  jevois.n = 0;
  jevois.data_available = false;
  memset(jevois.buf, 0, JEVOIS_MAX_LEN);
}

// extrat a number (int) from an idea string
// this might be needed as jevois ID can start with a letter
// this will extract the first substring with a number
// and return the result of atoi function
static int jevois_extract_nb(char *in) {
  unsigned int i, j = 0;
  bool first = false;
  char out[JEVOIS_MAX_LEN];
  for (i = 0; i < strlen(in)+1; i++) {
    if ((in[i] > '0' && in[i] < '9') || in[i] == '-') {
      out[j++] = in[i];
      first = true;
    } else if (first || in[i] == '\0') {
      out[j] = '\0';
      break;
    }
  }
  return atoi(out);
}

// send specific message if requested
static void jevois_send_message(void)
{
#if JEVOIS_SEND_MSG
  // send pprzlink JEVOIS message
  jevois_report();
#endif
#if JEVOIS_SEND_FOLLOW_TARGET
  float cam_heading = (JEVOIS_HFOV / (2.f * JEVOIS_NORM)) * (float)(jevois.msg.coord[0]);
  float cam_height = (JEVOIS_VFOV / (2.f * JEVOIS_NORM)) * (float)(jevois.msg.coord[1]);
  // send a FOLLOW_TARGET message in camera frame
  AbiSendMsgFOLLOW_TARGET(CAM_JEVOIS_ID, 0, 0, cam_heading, cam_height, 0.f);
#endif
#if JEVOIS_SEND_VISUAL_DETECTION
  AbiSendMsgVISUAL_DETECTION(CAM_JEVOIS_ID,
      jevois.msg.coord[0],
      jevois.msg.coord[1],
      jevois.msg.dim[0],
      jevois.msg.dim[1],
      0,
      (int16_t)jevois_extract_nb(jevois.msg.id));
#endif
}

// raw message parsing function
static void jevois_parse(struct jevois_t *jv, char c)
{
  switch (jv->state) {
    case JV_SYNC:
      // wait for sync (newline character)
      if (c == '\n') {
        jv->state = JV_TYPE;
        jv->idx = 0;
        jv->n = 0;
      }
      break;
    case JV_TYPE:
      jv->buf[jv->idx++] = c; // fill buffer
      // parse type
      if (jv->idx > 2) { // msg type + white space
        if (jv->buf[0] == 'T' && jv->buf[1] == '1') {
          jv->state = JV_COORD;
          jv->msg.type = JEVOIS_MSG_T1;
          jv->msg.nb = 1;
        } else if (jv->buf[0] == 'N' && jv->buf[1] == '1') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_N1;
          jv->msg.nb = 1;
        } else if (jv->buf[0] == 'D' && jv->buf[1] == '1') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_D1;
          jv->msg.nb = 2;
        } else if (jv->buf[0] == 'T' && jv->buf[1] == '2') {
          jv->state = JV_COORD;
          jv->msg.type = JEVOIS_MSG_T2;
          jv->msg.nb = 2;
        } else if (jv->buf[0] == 'N' && jv->buf[1] == '2') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_N2;
          jv->msg.nb = 2;
        } else if (jv->buf[0] == 'D' && jv->buf[1] == '2') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_D2;
          jv->msg.nb = 8;
        } else if (jv->buf[0] == 'F' && jv->buf[1] == '2') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_F2;
          jv->msg.nb = 0;
        } else if (jv->buf[0] == 'T' && jv->buf[1] == '3') {
          jv->state = JV_COORD;
          jv->msg.type = JEVOIS_MSG_T3;
          jv->msg.nb = 3;
        } else if (jv->buf[0] == 'N' && jv->buf[1] == '3') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_N3;
          jv->msg.nb = 3;
        } else if (jv->buf[0] == 'D' && jv->buf[1] == '3') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_D3;
          jv->msg.nb = 3;
        } else if (jv->buf[0] == 'F' && jv->buf[1] == '3') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_F3;
          jv->msg.nb = 0;
        } else {
          jv->state = JV_SYNC; // error
        }
        jv->idx = 0;
      }
      break;
    case JV_ID:
      if (JEVOIS_CHECK_DELIM(c)) {
        jv->msg.id[jv->idx] = '\0'; // end string
        if (jv->msg.type == JEVOIS_MSG_F2 ||
            jv->msg.type == JEVOIS_MSG_F3) {
          jv->state = JV_SIZE; // parse n before coordinates
        } else {
          jv->state = JV_COORD; // parse directly coordinates
        }
        jv->idx = 0;
        break;
      }
      else {
        jv->msg.id[jv->idx++] = c;
        if (jv->idx > JEVOIS_MAX_LEN - 1) {
          jv->state = JV_SYNC; // too long, return to sync
        }
      }
      break;
    case JV_SIZE:
      if (JEVOIS_CHECK_DELIM(c)) {
        jv->buf[jv->idx] = '\0'; // end string
        jv->msg.nb = (uint8_t)atoi(jv->buf); // store size
        jv->state = JV_COORD;
        jv->idx = 0;
      }
      else {
        jv->buf[jv->idx++] = c; // fill buffer
      }
      break;
    case JV_COORD:
      if (JEVOIS_CHECK_DELIM(c)) {
        jv->buf[jv->idx] = '\0'; // end string
        jv->msg.coord[jv->n++] = (int16_t)atoi(jv->buf); // store value
        if (jv->n == jv->msg.nb) {
          // got all coordinates, go to next state
          jv->n = 0; // reset number of received elements
          jv->idx = 0; // reset index
          switch (jv->msg.type) {
            case JEVOIS_MSG_T1:
            case JEVOIS_MSG_T2:
            case JEVOIS_MSG_T3:
              jv->state = JV_SEND_MSG;
              break;
            case JEVOIS_MSG_N1:
            case JEVOIS_MSG_N2:
            case JEVOIS_MSG_N3:
            case JEVOIS_MSG_D3:
              jv->state = JV_DIM;
              break;
            case JEVOIS_MSG_D1:
            case JEVOIS_MSG_D2:
            case JEVOIS_MSG_F2:
            case JEVOIS_MSG_F3:
              jv->state = JV_EXTRA;
              break;
            default:
              jv->state = JV_SYNC; // error
              break;
          }
        }
        jv->idx = 0; // reset index
      }
      else {
        jv->buf[jv->idx++] = c; // fill buffer
      }
      break;
    case JV_DIM:
      if (JEVOIS_CHECK_DELIM(c)) {
        jv->buf[jv->idx] = '\0'; // end string
        jv->msg.dim[jv->n++] = (uint16_t)atoi(jv->buf); // store dimension
        if (jv->n == jv->msg.nb) {
          // got all dimensions, go to next state
          jv->n = 0; // reset number of received elements
          jv->idx = 0; // reset index
          if (jv->msg.type == JEVOIS_MSG_D3) {
            jv->state = JV_QUAT;
          } else {
            jv->state = JV_SEND_MSG;
          }
          break;
        }
        jv->idx = 0; // reset index
      }
      else {
        jv->buf[jv->idx++] = c; // fill buffer
      }
      break;
    case JV_QUAT:
      if (JEVOIS_CHECK_DELIM(c)) {
        jv->buf[jv->idx] = '\0';
        float q = (float)atof(jv->buf);
        switch (jv->n) {
          case 0:
            jv->msg.quat.qi = q;
            break;
          case 1:
            jv->msg.quat.qx = q;
            break;
          case 2:
            jv->msg.quat.qy = q;
            break;
          case 3:
            jv->msg.quat.qz = q;
            break;
          case 4:
            jv->state = JV_EXTRA;
            break;
          default:
            jv->state = JV_SYNC; // error
            break;
        }
        jv->n++;
        jv->idx = 0; // reset index
      }
      else {
        jv->buf[jv->idx++] = c; // fill buffer
      }
      break;
    case JV_EXTRA:
      if (JEVOIS_CHECK_DELIM(c)) {
        jv->msg.extra[jv->idx] = '\0'; // end string
        jv->state = JV_SEND_MSG;
        jv->idx = 0; // reset index
      }
      else {
        jv->msg.extra[jv->idx++] = c; // store extra string
        if (jv->idx > JEVOIS_MAX_LEN - 1) {
          jv->state = JV_SYNC; // too long, return to sync
        }
      }
      break;
    case JV_SEND_MSG:
      // send ABI message
      AbiSendMsgJEVOIS_MSG(CAM_JEVOIS_ID,
          jv->msg.type,
          jv->msg.id,
          jv->msg.nb,
          jv->msg.coord,
          jv->msg.dim,
          jv->msg.quat,
          jv->msg.extra);
      // also send specific messages if needed
      jevois_send_message();
      jv->data_available = true;
      jv->state = JV_SYNC;
      break;
    default:
      // error, back to SYNC
      jv->state = JV_SYNC;
      break;
  }
}


// UART polling function
void jevois_event(void)
{
  // Look for data on serial link and send to parser
  while (uart_char_available(&(JEVOIS_DEV))) {
    uint8_t ch = uart_getch(&(JEVOIS_DEV));
    jevois_parse(&jevois, ch);
  }
}

// utility function to send a string
void jevois_send_string(char *s)
{
  uint8_t i = 0;
  while (s[i]) {
    uart_put_byte(&(JEVOIS_DEV), 0, (uint8_t)(s[i]));
    i++;
  }
}

void jevois_stream(bool activate)
{
  jevois_stream_setting = activate;
  if (activate) {
    jevois_send_string("streamon\r\n");
  } else {
    jevois_send_string("streamoff\r\n");
  }
}

void jevois_setmapping(int number)
{
  jevois_mapping_setting = number;
  jevois_stream(false);
  jevois_send_string("setmapping ");
  char s[4];
#ifndef SITL
  itoa(number, s, 10);
#endif
  jevois_send_string(s);
  jevois_send_string("\r\n");
  jevois_stream(true);
}

void jevois_send_state(void)
{
  char str[32] __attribute__((unused));
#if JEVOIS_SEND_ALT
  // send current altitude in millimeter
  int alt_mm = (int)(stateGetPositionEnu_f()->z * 1000.f);
  Bound(alt_mm, -999999, 999999);
  sprintf(str, "alt %d\r\n", alt_mm);
  jevois_send_string(str);
#endif
#if JEVOIS_SEND_POS
  // send current position in millimeter
  struct EnuCoor_f pos = *stateGetPositionEnu_f();
  int x_mm = (int)(pos.x * 1000.f);
  int y_mm = (int)(pos.y * 1000.f);
  int z_mm = (int)(pos.z * 1000.f);
  Bound(x_mm, -999999, 999999);
  Bound(y_mm, -999999, 999999);
  Bound(z_mm, -999999, 999999);
  sprintf(str, "pos %d %d %d\r\n", x_mm, y_mm, z_mm);
  jevois_send_string(str);
#endif
#if JEVOIS_SEND_QUAT
  // send quaternion
  struct FloatQuat quat = *stateGetNedToBodyQuat_f();
  int qi = (int)(quat.qi * 1000.f);
  int qx = (int)(quat.qx * 1000.f);
  int qy = (int)(quat.qy * 1000.f);
  int qz = (int)(quat.qz * 1000.f);
  Bound(qi, -9999, 9999);
  Bound(qx, -9999, 9999);
  Bound(qy, -9999, 9999);
  Bound(qz, -9999, 9999);
  sprintf(str, "quat %d %d %d %d\r\n", qi, qx, qy, qz);
  jevois_send_string(str);
#endif
}
