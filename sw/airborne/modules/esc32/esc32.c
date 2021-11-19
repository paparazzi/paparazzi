/*
 * Copyright (C) Murat Bronz and Xavier Paris
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
 * @file "modules/esc32/esc32.c"
 * @author Murat Bronz
 * Serial Connection module between esc32v3 and ap
 * Mainly from Bill Nesbitt's code parts from AutoQuad project
 */

#include "modules/esc32/esc32.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "modules/datalink/downlink.h"
#include <stdio.h>
#include <string.h>

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

struct esc32 esc32;

#define ESC32_BUF_IN_LEN      64
#define ESC32_BUF_OUT_LEN     32
#define ESC32_PARAMS_LEN      10
#define ESC32_TELEMETRY_RATE  100.0

struct esc32_ck {
  uint8_t ck_a;
  uint8_t ck_b;
};

struct esc32_private {
  struct uart_periph *dev;
  struct esc32_ck ck_in;
  struct esc32_ck ck_out;
  uint8_t buf_in[ESC32_BUF_IN_LEN];
  uint8_t buf_out[ESC32_BUF_OUT_LEN];
  uint8_t in_idx;
  uint8_t out_idx;
  uint16_t cmd_seq_id;
  bool msg_available;
  uint8_t state;
  uint8_t initialized;
  uint8_t params_idx;
  uint8_t in_rows;
  uint8_t in_cols;
  struct esc32_parameter params[ESC32_PARAMS_LEN];
};

static struct esc32_private esc32_priv;


static void esc32_send(struct esc32_private *esc) {
  int i;
  for (i = 0; i < esc->out_idx; i++) {
    uart_put_byte(esc->dev, 0, esc->buf_out[i]);
  }
}

static void esc32_compute_ck(struct esc32_ck *ck, uint8_t c) {
  ck->ck_a += c;
  ck->ck_b += ck->ck_a;
}

static void esc32_init_ck(struct esc32_ck *ck) {
  ck->ck_a = 0;
  ck->ck_b = 0;
}

static float esc32_get_float(struct esc32_private *esc, int idx) {
  float f;
  unsigned char *c = (unsigned char *)&f;
  unsigned int i;
  for (i = 0; i < sizeof(float); i++)
    *c++ = esc->buf_in[idx + i];

  return f;
}

static void esc32_put_char(struct esc32_private *esc, unsigned char c) {
  if (esc->out_idx < ESC32_BUF_OUT_LEN) {
    esc->buf_out[esc->out_idx++] = c;
    esc32_compute_ck(&(esc->ck_out), c);
  }
}

static void esc32_put_short(struct esc32_private *esc, unsigned short i) {
  unsigned int j;
  unsigned char *c = (unsigned char *)&i;

  for (j = 0; j < sizeof(short); j++)
    esc32_put_char(esc, *c++);
}

static void esc32_put_float(struct esc32_private *esc, float f) {
  unsigned int j;
  unsigned char *c = (unsigned char *)&f;

  for (j = 0; j < sizeof(float); j++)
    esc32_put_char(esc, *c++);
}

static uint16_t esc32_send_command(struct esc32_private *esc, enum binaryCommands command, float param1, float param2, int n) {
  esc32_init_ck(&(esc->ck_out));
  esc->out_idx = 0;

  esc->buf_out[esc->out_idx++] = 'A';
  esc->buf_out[esc->out_idx++] = 'q';
  esc32_put_char(esc, 1 + 2 + n*sizeof(float));
  esc32_put_char(esc, command);
  esc32_put_short(esc, esc->cmd_seq_id++);
  if (n > 0)
    esc32_put_float(esc, param1);
  if (n > 1)
    esc32_put_float(esc, param2);

  if (esc->out_idx < ESC32_BUF_OUT_LEN - 1) {
    esc->buf_out[esc->out_idx++] = esc->ck_out.ck_a;
    esc->buf_out[esc->out_idx++] = esc->ck_out.ck_b;
    esc32_send(esc);
  }

	return (esc->cmd_seq_id - 1);
}

#define UINIT           0
#define GOT_SYNC        1
#define SEARCH_COMMAND  2
#define GET_CMD_ROWS    3
#define GET_CMD_COLS    4
#define GET_COMMAND     5
#define CHECK_CK_A      6
#define CHECK_CK_B      7

static void parse_esc32(struct esc32_private *esc, uint8_t c) {

  switch (esc->state) {
    case UINIT:
      if (c == 'A') {
        esc->state = GOT_SYNC;
      }
      break;
    case GOT_SYNC:
      if (c == 'q') {
        esc->state = SEARCH_COMMAND;
      }
      else {
        esc->state = UINIT;
      }
      break;
    case SEARCH_COMMAND:
      if (c == 'T') {
        esc32_init_ck(&(esc->ck_in));
        esc->in_idx = 0;
        esc->state = GET_CMD_ROWS;
      }
      else {
        esc->state = UINIT;
      }
      break;
    case GET_CMD_ROWS:
      esc32_compute_ck(&(esc->ck_in), c);
      esc->in_rows = c;
      esc->state = GET_CMD_COLS;
      break;
    case GET_CMD_COLS:
      esc32_compute_ck(&(esc->ck_in), c);
      esc->in_cols = c;
      if ((esc->in_rows * esc->in_cols * sizeof(float)) > ESC32_BUF_IN_LEN) {
	esc->state = UINIT;
      } else {
        esc->state = GET_COMMAND;
      }
      break;
    case GET_COMMAND:
      if (esc->in_idx >= ESC32_BUF_IN_LEN) {
	esc->state = UINIT;
      } else {
        esc32_compute_ck(&(esc->ck_in), c);
        esc->buf_in[esc->in_idx++] = c;
        if (esc->in_idx >= esc->in_rows * esc->in_cols * sizeof(float)) {
          esc->state = CHECK_CK_A;
        }
      }
      break;
    case CHECK_CK_A:
      if (esc->ck_in.ck_a == c) {
        esc->state = CHECK_CK_B;
      }
      else {
        esc->state = UINIT;
      }
      break;
    case CHECK_CK_B:
      if (esc->ck_in.ck_b == c) {
        esc->msg_available = true;
      }
      esc->state = UINIT;
      break;
    default:
      esc->state = UINIT;
      break;
  }
}

static void esc32_msg_send(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_ESC(trans, dev, AC_ID,
      &esc32.params.amps,
      &esc32.params.volts_bat,
      &esc32.power,
      &esc32.params.rpm,
      &esc32.params.volts_motor,
      &esc32.energy,
      0); // only one motor handled for now
}

void esc32_init(void) {
  memset(&esc32, 0, sizeof(struct esc32));
  memset(&esc32_priv, 0, sizeof(struct esc32_private));

  // uart device
  esc32_priv.dev = &(ESC32_DEV);

  // set command sequence id to 1
  esc32_priv.cmd_seq_id = 1;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ESC, esc32_msg_send);
#endif

};

#if ESC32_DEBUG
static int debug_start = 0;
#endif

void esc32_periodic(void) {
  if (esc32_priv.initialized == false) {
    uart_put_byte(esc32_priv.dev, 0, '\r');
    // enter binary mode
    uint8_t data[] = "binary\r";
    int i = 0;
    while (i < (int) sizeof(data)) {
      uart_put_byte(esc32_priv.dev, 0, data[i]);
      i++;
    }
    esc32_send_command(&esc32_priv, BINARY_COMMAND_TELEM_RATE, 0.0, 0.0, 1);
    esc32_send_command(&esc32_priv, BINARY_COMMAND_TELEM_VALUE, 0.0, BINARY_VALUE_AMPS, 2);
    esc32_send_command(&esc32_priv, BINARY_COMMAND_TELEM_VALUE, 1.0, BINARY_VALUE_VOLTS_BAT, 2);
    esc32_send_command(&esc32_priv, BINARY_COMMAND_TELEM_VALUE, 2.0, BINARY_VALUE_RPM, 2);
    esc32_send_command(&esc32_priv, BINARY_COMMAND_TELEM_VALUE, 3.0, BINARY_VALUE_VOLTS_MOTOR, 2);
    esc32_send_command(&esc32_priv, BINARY_COMMAND_TELEM_RATE, ESC32_TELEMETRY_RATE, 0.0, 1);

    esc32_priv.initialized = true;
  }
  else {
#if ESC32_DEBUG
    if (!debug_start) {
      // motor spining for debug
      esc32_send_command(&esc32_priv, BINARY_COMMAND_ARM, 0.0, 0.0, 0);
      esc32_send_command(&esc32_priv, BINARY_COMMAND_STOP, 0.0, 0.0, 0);
      esc32_send_command(&esc32_priv, BINARY_COMMAND_START, 0.0, 0.0, 0);
      esc32_send_command(&esc32_priv, BINARY_COMMAND_DUTY, 8.0, 0.0, 1);
      debug_start = 1;
    }
#endif
  }

}

static void esc32_filter_data(struct esc32_parameter *in, struct esc32_parameter *out, struct esc32_parameter *min, struct esc32_parameter *max) {
  // amps
  out->amps += in->amps;
  if (in->amps > max->amps) max->amps = in->amps;
  if (in->amps < min->amps) min->amps = in->amps;
  // volts_bat
  out->volts_bat += in->volts_bat;
  if (in->volts_bat > max->volts_bat) max->volts_bat = in->volts_bat;
  if (in->volts_bat < min->volts_bat) min->volts_bat = in->volts_bat;
  // volts_motor
  out->volts_motor += in->volts_motor;
  if (in->volts_motor > max->volts_motor) max->volts_motor = in->volts_motor;
  if (in->volts_motor < min->volts_motor) min->volts_motor = in->volts_motor;
  // rpm
  out->rpm += in->rpm;
  if (in->rpm > max->rpm) max->rpm = in->rpm;
  if (in->rpm < min->rpm) min->rpm = in->rpm;
}

static void esc32_parse_msg(struct esc32_private *esc_priv, struct esc32 *esc) {

  if (esc_priv->in_cols >= 4 && esc_priv->in_rows > 0) {
    // store new values
    esc_priv->params[esc_priv->params_idx].amps         = esc32_get_float(esc_priv, 0);
    esc_priv->params[esc_priv->params_idx].volts_bat    = esc32_get_float(esc_priv, 4);
    esc_priv->params[esc_priv->params_idx].rpm          = esc32_get_float(esc_priv, 8);
    esc_priv->params[esc_priv->params_idx].volts_motor  = esc32_get_float(esc_priv, 12);
    // increment index and average/filter when input array is full
    esc_priv->params_idx++;
    if (esc_priv->params_idx == ESC32_PARAMS_LEN) {
      int i;
      struct esc32_parameter tmp = esc_priv->params[0]; // cumulated values
      struct esc32_parameter min = esc_priv->params[0]; // min value
      struct esc32_parameter max = esc_priv->params[0]; // max value
      for (i = 1; i < ESC32_PARAMS_LEN; i++) {
        esc32_filter_data(&(esc_priv->params[i]), &tmp, &min, &max);
      }
      tmp.amps = (tmp.amps - min.amps - max.amps) / (ESC32_PARAMS_LEN - 2);
      tmp.volts_bat = (tmp.volts_bat - min.volts_bat - max.volts_bat) / (ESC32_PARAMS_LEN - 2);
      tmp.volts_motor = (tmp.volts_motor - min.volts_motor - max.volts_motor) / (ESC32_PARAMS_LEN - 2);
      tmp.rpm = (tmp.rpm - min.rpm - max.rpm) / (ESC32_PARAMS_LEN - 2);
      esc->params = tmp;
      // compute power
      esc->power = tmp.amps * tmp.volts_bat;
      // accumulate energy
      esc->energy += esc->power * ESC32_PARAMS_LEN / ESC32_TELEMETRY_RATE / 3600.f;
      // new data available
      esc->data_available = true;
      // reset input array
      esc_priv->params_idx = 0;
    }
  }
  esc_priv->msg_available = false;
}


void esc32_event(void) {
  while (uart_char_available(esc32_priv.dev)) {
    parse_esc32(&esc32_priv, uart_getch(esc32_priv.dev));
    if (esc32_priv.msg_available) {
      esc32_parse_msg(&esc32_priv, &esc32);
    }
  }
}

