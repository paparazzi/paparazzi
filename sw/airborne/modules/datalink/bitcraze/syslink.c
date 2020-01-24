/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/**
 * @file modules/datalink/bitcraze/syslink.c
 *
 * Syslink protocol for communication with bitcraze/crazyflie NRF mcu
 *
 * based on PX4 implementation
 */

#include "syslink.h"

const char *syslink_stx = "\xbc\xcf";

void syslink_parse_init(syslink_parse_state *state)
{
  state->state = SYSLINK_STATE_START;
  state->index = 0;
}

bool syslink_parse_char(syslink_parse_state *state, uint8_t c, syslink_message_t *msg)
{

  switch (state->state) {
    case SYSLINK_STATE_START:
      if (c == syslink_stx[state->index]) {
        state->index++;
      } else {
        state->index = 0;
      }

      if (syslink_stx[state->index] == '\x00') {
        state->state = SYSLINK_STATE_TYPE;
      }

      break;

    case SYSLINK_STATE_TYPE:
      msg->type = c;
      state->state = SYSLINK_STATE_LENGTH;
      break;

    case SYSLINK_STATE_LENGTH:
      msg->length = c;

      if (c > SYSLINK_MAX_DATA_LEN) { // Too long
        state->state = SYSLINK_STATE_START;
      } else {
        state->state = c > 0 ? SYSLINK_STATE_DATA : SYSLINK_STATE_CKSUM;
      }

      state->index = 0;
      break;

    case SYSLINK_STATE_DATA:
      msg->data[state->index++] = c;

      if (state->index >= msg->length) {
        state->state = SYSLINK_STATE_CKSUM;
        state->index = 0;
        syslink_compute_cksum(msg);
      }

      break;

    case SYSLINK_STATE_CKSUM:
      if (c != msg->cksum[state->index]) {
        // fail checksum
        state->state = SYSLINK_STATE_START;
        state->index = 0;
        break;
      }

      state->index++;

      if (state->index >= (int)sizeof(msg->cksum)) {
        state->state = SYSLINK_STATE_START;
        state->index = 0;
        return true; // message is correct, return true
      }

      break;
  }

  return false;

}

/*
   Computes Fletcher 8bit checksum per RFC1146
A := A + D[i]
B := B + A
*/
void syslink_compute_cksum(syslink_message_t *msg)
{
  uint8_t a = 0, b = 0;
  uint8_t *Di = (uint8_t *)msg, *end = Di + (2 + msg->length) * sizeof(uint8_t);

  while (Di < end) {
    a = a + *Di;
    b = b + a;
    ++Di;
  }

  msg->cksum[0] = a;
  msg->cksum[1] = b;
}
