/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 */

#include "stdio.h"
#include "subsystems/radio_control.h"

static struct rc_joby_parser_state parser;
static const int16_t rc_joby_signs[RADIO_CONTROL_NB_CHANNEL] = RC_JOBY_SIGNS;


static void handle_channel(void (* callback)(void))
{
  if (parser.parser_normal_buf == RC_JOBY_MAGIC_START) {
    // got start channel, look for channel 0 next
    parser.current_channel = 0;
  } else if (parser.current_channel == -1) {
    // looking for start channel byte but didn't get it, reset
    parser.current_byte = READING_HIGH_BYTE;
    parser.current_inverted = READING_NORMAL;
  } else {
    // valid channel, store and look for next
    radio_control.values[parser.current_channel] = rc_joby_signs[parser.current_channel] * parser.parser_normal_buf;
    parser.current_channel++;
    if (parser.current_channel == RADIO_CONTROL_NB_CHANNEL) {
      // all channels read, reset parser and handle message
      parser.current_channel = -1;
      radio_control.frame_cpt++;
      radio_control.status = RC_OK;
      radio_control.time_since_last_frame = 0;
      if (callback != NULL) {
        callback();
      }
    }
  }
}

static void handle_tuple(void (* callback)(void))
{
  if (parser.current_inverted == READING_NORMAL) {
    parser.parser_normal_buf = ((parser.high_byte_buf << 8) | parser.low_byte_buf);
    parser.current_inverted = READING_INVERTED;
  } else if (parser.current_inverted == READING_INVERTED) {
    parser.parser_inverted_buf = ((parser.high_byte_buf << 8) | parser.low_byte_buf);
    parser.current_inverted = READING_NORMAL;
    if (parser.parser_normal_buf == ~parser.parser_inverted_buf) {
      handle_channel(callback);
    } else {
      // normal didn't match inverted, error, reset
      parser.current_inverted = READING_NORMAL;
      parser.current_byte = READING_HIGH_BYTE;
      parser.current_channel = -1;
      parser.error_counter++;
    }
  }
}

void rc_joby_parse(int8_t c, void (* callback)(void))
{
  if (parser.current_byte == READING_HIGH_BYTE) {
    parser.high_byte_buf = c;
    if (parser.current_channel >= 0 || parser.high_byte_buf == (RC_JOBY_MAGIC_START >> 8)
        || parser.current_inverted == READING_INVERTED) {
      // only advance parser state to low byte if we're not looking for a sync byte which we didn't find
      parser.current_byte = READING_LOW_BYTE;
    }
  } else { // READING_LOW_BYTE
    parser.low_byte_buf = c;
    parser.current_byte = READING_HIGH_BYTE;
    handle_tuple(callback);
  }
}

void radio_control_impl_init(void)
{
  parser.current_byte = READING_HIGH_BYTE;
  parser.current_inverted = READING_NORMAL;
  parser.current_channel = -1;
}
