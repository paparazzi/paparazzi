/*
 * Copyright (C) 2009 Pascal Brisset <pascal.brisset@gmail.com>,
 *                    Antoine Drouin <poinix@gmail.com>
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

#ifndef RADIO_CONTROL_JOBY_H
#define RADIO_CONTROL_JOBY_H

#include "std.h"
#include "mcu_periph/uart.h"

#define RC_JOBY_MAGIC_START   13999

#include RADIO_CONTROL_JOBY_MODEL_H

typedef enum {
  READING_LOW_BYTE = 0,
  READING_HIGH_BYTE
} parser_byte_t;

typedef enum {
  READING_NORMAL = 0,
  READING_INVERTED
} parser_inverted_t;

struct rc_joby_parser_state {
  parser_byte_t current_byte;
  parser_inverted_t current_inverted;
  int current_channel;

  int16_t parser_inverted_buf;
  int16_t parser_normal_buf;
  uint8_t high_byte_buf;
  uint8_t low_byte_buf;

  uint32_t error_counter;
};

void rc_joby_parse(int8_t c, void (* callback)(void));

#define __RcLink(dev, _x) dev##_x
#define _RcLink(dev, _x)  __RcLink(dev, _x)
#define RcLink(_x) _RcLink(RADIO_CONTROL_LINK, _x)

#define RcLinkChAvailable() RcLink(ChAvailable())
#define RcLinkGetCh() RcLink(Getch())

#define RadioControlEvent(_received_frame_handler) {                    \
    while (RcLinkChAvailable()) {                                       \
      rc_joby_parse(RcLinkGetCh(), _received_frame_handler);    \
    }                                                                   \
  }


#endif /* RADIO_CONTROL_JOBY_H */
