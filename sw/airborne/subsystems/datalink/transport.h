/*
 * Copyright (C) 2011 Gautier Hattenberger
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

/** \file transport.h
 *  generic transport header
 */

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <inttypes.h>
#include "std.h"

#ifndef TRANSPORT_PAYLOAD_LEN
#define TRANSPORT_PAYLOAD_LEN 256
#endif

/** Generic transport header
 */
struct transport {
  // payload buffer
  uint8_t payload[TRANSPORT_PAYLOAD_LEN];
  // payload length
  volatile uint8_t payload_len;
  // message received flag
  volatile bool_t msg_received;
  // overrun and error flags
  uint8_t ovrn, error;
};

/** Transport link macros
 *
 * make the link between the transport layer
 * and the device layer
 */
#define __TransportLink(dev, _x) dev##_x
#define _TransportLink(dev, _x)  __TransportLink(dev, _x)
#define TransportLink(_dev, _x) _TransportLink(_dev, _x)

#endif /* TRANSPORT_H */

