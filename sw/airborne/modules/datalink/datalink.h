/*
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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
/** \file datalink.h
 *  \brief Handling of messages coming from ground and other A/Cs
 *
 */

#ifndef DATALINK_H
#define DATALINK_H

#ifdef DATALINK_C
#define EXTERN
#else
#define EXTERN extern
#endif

#include "std.h"
#include "pprzlink/dl_protocol.h"

/** Datalink kinds */
#define PPRZ 1
#define XBEE 2
#define SUPERBITRF 3
#define W5100 4
#define BLUEGIGA 5

/** Flag provided to control calls to ::dl_parse_msg. NOT used in this module*/
EXTERN bool dl_msg_available;

/** time in seconds since last datalink message was received */
EXTERN uint16_t datalink_time;

/** number of datalink/uplink messages received */
EXTERN uint16_t datalink_nb_msgs;

#define MSG_SIZE 256
EXTERN uint8_t dl_buffer[MSG_SIZE]  __attribute__((aligned));

/** Should be called when chars are available in dl_buffer */
EXTERN void dl_parse_msg(struct link_device *dev, struct transport_tx *trans, uint8_t *buf);

/** Firmware specfic msg handler */
EXTERN void firmware_parse_msg(struct link_device *dev, struct transport_tx *trans, uint8_t *buf);

#if USE_NPS
EXTERN bool datalink_enabled;
#endif

/** Convenience macro to fill dl_buffer */
// TODO: replace with a memcpy for efficiency
#define DatalinkFillDlBuffer(_buf, _len) { \
  uint16_t _i = 0; \
  for (_i = 0; _i < _len; _i++) { \
    dl_buffer[_i] = _buf[_i]; \
  } \
  dl_msg_available = true; \
}

/** periodic function, should be called at 1Hz */
extern void datalink_periodic(void);

/** Check for new message and parse */
static inline void DlCheckAndParse(struct link_device *dev, struct transport_tx *trans, uint8_t *buf, bool *msg_available, bool update_dl)
{
  // make it possible to disable datalink in NPS sim
#if USE_NPS
  if (!datalink_enabled) {
    return;
  }
#endif

  if (*msg_available) {
    if (update_dl) {
      datalink_time = 0;
      datalink_nb_msgs++;
    }
    dl_parse_msg(dev, trans, buf);
    *msg_available = false;
  }
}

#endif /* DATALINK_H */
