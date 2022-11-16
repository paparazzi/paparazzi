/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
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

/**
 * @file modules/datalink/datalink.c
 * Handling of messages coming from ground and other A/Cs.
 *
 */

#define MODULES_DATALINK_C

#include "datalink.h"
#include "modules/datalink/downlink.h"

#include "generated/modules.h"
#include "generated/settings.h"

#include "pprzlink/messages.h"

bool dl_msg_available;
uint16_t datalink_time;
uint16_t datalink_nb_msgs;
uint8_t dl_buffer[MSG_SIZE]  __attribute__((aligned));

#if USE_NPS
bool datalink_enabled = true;
#endif

void datalink_init(void)
{
  dl_msg_available = false;
  datalink_time = 0;
  datalink_nb_msgs = 0;
}

void datalink_periodic(void)
{
  datalink_time++; // called at 1Hz
}

void datalink_parse_PING(struct link_device *dev, struct transport_tx *trans, uint8_t *buf)
{
  // Reply to the sender of the message
  struct pprzlink_msg msg;
  msg.trans = trans;
  msg.dev = dev;
  msg.sender_id = AC_ID;
  msg.receiver_id = pprzlink_get_msg_sender_id(buf);
  msg.component_id = 0;
  pprzlink_msg_send_PONG(&msg);
}

void WEAK dl_parse_msg(struct link_device *dev, struct transport_tx *trans, uint8_t *buf)
{
  uint8_t msg_id = pprzlink_get_msg_id(buf);
  uint8_t class_id = pprzlink_get_msg_class_id(buf);
  /* Parse modules datalink */
  modules_parse_datalink(msg_id, class_id, dev, trans, buf);
}

