/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
 * Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 *
 */

/** @file modules/datalink/intermcu_dl.c
 *  @brief datalink forwarder for InterMCU
 */

#include "modules/datalink/intermcu_dl.h"
#include "modules/datalink/datalink.h"
#include "modules/intermcu/intermcu.h"
#include "pprzlink/intermcu_msg.h"
#include "modules/datalink/telemetry.h"

#ifndef INTERMCU_DL_UPDATE_DL
#define INTERMCU_DL_UPDATE_DL TRUE
#endif

struct intermcu_dl_t intermcu_dl;

/* initialization */
void intermcu_dl_init(void)
{
#ifdef TELEMETRY_INTERMCU_DEV
  intermcu_dl.dev = &(TELEMETRY_INTERMCU_DEV).device;
#ifdef DOWNLINK_TRANSPORT
  intermcu_dl.trans = &(DOWNLINK_TRANSPORT).trans_tx;
#else // DEV defined but not transport
#error "TELEMETRY_INTERMCU_DEV is defined but not DOWNLINK_TRANSPORT"
#endif
#else // no DEV
  intermcu_dl.dev = NULL;
  intermcu_dl.trans = NULL;
#endif
}

/** Repack message with same header and send on selected link if possible
 */
void intermcu_dl_repack(struct transport_tx *trans, struct link_device *dev, uint8_t *msg, uint8_t size)
{
  struct pprzlink_msg pmsg;
  pmsg.trans = trans;
  pmsg.dev = dev;
  pmsg.sender_id = pprzlink_get_msg_sender_id(msg);
  pmsg.receiver_id = pprzlink_get_msg_receiver_id(msg);
  pmsg.component_id = pprzlink_get_msg_component_id(msg);

  if (trans->check_available_space(&pmsg, _FD_ADDR, size)) {
    trans->count_bytes(&pmsg, size);
    trans->start_message(&pmsg, _FD, size);
    trans->put_bytes(&pmsg, _FD, DL_TYPE_UINT8, DL_FORMAT_ARRAY, (void *) msg, size);
    trans->end_message(&pmsg, _FD);
  } else {
    trans->overrun(&pmsg);
  }
}

/**
 * function to forward telemetry from AP to the ground
 */
void intermcu_dl_on_msg(uint8_t* msg, uint8_t size)
{
  if (intermcu_dl.dev != NULL) {
    intermcu_dl_repack(intermcu_dl.trans, intermcu_dl.dev, msg, size);
  }
}


