/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file modules/telemetry/telemetry_intermcu_fbw.c
 *  @brief Telemetry through InterMCU
 */

#include "telemetry_intermcu.h"
#include "subsystems/intermcu.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/intermcu_msg.h"
#include "subsystems/datalink/telemetry.h"

/* Structure for handling telemetry over InterMCU */
struct telemetry_intermcu_t {
  struct link_device *dev;      ///< Device structure for communication
  struct pprz_transport trans;  ///< Transport without any extra encoding
};

/* Telemetry InterMCU throughput */
static struct telemetry_intermcu_t telemetry_intermcu;

/* Statically defined functions */
static void telemetry_intermcu_repack(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id, uint8_t msg_id, uint8_t *msg, uint8_t size);

/* InterMCU initialization */
void telemetry_intermcu_init(void)
{
  // Initialize transport structure
  pprz_transport_init(&telemetry_intermcu.trans);

  // Set the link device
  telemetry_intermcu.dev = &(TELEMETRY_INTERMCU_DEV.device);
}

/* InterMCU periodic handling of telemetry */
void telemetry_intermcu_periodic(void)
{

}

void telemetry_intermcu_on_msg(uint8_t msg_id, uint8_t* msg, uint8_t size)
{
  telemetry_intermcu_repack(&(telemetry_intermcu.trans.trans_tx), telemetry_intermcu.dev, AC_ID, msg_id, msg, size);
}

static void telemetry_intermcu_repack(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id, uint8_t msg_id, uint8_t *msg, uint8_t size)
{
  trans->count_bytes(trans->impl, dev, trans->size_of(trans->impl, size + 2 /* msg header overhead */));
  trans->start_message(trans->impl, dev, 0, size + 2 /* msg header overhead */);
  trans->put_bytes(trans->impl, dev, 0, DL_TYPE_UINT8, DL_FORMAT_SCALAR, &ac_id, 1);
  trans->put_bytes(trans->impl, dev, 0, DL_TYPE_UINT8, DL_FORMAT_SCALAR, &msg_id, 1);
  trans->put_bytes(trans->impl, dev, 0, DL_TYPE_UINT8, DL_FORMAT_ARRAY, (void *) msg, size);
  trans->end_message(trans->impl, dev, 0);
}
