/*
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

/** @file modules/datalink/intermcu_dl.h
 *  @brief datalink forwarder for InterMCU
 */

#ifndef INTERMCU_DL_H
#define INTERMCU_DL_H

#include "std.h"
#include "pprzlink/pprzlink_device.h"
#include "pprzlink/pprzlink_transport.h"

/* Structure for forwarding telemetry over InterMCU */
struct intermcu_dl_t {
  struct link_device *dev;      ///< Device structure for communication
  struct transport_tx *trans;   ///< Forwarding transport
};

extern struct intermcu_dl_t intermcu_dl;

/**
 *  Init function
 */
extern void intermcu_dl_init(void);

/** repack a message and send on device
 *
 * @param trans pointer to the TX transport
 * @param dev pointer to the sending device
 * @param msg pointer to the message buffer
 * @param size size of the message to send
 */
extern void intermcu_dl_repack(struct transport_tx *trans, struct link_device *dev, uint8_t *msg, uint8_t size);

/** function to forward telemetry on new message
 *
 * @param msg pointer to the message buffer
 * @param size size of the message to send
 */
extern void intermcu_dl_on_msg(uint8_t* msg, uint8_t size);

#endif /* INTERMCU_DL_H */


