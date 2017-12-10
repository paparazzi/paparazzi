/*
 * Copyright (C) 2017 Michal Podhradsky <mpodhradsky@galois.com>
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
 * @file datalink/gec/gec.c
 *
 * Galois embedded crypto iplementation
 *
 */
#include "pprzlink/spprz_utils.h" // for incoming/outgoing functions
#include "mcu_periph/rng.h"




/**
 * Trivial implementation of the processing function
 * Shall be replaced by user implementation!
 *
 * This function only copies trans->trans_rx.payload into provided buf
 */
bool spprz_process_incoming_packet(struct spprz_transport *trans, uint8_t *buf)
{
  for (uint8_t i = 0; i < trans->trans_rx.payload_len; i++) {
    buf[i] = trans->trans_rx.payload[i];
  }
  return true;
}

/**
 * Trivial implementation of the processing function
 * Shall be replaced by user implementation!
 *
 * This function is only a pass-through.
 */
bool spprz_process_outgoing_packet(struct spprz_transport *trans __attribute__((unused)))
{
  return true;
}
