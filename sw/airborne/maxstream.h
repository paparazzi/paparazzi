/*
 * Paparazzi mcu0 $Id$
 *  
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

/* Maxstream serial input and output */

#ifndef MAXSTREAM_H
#define MAXSTREAM_H

#include "uart_ap.h"

/** Set when a message (stored in internal buffer) has been parsed with
correct checksum. A new message cannot be received while this flag is true. */
extern bool_t maxstream_msg_received;

/** Incremented when chars are received while ::maxstream_msg_received is set*/
extern uint8_t maxstream_ovrn;

/** Incremented if error (e.g. checksum) is detected in frame */
extern uint8_t maxstream_error;

/** To be called when ::maxstream_msg_received is set. Fills ::dl_buffer and
    sets ::dl_msg_available if message approved. */
void maxstream_parse_payload(void);

#endif /* MAXSTREAM_H */
