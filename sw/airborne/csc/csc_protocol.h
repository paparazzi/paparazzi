/*
 * $Id:$
 *
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
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

#ifndef CSC_PROTOCOL_H
#define CSC_PROTOCOL_H

typedef void(* cscp_callback_t)(void *data);
typedef enum { INIT=0, READ=1, WRITE=2 } queue_state_t; 

void cscp_init(void);
int cscp_transmit(uint32_t client_id, uint8_t msg_id, const uint8_t *buf, uint8_t len);
void cscp_event(void);

/* Note that messages are only accepted and parsed if a callback function 
 * has been registered for their respective message id. 
 * Example: 
 *
 * 		cscp_register_callback(CSC_VANE_MSG_ID, main_on_vane_msg, (void *)&csc_vane_msg);
 * 
 */
void cscp_register_callback(uint32_t msg_id, cscp_callback_t callback, uint8_t *data);

#endif
