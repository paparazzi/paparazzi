/*
 * $Id$
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

#include "dl_protocol.h"

EXTERN bool_t dl_msg_available;
/** Flag provided to control calls to ::dl_parse_msg. NOT used in this module*/

#define MSG_SIZE 128
EXTERN char dl_buffer[MSG_SIZE];

void dl_parse_msg(void);
/** Should be called when chars are available in dl_buffer */

#endif
