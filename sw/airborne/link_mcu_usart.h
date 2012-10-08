/*
 * $Id: link_mcu.h,v 1.12 2007/08/03 10:06:45 hecto Exp $
 *
 * Copyright (C) 2003-2006 Pascal Brisset, Antoine Drouin
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

/** \brief Transport for the communication between FBW and AP in a bi-process
    architecture. Essentially adds a checksum to the inter_mcu structure.
 */

#ifndef LINK_MCU_H
#define LINK_MCU_H

#include <inttypes.h>
#include "inter_mcu.h"

struct link_mcu_msg {
  union  {
    struct fbw_state from_fbw;
    struct ap_state  from_ap;
  } payload;
};

extern struct link_mcu_msg link_mcu_from_ap_msg;
extern struct link_mcu_msg link_mcu_from_fbw_msg;

extern bool_t link_mcu_received;

extern void link_mcu_send( void );
extern void link_mcu_init( void );
extern void link_mcu_event_task( void );
extern void link_mcu_periodic_task( void );


#endif
