/*
 * $Id$
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

#ifndef SITL
#include "link_mcu_hw.h"
#endif

#define USE_SPI 1

struct link_mcu_msg {
  union  { 
    struct fbw_state from_fbw; 
    struct ap_state  from_ap;
  } payload;
  uint16_t checksum;
};

extern struct link_mcu_msg link_mcu_from_ap_msg;
extern struct link_mcu_msg link_mcu_from_fbw_msg;

extern bool_t link_mcu_received;

extern void link_mcu_event_task( void );

#ifdef FBW
#define SPI_SLAVE 1
extern void link_mcu_restart(void);
#endif /* FBW */

#ifdef AP
#define SPI_MASTER 1
extern uint8_t link_mcu_nb_err;
extern uint8_t link_mcu_fbw_nb_err;

extern void link_mcu_init(void);
extern void link_mcu_send(void);
#endif /* AP */

#endif
