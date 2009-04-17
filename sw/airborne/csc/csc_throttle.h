/*
 * Copyright (C) 2008 Joby Energy
 *  
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

/** \file uart_throttle.h
 */

#ifndef __CSC_THROTTLE_H__
#define __CSC_THROTTLE_H__

#include "types.h"
#include "std.h"


void csc_throttle_init( void );
void csc_throttle_event_task( void );
void csc_throttle_send_msg(uint8_t throttle_id, uint8_t cmd_id, uint16_t arg1, uint16_t arg2);


#endif 
