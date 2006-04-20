/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2005 Pascal Brisset, Antoine Drouin
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

#ifndef FBW_H
#define FBW_H

#include "std.h"
#include "adc.h"

extern uint8_t fbw_mode;
extern uint8_t fbw_vsupply_decivolt;
extern bool_t failsafe_mode;
extern struct adc_buf vsupply_adc_buf;

void init_fbw( void );
void periodic_task_fbw( void );
void event_task_fbw( void );

#endif
