/*
 * Copyright (C) 2009 Joby Energy
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

/** \file csc_rc_spektrum.h
 */

#ifndef __CSC_RC_SPEKTRUM_H__
#define __CSC_RC_SPEKTRUM_H__

#include "types.h"
#include "std.h"

struct spektrum_frame
{
  uint16_t right_horizontal;
  uint16_t flap_mix;
  uint16_t gear;
  uint16_t right_vertical;
  uint16_t aux2;
  uint16_t left_vertical;
  uint16_t left_horizontal;
} __attribute__((__packed__));

void spektrum_init(void);
void spektrum_event_task(void);
void spektrum_periodic_task(void);

#endif 
