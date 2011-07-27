/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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
 */

#ifndef PPM_ARCH_H
#define PPM_ARCH_H


/**
 * On tiny (and booz) the ppm counter is running at the same speed as
 * the systic counter. There is no reason for this to be true.
 * Let's add a pair of macros to make it possible for them to be different.
 *
 */
#define RC_PPM_TICS_OF_USEC(_x) (_x)
#define RC_PPM_SIGNED_TICS_OF_USEC(_x) (_x)

#define PPM_NB_CHANNEL RADIO_CONTROL_NB_CHANNEL

#ifdef USE_NPS
extern void radio_control_feed(void);
#endif

#endif /* PPM_ARCH_H */
