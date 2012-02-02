/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2009 Pascal Brisset, Antoine Drouin
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

/*
 *\brief architecture independent timing functions
 *
 */

#ifndef SYS_TIME_H
#define SYS_TIME_H

#include <inttypes.h>
#include BOARD_CONFIG

extern uint16_t cpu_time_sec;

#define SYS_TICS_OF_USEC(us) SYS_TICS_OF_SEC((us) * 1e-6)
#define SYS_TICS_OF_NSEC(ns) SYS_TICS_OF_SEC((ns) * 1e-9)
#define SIGNED_SYS_TICS_OF_USEC(us) SIGNED_SYS_TICS_OF_SEC((us) * 1e-6)
#define SIGNED_SYS_TICS_OF_NSEC(us) SIGNED_SYS_TICS_OF_SEC((us) * 1e-9)

#define TIME_TICKS_PER_SEC SYS_TICS_OF_SEC( 1.)
#define FIFTY_MS           SYS_TICS_OF_SEC( 50e-3 )
#define AVR_PERIOD_MS      SYS_TICS_OF_SEC( 16.666e-3 )

#include "sys_time_hw.h"


#endif /* SYS_TIME_H */
