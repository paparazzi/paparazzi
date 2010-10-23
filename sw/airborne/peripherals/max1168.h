/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef MAX1168_H
#define MAX1168_H

#include "std.h"

#define MAX1168_NB_CHAN 8

extern void max1168_init( void );

#define STA_MAX1168_IDLE           0
#define STA_MAX1168_SENDING_REQ    1
#define STA_MAX1168_READING_RES    2
#define STA_MAX1168_DATA_AVAILABLE 3
extern volatile uint8_t max1168_status;

extern uint16_t max1168_values[MAX1168_NB_CHAN];

/* underlying architecture */
#include "peripherals/max1168_arch.h"
/* must be implemented by underlying architecture */
extern void max1168_arch_init( void );
extern void max1168_read( void );


#endif /* MAX1168_H */
