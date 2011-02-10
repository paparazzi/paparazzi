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

#ifndef MS2100_H
#define MS2100_H


#include "std.h"
#define MS2100_NB_AXIS 3

extern void ms2100_init( void );
extern void ms2100_read( void );
extern void ms2100_reset( void);

#define MS2100_IDLE            0
#define MS2100_BUSY            1
#define MS2100_SENDING_REQ     2
#define MS2100_WAITING_EOC     3
#define MS2100_GOT_EOC         4
#define MS2100_READING_RES     5
#define MS2100_DATA_AVAILABLE  6

extern volatile uint8_t ms2100_status;
extern volatile int16_t ms2100_values[MS2100_NB_AXIS];

/* underlying architecture */
#include "peripherals/ms2100_arch.h"
/* must be implemented by underlying architecture */
extern void ms2100_arch_init( void );

#define MS2100_DIVISOR_128  2
#define MS2100_DIVISOR_256  3
#define MS2100_DIVISOR_512  4
#define MS2100_DIVISOR_1024 5

#define MS2100_DIVISOR MS2100_DIVISOR_1024


#endif /* MS2100_H */
