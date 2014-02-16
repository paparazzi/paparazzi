/*
 * Copyright (C) 2013 Gautier Hattenberger, Alexandre Bustico
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

/*
 * @file firmwares/fixedwing/chibios-libopencm3/chibios_init.h
 *
 */

#ifndef CHIBIOS_INIT_H
#define CHIBIOS_INIT_H

#include <ch.h>
#include "std.h"

extern Thread *pprzThdPtr;
extern bool_t chibios_init(void);
extern void launch_pprz_thd (int32_t (*thd) (void *arg));

#endif
