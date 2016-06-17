/*
 * Copyright (C) 2010 Martin Mueller
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

/**
 * @file modules/core/trigger_ext.h
 * Measure external trigger pulse at PPM input (default).
 *
 * This measures a trigger pulse length
 */

#ifndef TRIGGER_EXT_H
#define TRIGGER_EXT_H

#include "std.h"

/**
 *  falling/rising edge
 */
#define TRIG_EXT_EDGE_RISING 1
#define TRIG_EXT_EDGE_FALLING 0

extern uint32_t trigger_t0;
extern uint32_t trigger_delta_t0;
extern volatile bool trigger_ext_valid;

void trigger_ext_init(void);

#endif


