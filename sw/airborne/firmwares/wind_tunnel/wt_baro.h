/*
 * Copyright (C) 2007  ENAC
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

/** \file
 *  \brief
 *
 */

#include "std.h"

extern uint8_t buf_input[3];
extern uint8_t buf_output[3];

extern bool wt_baro_available;
extern uint32_t wt_baro_pressure;

extern void wt_baro_init(void);
extern void wt_baro_periodic(void);
extern void wt_baro_event(void);

