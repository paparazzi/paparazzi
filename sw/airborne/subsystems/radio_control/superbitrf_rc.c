/*
 * Copyright (C) 2013 Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file subsystems/radio_control/superbitrf_rc.c
 * DSM2 and DSMX radio control implementation for the cyrf6936 2.4GHz radio chip trough SPI
 */

#include "superbitrf_rc.h"
#include "subsystems/radio_control.h"

/**
 * Initialization
 */
//#if DATALINK == SUPERBITRF
//void radio_control_impl_init(void) {}
//#else
void radio_control_impl_init(void) { superbitrf_init(); }
//#endif
