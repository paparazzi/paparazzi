/*
 * Copyright (C) 2010-2014 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file arch/sim/modules/radio_control/ppm_arch.h
 *
 * PPM radio control, simulator specific.
 *
 */

#ifndef PPM_ARCH_H
#define PPM_ARCH_H


/**
 * On tiny (and booz) the ppm counter is running at the same speed as
 * the systic counter. There is no reason for this to be true.
 * Let's add a pair of macros to make it possible for them to be different.
 *
 */
#define RC_PPM_TICKS_OF_USEC(_x) (_x)
#define RC_PPM_SIGNED_TICKS_OF_USEC(_x) (_x)
#define USEC_OF_RC_PPM_TICKS(_x) (_x)

#if USE_NPS
extern void radio_control_feed(void);
#endif

#endif /* PPM_ARCH_H */
