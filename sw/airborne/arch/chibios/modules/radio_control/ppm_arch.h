/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
 * @file arch/chibios/modules/radio_control/ppm_arch.h
 * PPM interface between ChibiOS and Paparazzi
 *
 * Input capture configuration has to be defined in the board.h
 */
#ifndef PPM_ARCH_H
#define PPM_ARCH_H

#include BOARD_CONFIG

/*
 * The ppm counter desired resolution is 1/6 us.
 */
#ifndef RC_PPM_TICKS_PER_USEC
#error "RC_PPM_TICKS_PER_USEC not set in board.h file"
#endif

#define RC_PPM_TICKS_OF_USEC(_v)        ((_v)*RC_PPM_TICKS_PER_USEC)
#define RC_PPM_SIGNED_TICKS_OF_USEC(_v) (int32_t)((_v)*RC_PPM_TICKS_PER_USEC)
#define USEC_OF_RC_PPM_TICKS(_v)        ((_v)/RC_PPM_TICKS_PER_USEC)

#define PPM_NB_CHANNEL RADIO_CONTROL_NB_CHANNEL

#endif /* PPM_ARCH_H */
