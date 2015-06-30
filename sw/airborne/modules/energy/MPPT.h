/*
 * Copyright (C) 2009  ENAC, Pascal Brisset, Michel Gorraz
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
 * @file modules/energy/MPPT.h
 * @brief Solar cells MPTT monitoring
 *
 */

#ifndef MPPT_H
#define MPPT_H

#include <inttypes.h>

#define MPPT_MODE_OFF     1
#define MPPT_MODE_PASSIVE 2
#define MPPT_MODE_ACTIVE  3

#define NB_DATA 9
#define MPPT_IBAT_INDEX 1
#define MPPT_ICONV_INDEX 6
#define MPPT_ITOTAL_INDEX 8


extern uint8_t MPPT_mode;

void MPPT_init(void);
void MPPT_automata(void);
void MPPT_periodic(void);

#endif // MPPT_H
