/*
 * Copyright (C) 2013 Gautier Hattenberger (ENAC)
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
 * @file boards/lisa_l/baro_board.h
 *
 * board specific fonctions for the lisa_l board
 *
 */

#ifndef BOARDS_LISA_L_BARO_H
#define BOARDS_LISA_L_BARO_H

extern void lisa_l_baro_event(void);
#define BaroEvent lisa_l_baro_event

#endif /* BOARDS_LISA_L_BARO_H */
