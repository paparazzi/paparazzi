/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2005-2006 Pascal Brisset, Antoine Drouin
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

/** \file spi.h
 * \brief arch independant SPI (Serial Peripheral Interface) API */


#ifndef SPI_H
#define SPI_H

#include "std.h"
#include "spi_hw.h"

extern uint8_t* spi_buffer_input;
extern uint8_t* spi_buffer_output;
extern uint8_t spi_buffer_length;

extern volatile bool_t spi_message_received;

void spi_init(void);

#ifdef AP

#define SPI_NONE   0
#define SPI_SLAVE0 1
#define SPI_SLAVE1 2

extern volatile uint8_t spi_cur_slave;
extern uint8_t spi_nb_ovrn;

#define SpiCheckAvailable() (spi_cur_slave == SPI_NONE)
#define SpiOverRun() {spi_nb_ovrn++;}

#endif /* AP */

#endif /* SPI_H */


