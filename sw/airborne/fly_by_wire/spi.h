/* $Id$
 *
 * Paparazzi fbw spi functions
 *  
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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

#ifndef SPI_H
#define SPI_H

#include "link_autopilot.h"


#define SPI_PORT   PORTB
#define SPI_PIN    PINB
#define SPI_SS_PIN 2

#define SpiIsSelected() (bit_is_clear(SPI_PIN, SPI_SS_PIN))

extern struct inter_mcu_msg from_mega128;
extern struct inter_mcu_msg to_mega128;
extern volatile bool_t mega128_receive_valid;
extern volatile bool_t spi_was_interrupted;


void spi_init(void);
void spi_reset(void);


#endif /* SPI_H */
