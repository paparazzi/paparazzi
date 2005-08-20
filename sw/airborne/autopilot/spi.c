/*
 * $Id$
 *  
 * Copyright (C) 2003-2005  Pascal Brisset, Antoine Drouin
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

#include <inttypes.h>
#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>


#include "spi.h"
#include "autopilot.h"
#include "link_fbw.h"

volatile uint8_t spi_cur_slave;
uint8_t spi_nb_ovrn;

void spi_init( void) {
  /* Set MOSI and SCK output, all others input */ 
  SPI_DDR |= _BV(SPI_MOSI_PIN)| _BV(SPI_SCK_PIN); 

  /* enable pull up for miso */
  //  SPI_PORT |= _BV(SPI_MISO_PIN);

  /* Set SS0 output */
  sbi( SPI_SS0_DDR, SPI_SS0_PIN);
  /* SS0 idles high (don't select slave yet)*/
  SPI_UNSELECT_SLAVE0();

  /* Set SS1 output */
  sbi( SPI_SS1_DDR, SPI_SS1_PIN);
  /* SS1 idles high (don't select slave yet)*/
  SPI_UNSELECT_SLAVE1();
  
  spi_cur_slave = SPI_NONE;
}


SIGNAL(SIG_SPI) {
  if (spi_cur_slave == SPI_SLAVE0)
    link_fbw_on_spi_it();
  else
    fatal_error_nb++;
}
