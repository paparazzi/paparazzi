/*  $Id$
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

/** \brief handling of arm7 SPI hardware
 *  for now only SPI1 ( aka SSP )
 */

#ifndef SPI_HW_H
#define SPI_HW_H

#include "std.h"
#include "LPC21xx.h"

#define SPI_START() {		\
    SET_BIT(SSPCR1, SSE);	\
  }

#define SPI_STOP() {		\
    CLEAR_BIT(SSPCR1, SSE);	\
  }

#define SPI_ENABLE_RTI() {	\
    SET_BIT(SSPIMSC, RTIM);	\
  }

#define SPI_DISABLE_RTI() {	\
    CLEAR_BIT(SSPIMSC, RTIM);	\
  }

#define SPI_CLEAR_RTI() {       \
    SET_BIT(SSPICR, RTIC);	\
  }

#define SPI_ENABLE_TXI() {	\
    SET_BIT(SSPIMSC, TXIM);	\
  }

#define SPI_DISABLE_TXI() {	\
    CLEAR_BIT(SSPIMSC, TXIM);	\
  }

#define SPI_ENABLE_RXI() {	\
    SET_BIT(SSPIMSC, RXIM);	\
  }

#define SPI_DISABLE_RXI() {	\
    CLEAR_BIT(SSPIMSC, RXIM);	\
  }

#define SPI_SEND(a) {           \
    SSPDR = a;			\
  }

#define SPI_READ(a) {           \
    a = SSPDR;			\
  }

#ifdef FBW

#endif /* FBW */

#ifdef AP

/* 
 * Slave0 select : P0.20  PINSEL1 00 << 8
 * Slave1 select : P1.20  
 *
 */

#define SPI_SELECT_SLAVE0() {	\
    spi_cur_slave = SPI_SLAVE0;	\
    SET_BIT(IO0CLR, 20);	\
  }

#define SPI_UNSELECT_SLAVE0() { \
    spi_cur_slave = SPI_NONE;	\
    SET_BIT(IO0SET, 20);	\
  }

#define SPI_SELECT_SLAVE1() {	\
    spi_cur_slave = SPI_SLAVE1;	\
    SET_BIT(IO1CLR, 20);	\
  }

#define SPI_UNSELECT_SLAVE1() { \
    spi_cur_slave = SPI_NONE;	\
    SET_BIT(IO1SET, 20);	\
  }

#endif /* AP */

#endif /* SPI_HW_H */
