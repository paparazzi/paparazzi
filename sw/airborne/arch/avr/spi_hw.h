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

#ifndef SPI_HW_H
#define SPI_HW_H

/** Index in SPI buffers: one is enough for full duplex communication */
extern volatile uint8_t spi_idx_buf;

#define SpiInitBuf() { \
  spi_idx_buf = 0; \
  SPDR = spi_buffer_output[0]; \
  spi_message_received = FALSE; \
}

#ifdef SPI_SLAVE

#define SpiStart() SpiInitBuf()

#endif /* SPI_SLAVE */





#ifdef SPI_MASTER

/* Enable SPI, Master, clock fck/16, interrupt */ 
#define SpiStart() { \
  SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0); \
  uint8_t foo; \
  if (bit_is_set(SPSR, SPIF)) \
    foo = SPDR; \
  SPCR |= _BV(SPIE); \
  SpiInitBuf(); \
}

#define SpiUnselectAllSlaves() { \
  spi_cur_slave = SPI_NONE; \
  SetBit( MASTER_SPI_SS0_PORT, MASTER_SPI_SS0_PIN );\
  /*							\
  SetBit( MASTER_SPI_SS1_PORT, MASTER_SPI_SS1_PIN );	\
  SetBit( MASTER_SPI_SS2_PORT, MASTER_SPI_SS2_PIN );\
  */						    \
}

#define SpiSelectSlave0() { \
  spi_cur_slave = SPI_SLAVE0; \
  ClearBit( MASTER_SPI_SS0_PORT, MASTER_SPI_SS0_PIN );\
}

#define SpiSelectSlave1() { \
  spi_cur_slave = SPI_SLAVE1; \
  ClearBit( MASTER_SPI_SS1_PORT, MASTER_SPI_SS1_PIN );\
}

#define SpiSelectSlave2() { \
  spi_cur_slave = SPI_SLAVE2; \
  ClearBit( MASTER_SPI_SS2_PORT, MASTER_SPI_SS2_PIN );\
}

#endif /* SPI_MASTER */


#endif /* SPI_HW_H */
