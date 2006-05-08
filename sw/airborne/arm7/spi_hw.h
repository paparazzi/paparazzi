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

extern volatile uint8_t spi_tx_idx;
extern volatile uint8_t spi_rx_idx;

#define SpiInitBuf() {                                                  \
  spi_rx_idx = 0;                                                       \
  spi_tx_idx = 0;                                                       \
  spi_message_received = FALSE;                                         \
  SpiTransmit();      /* fill fifo */                                   \
}

#define SpiTransmit() {						        \
    while (spi_tx_idx < spi_buffer_length	                 	\
	   && bit_is_set(SSPSR, TNF)) {					\
      SpiSend(spi_buffer_output[spi_tx_idx]);	                        \
      spi_tx_idx++;						        \
    }			                                                \
    if (spi_tx_idx == spi_buffer_length)	        		\
      SpiDisableTxi();                                                  \
}

#define SpiReceive() {		         				\
    while (bit_is_set(SSPSR, RNE)) {					\
      if (spi_rx_idx < spi_buffer_length) {                             \
          SpiRead(spi_buffer_input[spi_rx_idx])                         \
          spi_rx_idx++;						        \
      }                                                                 \
      else {                                                            \
         uint8_t foo;                                                   \
	 SpiRead(foo);                                                  \
      }                                                                 \
    }									\
  }

#define SpiEnable() {		\
    SetBit(SSPCR1, SSE);	\
  }

#define SpiDisable() {		\
    ClearBit(SSPCR1, SSE);	\
  }

#define SpiEnableRti() {	\
    SetBit(SSPIMSC, RTIM);	\
  }

#define SpiDisableRti() {	\
    ClearBit(SSPIMSC, RTIM);	\
  }

#define SpiClearRti() {         \
    SetBit(SSPICR, RTIC);	\
  }

#define SpiEnableTxi() {	\
    SetBit(SSPIMSC, TXIM);	\
  }

#define SpiDisableTxi() {	\
    ClearBit(SSPIMSC, TXIM);	\
  }

#define SpiEnableRxi() {	\
    SetBit(SSPIMSC, RXIM);	\
  }

#define SpiDisableRxi() {	\
    ClearBit(SSPIMSC, RXIM);	\
  }

#define SpiSend(a) {            \
    SSPDR = a;			\
  }

#define SpiRead(a) {            \
   a = SSPDR;			\
  }

#ifdef FBW
#define SpiStart() {                                                    \
 SpiEnable(); \
 SpiInitBuf();         \
 SpiEnableTxi();     /* enable tx fifo half empty interrupt */        \
}

#endif /* FBW */

#ifdef AP

#define SpiStart() {                                                    \
   SpiEnable();                                                         \
   SpiInitBuf();                                                        \
   SpiEnableTxi();     /* enable tx fifo half empty interrupt */        \
}

/* 
 * Slave0 select : P0.20  PINSEL1 00 << 8
 * Slave1 select : P1.20  
 *
 */

#define SpiSelectSlave0() {	\
    spi_cur_slave = SPI_SLAVE0;	\
    SetBit(IO0CLR, 20);	\
  }

#define SpiUnselectSlave0() { \
    spi_cur_slave = SPI_NONE;	\
    SetBit(IO0SET, 20);	\
  }

#define SpiSelectSlave1() {	\
    spi_cur_slave = SPI_SLAVE1;	\
    SetBit(IO1CLR, 20);	\
  }

#define SpiUnselectSlave1() { \
    spi_cur_slave = SPI_NONE;	\
    SetBit(IO1SET, 20);	\
  }

#endif /* AP */

#endif /* SPI_HW_H */
