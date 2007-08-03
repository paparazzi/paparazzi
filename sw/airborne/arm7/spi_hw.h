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
#include CONFIG

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

#ifdef SPI_SLAVE
#define SpiStart() {                                                    \
 SpiEnable(); \
 SpiInitBuf();         \
 SpiEnableTxi();     /* enable tx fifo half empty interrupt */        \
}

#endif /* SPI_SLAVE */



#ifdef SPI_MASTER


/* !!!!!!!!!!!!! Code for one single slave at a time !!!!!!!!!!!!!!!!! */
#if defined SPI_SELECT_SLAVE1_PIN && defined SPI_SELECT_SLAVE0_PIN
#error "SPI: one single slave, please"
#endif


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

#define SPI_SELECT_SLAVE_IO__(port, reg) IO ## port ## reg
#define SPI_SELECT_SLAVE_IO_(port, reg) SPI_SELECT_SLAVE_IO__(port, reg)

#define SPI_SELECT_SLAVE0_IODIR SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE0_PORT, DIR)
#define SPI_SELECT_SLAVE0_IOCLR SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE0_PORT, CLR)
#define SPI_SELECT_SLAVE0_IOSET SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE0_PORT, SET)

#define SPI_SELECT_SLAVE1_IODIR SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE1_PORT, DIR)
#define SPI_SELECT_SLAVE1_IOCLR SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE1_PORT, CLR)
#define SPI_SELECT_SLAVE1_IOSET SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE1_PORT, SET)


#define SpiSelectSlave0() {	\
    spi_cur_slave = SPI_SLAVE0;	\
    SetBit(SPI_SELECT_SLAVE0_IOCLR, SPI_SELECT_SLAVE0_PIN);	\
  }

#define SpiUnselectSlave0() { \
    spi_cur_slave = SPI_NONE;	\
    SetBit(SPI_SELECT_SLAVE0_IOSET, SPI_SELECT_SLAVE0_PIN);	\
  }


#define SpiSelectSlave1() {	\
    spi_cur_slave = SPI_SLAVE1;	\
    SetBit(SPI_SELECT_SLAVE1_IOCLR, SPI_SELECT_SLAVE1_PIN);	\
  }

#define SpiUnselectSlave1() { \
    spi_cur_slave = SPI_NONE;	\
    SetBit(SPI_SELECT_SLAVE1_IOSET, SPI_SELECT_SLAVE1_PIN);	\
  }

#ifdef SPI_SELECT_SLAVE0_PIN
#define SpiUnselectCurrentSlave() SpiUnselectSlave0()
#endif

#ifdef SPI_SELECT_SLAVE1_PIN
#define SpiUnselectCurrentSlave() SpiUnselectSlave1()
#endif

#endif /* SPI_MASTER */


#define SpiSetCPHA() (SSPCR0 |= _BV(7))
#define SpiClrCPHA() (SSPCR0 &= ~(_BV(7)))


#endif /* SPI_HW_H */
