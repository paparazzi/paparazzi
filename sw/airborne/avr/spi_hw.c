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

/** \file spi_hw.c
 *  \brief handling of hardware dependant SPI on AVR architecture
 */

#include "spi.h"

#include <inttypes.h>
#include <avr/io.h>

#if (__GNUC__ == 3)
#include <avr/signal.h>
#endif

#include <avr/interrupt.h>

#include "inter_mcu.h"

#ifdef FBW

#define IT_PORT PORTD
#define IT_DDR  DDRD
#define IT_PIN  7

#define SPI_DDR  DDRB
#define SPI_MOSI_PIN 3
#define SPI_MISO_PIN 4
#define SPI_SCK_PIN  5

volatile bool_t spi_was_interrupted = FALSE;

static volatile uint8_t idx_buf = 0;

void spi_init(void) {
  /* set it pin output */
  //  IT_DDR |= _BV(IT_PIN);

  /* set MISO pin output */
  SPI_DDR |= _BV(SPI_MISO_PIN);
  /* enable SPI, slave, MSB first, sck idle low */
  SPCR = _BV(SPE);
  /* enable interrupt */
  SPCR |= _BV(SPIE);
}

void spi_reset(void) {
  idx_buf = 0;
  SPDR = spi_buffer_input[0];

  spi_message_received = FALSE;
}


SIGNAL(SIG_SPI) {
  idx_buf++;

  spi_was_interrupted = TRUE;

  if (idx_buf < spi_buffer_length) {
    SPDR = spi_buffer_output[idx_buf];
    spi_buffer_input[idx_buf-1] = SPDR;
  } else if (idx_buf ==spi_buffer_length) {
    spi_buffer_input[idx_buf-1] = SPDR;
    spi_message_received = TRUE;
  }
  idx_buf++;
}

#endif /** FBW */


#ifdef AP

#include "autopilot.h"
#include "link_mcu.h"
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
  if (spi_cur_slave == SPI_SLAVE0) {
    /* setup OCR1A to pop in 200 clock cycles */
    /* this leaves time for the slave (fbw) */
    /* to process the byte we've sent and to  */
    /* prepare a new one to be sent           */
    OCR1A = TCNT1 + 200;
    /* clear interrupt flag  */
    sbi(TIFR, OCF1A);
    /* enable OC1A interrupt */
    sbi(TIMSK, OCIE1A);
  }
  else
    fatal_error_nb++;
}

/** send the next byte
    c.f. fly_by_wire/spi.c */
SIGNAL(SIG_OUTPUT_COMPARE1A) {
  static uint8_t tmp, crc_in1;

  /* disable OC1A interrupt */
  cbi(TIMSK, OCIE1A); 

  idx_buf++;

  /* we have sent/received a complete frame */
  if (idx_buf == FRAME_LENGTH) {
    /* read second byte of crc from receive register  */
    tmp = SPDR;
    /* notify valid frame                   */
    if (crc_in1 == Crc1(crc_in) && tmp == Crc2(crc_in)) {
      from_fbw_receive_valid = TRUE;
      link_fbw_fbw_nb_err = from_fbw.from_fbw.nb_err;
    } else
      link_fbw_nb_err++;
    /* unselect slave0                      */
    SPI_UNSELECT_SLAVE0();
    SPI_STOP();
    return;
  }

  if (idx_buf == FRAME_LENGTH - 1) {
    /* send the second byte of the crc_out */
    tmp = Crc2(crc_out);
    SPDR = tmp;
    /* get the first byte of the crc_in */
    crc_in1 = SPDR;
    return;
  } 

  /* we are sending/receiving payload       */
  if (idx_buf < FRAME_LENGTH - 2) {
    /* place new payload byte in send register */
    tmp = ((uint8_t*)&from_ap.from_ap)[idx_buf];
    SPI_SEND(tmp);
    crc_out = CrcUpdate(crc_out, tmp);
  } 
  /* we are done sending the payload */
  else { // idx_buf == FRAME_LENGTH - 2
    /* Send first byte of crc_out */
    tmp = Crc1(crc_out);
    SPI_SEND(tmp);
  }
  
  /* read the byte from receive register */
  tmp = SPDR;
  ((uint8_t*)&from_fbw.from_fbw)[idx_buf-1] = tmp;
  crc_in = CrcUpdate(crc_in, tmp);
}

#endif /* AP */
