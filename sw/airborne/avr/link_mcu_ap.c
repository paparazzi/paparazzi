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

#include <avr/io.h>
#include <avr/interrupt.h>

#if (__GNUC__ == 3)
#include <avr/signal.h>
#include <avr/crc16.h>
#else
#include <util/crc16.h>
#endif

#include <inttypes.h>

#include "inter_mcu.h"
#include "link_mcu_ap.h"
#include "link_mcu.h"
#include "spi.h"

static uint8_t idx_buf;
static uint16_t crc_in, crc_out;

void link_fbw_init(void) {
  link_fbw_nb_err = 0;
}

void link_fbw_send(void) {
  if (spi_cur_slave != SPI_NONE) {
    spi_nb_ovrn++;
    return;
  }

  /* Enable SPI, Master, set clock rate fck/16 */ 
  SPI_START(_BV(SPE) | _BV(MSTR) | _BV(SPR0)); // | _BV(SPR1);
  SPI_SELECT_SLAVE0();

  idx_buf = 0;
  crc_in = CRC_INIT;
  crc_out = CRC_INIT;
  uint8_t byte1 = ((uint8_t*)&from_ap.from_ap)[0];
  SPDR = byte1;
  crc_out = CrcUpdate(crc_out, byte1);
  from_fbw_receive_valid = FALSE;
  // Other bytes will follow SIG_SPI interrupts
}

void link_fbw_on_spi_it( void ) {
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
