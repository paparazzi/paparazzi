/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2004 Pascal Brisset, Antoine Drouin
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
#include <avr/interrupt.h>

#include "spi.h"
#include "timer.h"

void uart_send(uint8_t c);

volatile uint8_t spi_cur_slave = SPI_NONE;

void my_spi_init(void) {
 /* Set MOSI and SCK output, all others input */ 
  SPI_DDR = _BV(SPI_MOSI_PIN)| _BV(SPI_SCK_PIN); 

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
 
}


void spi_start( void ) {
  uint8_t foo;
 /* Enable SPI, Master, MSB first, clock idle low, sample on leading edge, clock rate fck/128 */
  SPCR = (_BV(SPE)| _BV(MSTR) | _BV(SPR1)  |  _BV(SPR0));
  if (bit_is_set(SPSR, SPIF))
    foo = SPDR;
  SPI_SELECT_SLAVE1();
}

uint8_t spi_transmit(uint8_t c) {
  uint8_t foo;
  SPDR = c;
  while (bit_is_clear(SPSR, SPIF));
  foo = inp(SPDR);

  uart_send(c);
  uart_send(foo);
  return foo;
}

void spi_stop(void) {
  SPI_UNSELECT_SLAVE1();
  SPI_STOP();
}


inline void delay_long(uint8_t n) {
  uint8_t ctx;
  while (n > 0) {
    ctx=0;
    do {ctx++;} while (ctx);
    n--;
  }
}

#define SYNC_FLAG  0xAA
#define TERMINATOR 0x00

#define GET_MODE_INFO       0x01
#define MOD_INFO_RESP       0x02
#define SET_DATA_COMPONENTS 0x03
#define GET_DATA            0x04
#define DATA_RESP           0x05
#define SET_CONFIG          0x06
#define GET_CONFIG          0x07
#define CONFIG_RESP         0x08
#define SAVE_CONFIG         0x09
#define START_CAL           0x0A
#define STOP_CAL            0x0B
#define GET_CAL_DATA        0x0C
#define CAL_DATA_RESP       0x0D
#define SET_CAL_DATA        0x0E

#define DATA_XRAW        0x01 // Slnt32 counts  32768 to 32767 
#define DATA_YRAW        0x02 // Slnt32 counts  32768 to 32767 
#define DATA_XCAL        0x03 // Float32 scaled to 1.0 
#define DATA_YCAL        0x04 // Float32 scaled to 1.0 
#define DATA_HEADING     0x05 // Float32 degrees 0.0 ° to 359.9 ° 
#define DATA_MAGNITUDE   0x06 // Float32 scaled to 1.0 
#define DATA_TEMPERATURE 0x07 // Float32 ° Celsius 
#define DATA_DISTORTION  0x08 // Boolean 
#define DATA_CAL_STATUS  0x09 // Boolean

uint8_t err_cnt;
uint8_t errno;

#define DATA_FIELD_NB 3
#define DATA_LEN 12
void v2xe_setup_data_components(void) {
  uint8_t c;
  spi_start();
  c = spi_transmit(SYNC_FLAG);
  c = spi_transmit(SET_DATA_COMPONENTS);
  c = spi_transmit(DATA_FIELD_NB);
  c = spi_transmit(DATA_XRAW);
  c = spi_transmit(DATA_YRAW);
  c = spi_transmit(DATA_TEMPERATURE);
  c = spi_transmit(TERMINATOR);
  spi_stop(); 
  uart_send(0xFF);
}

void v2xe_read_data(void) {
  uint8_t c, i=0;
  spi_start();

  /* querry */
  c = spi_transmit(SYNC_FLAG);
  c = spi_transmit(GET_DATA);
  c = spi_transmit(TERMINATOR);

  delay_long(255);
  
  /* answer */
  do { c = spi_transmit(0x00); i++;}
  while (c!=SYNC_FLAG && i < 20);
  //  if (i>20) TIMEOUT;
  /* frame type */
  c = spi_transmit(0x00);
  /* nb fields  */
  c = spi_transmit(0x00);
  /* fields + data       */
  for (i=0; i < DATA_LEN + DATA_FIELD_NB + 1; i++) {
    c = spi_transmit(0x00);
  }

  spi_stop();
  uart_send(0xFF);
}

#define ID_LEN 8
uint8_t id_str[ID_LEN];
void v2xe_read_id (void) {
  uint8_t c, i=0;

  spi_start();
  c = spi_transmit(SYNC_FLAG);
  c = spi_transmit(GET_MODE_INFO);
  c = spi_transmit(TERMINATOR);

  //  delay_long(10);
  
  //  c = spi_transmit(0x00);

 
  do { c = spi_transmit(0x00); i++;}
  while (c!=SYNC_FLAG && i < 20);

 /*  if (c != SYNC_FLAG) { */
/*     err_cnt++; */
/*     errno = 1; */
/*     spi_stop(); */
/*     return; */
/*   } */
//  c = spi_transmit(0x00);
/*   if (c != MOD_INFO_RESP) { */
/*     err_cnt++; */
/*     errno = 2; */
/*     spi_stop(); */
/*     return; */
/*   } */
  for (c = 0; c < ID_LEN; c++) 
    id_str[c] = spi_transmit(0x00);
  c = spi_transmit(0x00);
/*   if (c != TERMINATOR) { */
/*     err_cnt++; */
/*     errno = 3; */
/*     spi_stop(); */
/*     return; */
/*   }  */

  spi_stop();

  uart_send(0xFF);
}


#define UBRRH UBRR0H
#define UBRRL UBRR0L
#define UCSRA UCSR0A
#define UCSRB UCSR0B
#define UCSRC UCSR0C
#define UDR UDR0

void uart_init(void) {
  /* Baudrate is 38.4k */
  UBRRH = 0; 
  UBRRL = 25; 
  /* single speed */ 
  UCSRA = 0; 
  /* Enable receiver and transmitter */ 
  UCSRB = _BV(RXEN) | _BV(TXEN);
  /* Set frame format: 8data, 1stop bit */ 
  UCSRC = _BV(UCSZ1) | _BV(UCSZ0); 

}

void uart_send(uint8_t c) {
  /* Wait for empty transmit buffer */
  while ( !( UCSRA & _BV(UDRE)) ) ;
  /* Put data into buffer, sends the data */
  UDR = c;
}


inline void periodic_task(void) {
  static uint8_t foo; 
     if (foo == 0)
       v2xe_read_id();
     if (foo == 10)
       v2xe_setup_data_components();
     if (foo > 10 && !(foo%10))
       v2xe_read_data();
	//	uart_send(err_cnt);
	//	uart_send (errno);
	//	{
	//	  uint8_t i;
	//	  for (i=0; i<8; i++)
	//	    uart_send(id_str[i]);
	//	}
	//	uart_send('\n');
     foo++;
     if (foo > 60) foo=0;
}

int main( void ) {
    
  timer_init();
  my_spi_init();
  uart_init();
  //  sei();

  delay_long(255);
  delay_long(255);
  
  while (1) {
    if (timer_periodic())
      periodic_task();
  }
  return 0;
}
