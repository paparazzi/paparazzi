/*
 * $Id$
 *  
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2002  Chris efstathiou hendrix@otenet.gr
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
/** \file srf08.h
 *  \brief Basic library for SRF08 telemeter
 *
 */

#include <avr/io.h>
#include "i2c.h"
#include "uart.h"
#include "srf08.h"


uint16_t srf08_range;

/* Global Variables */
static unsigned char address=SRF08_UNIT_0;

/*###########################################################################*/

void srf08_init(void)
{
  unsigned int  range=0;
  i2c_init();
  I2C_START_TX(address);
  i2c_transmit(0);                    
  i2c_transmit(0x51);
       
  do{
    i2c_start();
    range=i2c_sla(address);               
    i2c_stop();
  } while(range != I2C_NO_ERROR);                               
                                

  return;
}
/*###########################################################################*/


void srf08_initiate_ranging(void) {
  i2c_buf[0]=0;
  i2c_buf[1]=SRF08_CENTIMETERS;
  i2c_send(address, 2);
}

void srf08_receive(void) {
  i2c_buf[0]=SRF08_ECHO_1;
  i2c_send(address, 1);
  while (!i2c_idle);
  i2c_get(address, 2);
  while (!i2c_idle);
  srf08_range = i2c_buf[0] << 8 | i2c_buf[1];
}

void srf08_ping()
{
  uint8_t byte;

  srf08_initiate_ranging();
  while (!i2c_idle);
       
  do {
    i2c_start();
    byte=i2c_sla(address);               
    i2c_stop();
  } while(byte != I2C_NO_ERROR);
       

  srf08_receive();
}
/*###########################################################################*/

unsigned int srf08_read_register(unsigned char srf08_register)
{
  union i2c_union {
    unsigned int  rx_word; 
    unsigned char rx_byte[2];
  } i2c;


  I2C_START_TX(address);
  i2c_transmit(srf08_register);
  I2C_START_RX(address);
       
  /* get high byte msb first */ 
  if(srf08_register>=2) {
    i2c.rx_byte[1]=i2c_receive(I2C_CONTINUE);
  }                         
       
  /* get low byte msb first  */ 
  i2c.rx_byte[0]=i2c_receive(I2C_QUIT);                          

  i2c_stop();

  return(i2c.rx_word);
}

