/*
 * $Id$
 *  
 * Copyright (C) 2008 Roman Krashanitsa
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

#include "sys_time.h"
#include "print.h"
#include "aerocomm_new.h"

#ifdef SIM_UART
#include "sim_uart.h"
#endif

uint8_t aerocomm_cs;
uint8_t aerocomm_payload[AEROCOMM_PAYLOAD_LEN];
volatile bool_t aerocomm_msg_received;
volatile bool_t aerocomm_confirmation_received=TRUE;
volatile bool_t aerocomm_confirmation_status=TRUE;
volatile uint8_t aerocomm_payload_len=0;
uint8_t aerocomm_rssi;
uint8_t aerocomm_ovrn, aerocomm_error;


#define AT_COMMAND_SEQUENCE "AT+++\r"
#define AT_INIT_PERIOD_US 2000000
#define AT_SET_MY "\xCC\xC1\x80\x06xxxxxx"
#define AT_DEST_ADDR "\xCC\x10xxx"
#define AT_READ_MY "\xCC\xC0\x85\x1\r"
#define AT_AP_MODE "\xCC\x17\x09"
#define AT_RESET "\xCC\xFF"
#define AT_EXIT "\xCC\x41\x54\x4F\r"


void aerocomm_init( void ) {

aerocomm_confirmation_received=TRUE;
{  uint8_t init_cpt = 120;
  while (init_cpt) {
    if (sys_time_periodic())
      init_cpt--;
  }
}

#ifdef NO_API_INIT
#warning "NO_API_INIT defined"
#else
#warning "NO_API_INIT not defined"
#endif

#ifndef NO_API_INIT
  /** Switching to AT mode (FIXME: busy waiting) */
  AerocommPrintString(AT_COMMAND_SEQUENCE);
  char c;
  while (!AerocommLink(ChAvailable())) { 
     uint8_t init_cpt = 3;
     while (init_cpt) {
       if (sys_time_periodic())
          init_cpt--;
     }
  }
  int j;
  j=0;
  while (AerocommLink(ChAvailable()) && j<4) 
  {
	c=AerocommLink(Getch()); j++;
  }

  uint16_t addr = AEROCOMM_MY_ADDR;
  char s[]=AT_SET_MY;
  s[4]=(unsigned int)(0x00);
  s[5]=(unsigned int)(0x50);
  s[6]=(unsigned int)(0x67);
  s[7]=(unsigned int)(0x00);
  s[8]=(unsigned int)(addr>> 8);
  s[9]=(unsigned int)(addr & 0xff);
  AerocommPrintString(AT_READ_MY);
  while (!AerocommLink(ChAvailable())) { 
     uint8_t init_cpt = 3;
     while (init_cpt) {
       if (sys_time_periodic())
          init_cpt--;
     }
  }
  j=0;
  while (AerocommLink(ChAvailable()) && j<4) 
  {
	c=AerocommLink(Getch()); j++;
  }

  if (c!=s[9]) for (j=0; j<10; j++)  AerocommTransportPut1Byte(s[j]);

  while (!AerocommLink(ChAvailable())) { 
     uint8_t init_cpt = 3;
     while (init_cpt) {
       if (sys_time_periodic())
          init_cpt--;
     }
  }
  j=0;
  while (AerocommLink(ChAvailable()) && j<3) 
  {
	c=AerocommLink(Getch()); j++;
  }
  AerocommPrintString(AT_RESET);
  {  uint8_t init_cpt = 60;
     while (init_cpt) {
     if (sys_time_periodic())
        init_cpt--;
     }
   }

  AerocommPrintString(AT_COMMAND_SEQUENCE);
  while (!AerocommLink(ChAvailable())) { 
     uint8_t init_cpt = 3;
     while (init_cpt) {
       if (sys_time_periodic())
          init_cpt--;
     }
  }
  j=0;
  while (AerocommLink(ChAvailable()) && j<4) 
  {
	c=AerocommLink(Getch()); j++;
  }
  AerocommPrintString(AT_AP_MODE);
  while (!AerocommLink(ChAvailable())) { 
     uint8_t init_cpt = 3;
     while (init_cpt) {
       if (sys_time_periodic())
          init_cpt--;
     }
  }
  j=0;
  while (AerocommLink(ChAvailable()) && j<2) 
  {
	c=AerocommLink(Getch()); j++;
  }
  char s1[]=AT_DEST_ADDR;
  s1[2]=(unsigned int)(0x00);
  s1[3]=(unsigned int)(0x01);
  s1[4]=(unsigned int)(0x00);
  for (j=0; j<5; j++)  AerocommTransportPut1Byte(s1[j]);
  while (!AerocommLink(ChAvailable())) { 
     uint8_t init_cpt = 3;
     while (init_cpt) {
       if (sys_time_periodic())
          init_cpt--;
     }
  }
  j=0;
  while (AerocommLink(ChAvailable()) && j<4) 
  {
	c=AerocommLink(Getch()); j++;
  }

  AerocommPrintString(AT_EXIT);
  while (!AerocommLink(ChAvailable())) { 
     uint8_t init_cpt = 3;
     while (init_cpt) {
       if (sys_time_periodic())
          init_cpt--;
     }
  }
  j=0;
  while (AerocommLink(ChAvailable()) && j<4) 
  {
	c=AerocommLink(Getch()); j++;
  }

#endif
}
