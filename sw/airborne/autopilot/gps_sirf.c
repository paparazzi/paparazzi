/*
 * Paparazzi mcu0 $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
#include <string.h> 


#include "uart.h"
#include "gps.h"

float gps_falt;
float gps_fspeed;
float gps_fclimb;
float gps_fcourse;
uint8_t gps_mode;
volatile bool_t gps_msg_received;
bool_t gps_pos_available;


#define SIRF_MAX_PAYLOAD 255
uint8_t  sirf_msg_buf[SIRF_MAX_PAYLOAD];

#define READ_INT32_AT_OFFSET(offset, dest)  \
{                                           \
  dest[0] = sirf_msg_buf[offset+3];    \
  dest[1] = sirf_msg_buf[offset+2];    \
  dest[2] = sirf_msg_buf[offset+1];    \
  dest[3] = sirf_msg_buf[offset];      \
}                                     \
/*  ext nav type = 0x62
           offset  len
   type    0       1
   lat     1       4
   lon     5       4
   alt     9       4
   speed   13      4
   climb   17      4
   course  21      4
   mode    25      1
*/
void parse_gps_msg( void ) {
  static int32_t tmp_int32;
  uint8_t *tmp = (uint8_t*)&tmp_int32;

  READ_INT32_AT_OFFSET(1, tmp);
  gps_lat = tmp_int32;

  READ_INT32_AT_OFFSET(5, tmp);
  gps_lon = tmp_int32;
  
  READ_INT32_AT_OFFSET(9, tmp);
  gps_falt = (float)tmp_int32 / 1e3;

  READ_INT32_AT_OFFSET(13, tmp); 
  gps_fspeed = (float)tmp_int32 / 1e3; 

  READ_INT32_AT_OFFSET(17, tmp);
  gps_fclimb = (float)tmp_int32 / 1e3;

  READ_INT32_AT_OFFSET(21, tmp);
  gps_fcourse = (float)tmp_int32 / 1e8;
  
  gps_mode = sirf_msg_buf[25];

  gps_pos_available = TRUE;
}






void gps_init( void ) {
  /* Enable uart                   */
#ifdef SIMUL
  uart0_init();
#else
  uart1_init();
#endif
}

#define SIRF_START1 0xA0
#define SIRF_START2 0xA2
#define SIRF_END1   0xB0
#define SIRF_END2   0xB3

#ifdef SIMUL
#define IR_START    0xA1  /* simulator/mc.ml */
volatile int16_t simul_ir_roll;
volatile int16_t simul_ir_pitch;
#endif

#define SIRF_TYP_NAV     0x02
#define SIRF_TYP_EXT_NAV 0x62

#define UNINIT        0
#define GOT_START1    1
#define GOT_START2    2
#define GOT_LEN1      3
#define GOT_LEN2      4
#define GOT_PAYLOAD   5
#define GOT_CHECKSUM1 6
#define GOT_CHECKSUM2 7
#define GOT_END1      8
#ifdef SIMUL
#define GOT_IR_START  9
#define GOT_IR1 10
#define GOT_IR2 11
#define GOT_IR3 12
#endif

static uint8_t  sirf_status;
static uint16_t sirf_len;
static uint16_t sirf_checksum;
static uint8_t  sirf_type;
static uint8_t  sirf_msg_idx;


static inline void parse_sirf( uint8_t c ) {
  switch (sirf_status) {
  case UNINIT:
    if (c == SIRF_START1)
      sirf_status++;
#ifdef SIMUL
    if (c == IR_START)
      sirf_status = GOT_IR_START;
#endif
    break;
  case GOT_START1:
    if (c != SIRF_START2)
      goto error;
    sirf_status++;
    break;
  case GOT_START2:
    sirf_len = (c<<8) & 0xFF00;
    sirf_status++;
    break;
  case GOT_LEN1:
    sirf_len += (c & 0x00FF);
    if (sirf_len > SIRF_MAX_PAYLOAD)
      goto error;
    sirf_msg_idx = 0;
    sirf_status++;
    break;
  case GOT_LEN2:
    if (sirf_msg_idx==0) {
      sirf_type = c;
    }
    if (sirf_type == SIRF_TYP_EXT_NAV)
      sirf_msg_buf[sirf_msg_idx] = c;
    sirf_msg_idx++;
    if (sirf_msg_idx >= sirf_len) {
      sirf_status++; 
    }
    break;
  case GOT_PAYLOAD:
    sirf_checksum = (c<<8) & 0xFF00;
    sirf_status++;
    break;
  case GOT_CHECKSUM1:
    sirf_checksum += (c & 0x00FF);
    /* fixme: check correct */
    sirf_status++;
    break;
  case GOT_CHECKSUM2:
    if (c != SIRF_END1)
      goto error;
    sirf_status++; 
    break;
  case GOT_END1:
    if (c != SIRF_END2)
      goto error;

    if (sirf_type == SIRF_TYP_EXT_NAV)
      gps_msg_received = TRUE;
    goto restart;
    break;
#ifdef SIMUL
  case GOT_IR_START:
    simul_ir_roll = c << 8;
    sirf_status++;
    break;
  case GOT_IR1:
    simul_ir_roll |= c;
    sirf_status++;
    break;
  case GOT_IR2:
    simul_ir_pitch = c << 8;
    sirf_status++;
    break;
  case GOT_IR3:
    simul_ir_pitch |= c;
    goto restart;
    break;
#endif
  }
  return;
 error:  
  //  modem_putc('r');
 restart:
  // modem_putc('\n');
  sirf_status = UNINIT;
  sirf_checksum = 0;
  return;
}

#ifdef SIMUL
ReceiveUart0(parse_sirf);
#else
ReceiveUart1(parse_sirf);
#endif
