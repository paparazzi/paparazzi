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
#include <math.h>


#include "flight_plan.h"
#include "uart.h"
#include "gps.h"
#include "ubx_protocol.h"
#include "flight_plan.h"

float gps_ftow;
float gps_falt;
float gps_fspeed;
float gps_fclimb;
float gps_fcourse;
int32_t gps_utm_east, gps_utm_north;
float gps_east, gps_north;
uint8_t gps_mode;
volatile bool_t gps_msg_received;
bool_t gps_pos_available;
const int32_t utm_east0 = NAV_UTM_EAST0;
const int32_t utm_north0 = NAV_UTM_NORTH0;

#define UBX_MAX_PAYLOAD 255
static uint8_t  ubx_msg_buf[UBX_MAX_PAYLOAD];

#define RadianOfDeg(d) ((d)/180.*3.1415927)

#ifdef SIMUL
#include "infrared.h"
#define IR_START    0xA1  /* simulator/mc.ml */
volatile int16_t simul_ir_roll;
volatile int16_t simul_ir_pitch;
#endif

#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2
#define GOT_CLASS     3
#define GOT_ID        4
#define GOT_LEN1      5
#define GOT_LEN2      6
#define GOT_PAYLOAD   7
#define GOT_CHECKSUM1 8
#ifdef SIMUL
#define GOT_IR_START  20
#define GOT_IR1 21
#define GOT_IR2 22
#define GOT_IR3 23
#endif

static uint8_t  ubx_status;
static uint16_t ubx_len;
static uint8_t  ubx_msg_idx;
static uint8_t ck_a, ck_b, ubx_id, ubx_class;

void gps_init( void ) {
  /* Enable uart                   */
#ifdef SIMUL
  uart0_init();
  simul_ir_roll = ir_roll_neutral;
  simul_ir_pitch = ir_pitch_neutral;
#else
  uart1_init();
#endif
  ubx_status = UNINIT;
}

void parse_gps_msg( void ) {
  if (ubx_class == UBX_NAV_ID) {
    if (ubx_id == UBX_NAV_POSUTM_ID) {
      gps_utm_east = UBX_NAV_POSUTM_EAST(ubx_msg_buf);
      gps_utm_north = UBX_NAV_POSUTM_NORTH(ubx_msg_buf);
      gps_falt = (float)(UBX_NAV_POSUTM_ALT(ubx_msg_buf)/100);
    } else if (ubx_id == UBX_NAV_STATUS_ID) {
      gps_mode = UBX_NAV_STATUS_GPSfix(ubx_msg_buf);
    } else if (ubx_id == UBX_NAV_VELNED_ID) {
      gps_fspeed = ((float)UBX_NAV_VELNED_GSpeed(ubx_msg_buf)) / 1e2; 
      gps_fclimb = ((float)UBX_NAV_VELNED_VEL_D(ubx_msg_buf)) / -1e2;
      gps_fcourse = RadianOfDeg(((float)UBX_NAV_VELNED_Heading(ubx_msg_buf)) / 1e5);
      gps_ftow = ((float)UBX_NAV_VELNED_ITOW(ubx_msg_buf)) / 1e3;
      
      gps_east = gps_utm_east / 100 - NAV_UTM_EAST0;
      gps_north = gps_utm_north / 100 - NAV_UTM_NORTH0;
      
      
      gps_pos_available = TRUE; /* The 3 UBX messages are sent in one rafale */
    }
  }
#ifdef SIMUL
  if (ubx_class == UBX_USR_ID) {
    if (ubx_id == UBX_USR_IRSIM_ID) {
      simul_ir_roll = UBX_USR_IRSIM_ROLL(ubx_msg_buf);
      simul_ir_pitch = UBX_USR_IRSIM_PITCH(ubx_msg_buf);
    }
  }
#endif



}


uint8_t gps_nb_ovrn;


static inline void parse_ubx( uint8_t c ) {
  if (ubx_status < GOT_PAYLOAD) {
    ck_a += c;
    ck_b += ck_a;
  }
  switch (ubx_status) {
  case UNINIT:
    if (c == UBX_SYNC1)
      ubx_status++;
    break;
  case GOT_SYNC1:
    if (c != UBX_SYNC2)
      goto error;
    ck_a = 0;
    ck_b = 0;
    ubx_status++;
    break;
  case GOT_SYNC2:
    if (gps_msg_received) {
      /* Previous message has not yet been parsed: discard this one */
      gps_nb_ovrn++;
      goto error;
    }
    ubx_class = c;
    ubx_status++;
    break;
  case GOT_CLASS:
    ubx_id = c;
    ubx_status++;
    break;    
  case GOT_ID:
    ubx_len = c;
    ubx_status++;
    break;
  case GOT_LEN1:
    ubx_len |= (c<<8);
    if (ubx_len > UBX_MAX_PAYLOAD)
      goto error;
    ubx_msg_idx = 0;
    ubx_status++;
    break;
  case GOT_LEN2:
    ubx_msg_buf[ubx_msg_idx] = c;
    ubx_msg_idx++;
    if (ubx_msg_idx >= ubx_len) {
      ubx_status++;
    }
    break;
  case GOT_PAYLOAD:
    if (c != ck_a)
      goto error;
    ubx_status++;
    break;
  case GOT_CHECKSUM1:
    if (c != ck_b)
      goto error;
    gps_msg_received = TRUE;
    goto restart;
    break;
  }
  return;
 error:  
 restart:
  ubx_status = UNINIT;
  return;
}

#ifdef SIMUL
ReceiveUart0(parse_ubx);
#else
ReceiveUart1(parse_ubx);
#endif

