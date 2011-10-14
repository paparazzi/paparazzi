/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2005 Pascal Brisset, Antoine Drouin
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

#include "generated/airframe.h"

#include "std.h"
#include "3dmg.h"
#include "mcu_periph/uart.h"

volatile bool_t _3dmg_data_ready;
int16_t   _3dmg_roll, _3dmg_pitch, _3dmg_yaw;
int16_t   _3dmg_roll_dot, _3dmg_pitch_dot, _3dmg_yaw_dot;
uint16_t  _3dmg_timer_tick;
uint8_t   type, state;
uint16_t  checksum_read, checksum_comp;

void _3dmg_set_continuous_mode ( void ) {
#define REQ_CONT_LEN 5
  uint8_t msg[REQ_CONT_LEN] = { 0x10, 0x00, 0x31, 0x00, 0x41};
  uint8_t i;
  for (i=0; i<REQ_CONT_LEN; i++)
    Uart0Transmit(msg[i]);
}

void _3dmg_capture_neutral ( void ) {
#define CAP_GYR_LEN 3
  uint8_t msg[REQ_CONT_LEN] = { 0x06, 0x00, 0x06};
  uint8_t i;
  for (i=0; i<CAP_GYR_LEN; i++)
    Uart0Transmit(msg[i]);
}


#define READ_MSB(my_short, my_char) {		 \
    {						 \
     my_short = (((int16_t)my_char)<<8)&0xFF00;  \
     state++;					 \
    }						 \
  }

#define READ_LSB(my_short, my_char) {		\
    {						\
    my_short += (((int16_t)my_char)&0x00FF);	\
    checksum_comp += (uint16_t)my_short;	\
    state++;					\
    }						\
  }

static inline void on_3dmg_receive(uint8_t c) {
  int16_t foo = 0;
  switch(state) {
  case 0:
    _3dmg_data_ready = FALSE ;
    if(c==0x31) {
      type = c;
      checksum_comp = (int16_t)type;
      state++ ;
    }
    break;
  case 1:
    READ_MSB(_3dmg_roll, c);
    break ;
  case 2:
    READ_LSB(_3dmg_roll, c);
    break ;
  case 3:
    READ_MSB(_3dmg_pitch, c);
    break ;
  case 4:
    READ_LSB(_3dmg_pitch, c);
    break ;
  case 5:
    READ_MSB(_3dmg_yaw, c);
    break ;
  case 6:
    READ_LSB(_3dmg_yaw, c);
    break ;
  case 7:
    READ_MSB(foo, c);
     /* accel_x_msb */
    break ;
  case 8:
    READ_LSB(foo, c);
    /* accel_x_lsb */
    break ;
  case 9:
    READ_MSB(foo, c);
    /* accel_y_msb */
    break ;
  case 10:
    READ_LSB(foo, c);
    /* accel_y_lsb */
    break ;
  case 11:
    READ_MSB(foo, c);
    /* accel_z_msb */
    break ;
  case 12:
    READ_LSB(foo, c);
    /* accel_z_lsb */
    break ;
  case 13:
    READ_MSB(_3dmg_roll_dot, c);
    break ;
  case 14:
    READ_LSB(_3dmg_roll_dot, c);
    break ;
  case 15:
    READ_MSB(_3dmg_pitch_dot, c);
    break ;
  case 16:
    READ_LSB(_3dmg_pitch_dot, c);
    break ;
  case 17:
    READ_MSB(_3dmg_yaw_dot, c);
    break ;
  case 18:
    READ_LSB(_3dmg_yaw_dot, c);
    break ;
  case 19:
    READ_MSB(_3dmg_timer_tick, c);
    break ;
  case 20:
    READ_LSB(_3dmg_timer_tick, c);
    break ;
  case 21:
    checksum_read = (((uint16_t)c)&0x00FF)<<8;
    state++;
    break ;
  case 22:
    checksum_read += ((uint16_t)c)&0x00FF;
    state = 0 ;
    //   if (checksum_read == checksum_comp)
      _3dmg_data_ready = TRUE ;
    break ;
  }
}


#ifdef AVR_ARCH

ReceiveUart(on_3dmg_receive);

#endif /* AVR_ARCH */
