/*
 * Copyright (C) 2008 Joby Energy
 *  
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

/** \file uart_throttle.h
 */

#ifndef __CSC_THROTTLE_H__
#define __CSC_THROTTLE_H__

#include "types.h"
#include "std.h"

extern uint16_t uart_throttle_faultflags;
extern uint16_t uart_throttle_rpm;

extern uint16_t uart_throttle_recv_cmd_tid;
extern uint16_t uart_throttle_recv_cmd_cid;
extern uint16_t uart_throttle_recv_cmd_arg1;
extern uint16_t uart_throttle_recv_cmd_arg2;

extern uint16_t uart_throttle_send_cmd_req;
extern uint16_t uart_throttle_send_cmd_tid;
extern uint16_t uart_throttle_send_cmd_cid;
extern uint16_t uart_throttle_send_cmd_arg1;
extern uint16_t uart_throttle_send_cmd_arg2;


void uart_throttle_init( void );
void uart_throttle_event_task( void );
void throttle_send_command(uint8_t throttle_id, uint8_t cmd_id, uint16_t arg1, uint16_t arg2);

#if 0
#ifdef USE_UART_THROTTLE
#define PERIODIC_SEND_MOTOR_CONTROL() DOWNLINK_SEND_MOTOR_CONTROL(uart_throttle_rpm, uart_throttle_rpm + 1, uart_throttle_faultflags, uart_throttle_faultflags + 1, \
  &uart_throttle_recv_cmd_tid, &uart_throttle_recv_cmd_cid, &uart_throttle_recv_cmd_arg1, &uart_throttle_recv_cmd_arg2)
#else
#define PERIODIC_SEND_MOTOR_CONTROL() 
#endif
#endif


#endif 
