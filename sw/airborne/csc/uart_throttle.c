/*
 * Copyright (C) 2009 Joby Energy
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

/** \file uart_throttle.c
 */

#include "uart_throttle.h"

#include <inttypes.h>

#include "led.h"

//#include "downlink.h"
//#include "messages.h"
#include "uart.h"
//#include "print.h"
//#include "com_stats.h"

#define THROTTLE_START1 0xAC
#define THROTTLE_START2 0xBE

#define THROTTLE_ID 0	

uint32_t throttle_err_count;
uint32_t throttle_recv_count;

uint16_t uart_throttle_faultflags;
uint16_t uart_throttle_rpm;
uint16_t uart_throttle_vbus;

uint16_t uart_throttle_recv_cmd_tid;
uint16_t uart_throttle_recv_cmd_cid;
uint16_t uart_throttle_recv_cmd_arg1;
uint16_t uart_throttle_recv_cmd_arg2;

uint16_t uart_throttle_send_cmd_req;
uint16_t uart_throttle_send_cmd_tid;
uint16_t uart_throttle_send_cmd_cid;
uint16_t uart_throttle_send_cmd_arg1;
uint16_t uart_throttle_send_cmd_arg2;

#define __ThrottleLink(dev, _x) dev##_x
#define _ThrottleLink(dev, _x)  __ThrottleLink(dev, _x)
#define ThrottleLink(_x) _ThrottleLink(THROTTLE_LINK, _x)

#define ThrottlePrintString(s) ThrottleLink(PrintString(s))
#define ThrottleBuffer() ThrottleLink(ChAvailable())
#define ReadThrottleBuffer() { while (ThrottleLink(ChAvailable())&&!throttle_msg_received) parse_throttle_msg(ThrottleLink(Getch())); }

#define ThrottleUartSend1(c) ThrottleLink(Transmit(c))
#define ThrottleUartInitParam(_a,_b,_c) ThrottleLink(InitParam(_a,_b,_c))
#define ThrottleUartRunning ThrottleLink(TxRunning)

#define UNINIT         0
#define GOT_START1     1
#define GOT_START2     2
#define GOT_TID        3
#define GOT_CMD        4
#define GOT_ARG1H      5
#define GOT_ARG1L      6
#define GOT_ARG2H      7
#define GOT_ARG2L      8
#define GOT_CHECKSUML  9

#define THROTTLE_CMD_INFO 1
#define THROTTLE_CMD_STOP 2
#define THROTTLE_CMD_START 3
#define THROTTLE_CMD_CONFIGURE 4
#define THROTTLE_CMD_FAULT_CLEAR 5
#define THROTTLE_CMD_SET_SPEED 6
#define THROTTLE_CMD_GET_RREG 7
#define THROTTLE_CMD_SET_RREG 8
#define THROTTLE_CMD_REG_RVALUE 9
#define THROTTLE_CMD_GET_MREG 10
#define THROTTLE_CMD_SET_MREG 11
#define THROTTLE_CMD_REG_MVALUE 12

#define THROTTLE_MAX_PAYLOAD 254
static volatile uint8_t throttle_msg_received;

struct throttle_msg {
  uint8_t start_byte1;
  uint8_t start_byte2;
  uint8_t throttle_id;
  uint8_t cmd_id;
  uint16_t arg1;
  uint16_t arg2;
  uint16_t checksum;
};

static struct throttle_msg recv_msg;

void parse_throttle_msg(uint8_t c );
void throttle_parse_msg( void );
void throttle_send_command(uint8_t throttle_id, uint8_t cmd_id, uint16_t arg1, uint16_t arg2);
void throttle_send_message(struct throttle_msg *send);
uint16_t throttle_calculate_checksum(struct throttle_msg *send);

void uart_throttle_init( void )
{

  throttle_err_count = 0;
  throttle_recv_count = 0;
}

void uart_throttle_event_task( void ) 
{
  if (ThrottleBuffer()) {
    ReadThrottleBuffer();
  }
  if (throttle_msg_received) {
    throttle_parse_msg();
    throttle_msg_received = FALSE;
  }
}

// Called after receipt of valid message
void throttle_parse_msg( void )
{
  if (throttle_calculate_checksum(&recv_msg) != recv_msg.checksum) {
    throttle_err_count++;
    return;
  }
  throttle_recv_count++;

  switch (recv_msg.cmd_id) {
     case THROTTLE_CMD_INFO:
      uart_throttle_rpm = recv_msg.arg1;
      uart_throttle_faultflags = recv_msg.arg2;
      break;
     default:
      uart_throttle_recv_cmd_tid = recv_msg.throttle_id;
      uart_throttle_recv_cmd_cid = recv_msg.cmd_id;
      uart_throttle_recv_cmd_arg1 = recv_msg.arg1;
      uart_throttle_recv_cmd_arg2 = recv_msg.arg2;
      break;
  }
}

uint16_t throttle_calculate_checksum(struct throttle_msg *send)
{
  return send->throttle_id + send->cmd_id + send->arg1 + send->arg2;
}

void throttle_send_command(uint8_t throttle_id, uint8_t cmd_id, uint16_t arg1, uint16_t arg2)
{
  struct throttle_msg msg;

  msg.start_byte1 = THROTTLE_START1;
  msg.start_byte2 = THROTTLE_START2;
  msg.throttle_id = throttle_id;
  msg.cmd_id = cmd_id;
  msg.arg1 = arg1;
  msg.arg2 = arg2;

  msg.checksum = throttle_calculate_checksum(&msg);

  throttle_send_message(&msg);
}

void throttle_send_message(struct throttle_msg *send)
{

  ThrottleUartSend1(send->start_byte1);
  ThrottleUartSend1(send->start_byte2);
  ThrottleUartSend1(send->throttle_id);
  ThrottleUartSend1(send->cmd_id);
  ThrottleUartSend1(send->arg1 >> 8);
  ThrottleUartSend1(send->arg1 & 0xFF);
  ThrottleUartSend1(send->arg2 >> 8);
  ThrottleUartSend1(send->arg2 & 0xFF);
  ThrottleUartSend1(send->checksum >> 8);
  ThrottleUartSend1(send->checksum & 0xFF);
}

// Simple state machine parser for throttle messages
// Passed serial bytes one per call and parses stream into message id, data length, and data buffer
// for use at higher level
void parse_throttle_msg( uint8_t c ) {
  static uint8_t throttle_status;

  switch (throttle_status) {
  case UNINIT:
    // Look for throttle start byte 1
    if (c != THROTTLE_START1)
      goto error;
    recv_msg.start_byte1 = c;
    throttle_status++; 
    break;
  case GOT_START1:
    // Look for throttle start byte 2
    if (c != THROTTLE_START2)
      goto error;
    recv_msg.start_byte2 = c;
    throttle_status++;
    break;
  case GOT_START2:
    // Look for throttle ID
    recv_msg.throttle_id = c;
    throttle_status++;
    break;
  case GOT_TID:
    // Save command ID
    recv_msg.cmd_id = c;
    throttle_status++;
    break;
  case GOT_CMD:
    // Save arg1 high
    recv_msg.arg1 = (c << 8);
    throttle_status++;
    break;
  case GOT_ARG1H:
    // Save arg1 low
    recv_msg.arg1 |= c;
    throttle_status++;
    break;
  case GOT_ARG1L:
    // Save arg2 high
    recv_msg.arg2 = (c << 8);
    throttle_status++;
    break;
  case GOT_ARG2H:
    // Save arg2 low
    recv_msg.arg2 |= c;
    throttle_status++;
    break;
  case GOT_ARG2L:
    // Read checksum byte high
    recv_msg.checksum = (c << 8);
    throttle_status++;
    break;
  case GOT_CHECKSUML:
    recv_msg.checksum |= c;
    // Notification for valid message received
    throttle_msg_received = TRUE;
    goto restart;
    break;
  }
  return;
 error:  
 restart:
  // Start over (Reset parser state)
  throttle_status = UNINIT;
  return;
}
