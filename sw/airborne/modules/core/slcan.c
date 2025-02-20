/*
 * Copyright (C) 2025 Fabien-B <fabien-b@github.com>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/core/slcan.c"
 * @author Fabien-B <fabien-b@github.com>
 * SLCAN interface. CAN over serial link. See https://www.canusb.com/files/can232_v3.pdf
Useful for using DroneCAN GUI over serial USB.
 */

#include "modules/core/slcan.h"
#include "generated/airframe.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/can.h"
#include "mcu_periph/usb_serial.h"
#include "pprzlink/pprzlink_device.h"
#include "stdio.h"
#include "string.h"



slcan_t slcan;

static void slcan_handle_cmd(slcan_t* slcan);
static void slcan_rsp_ok(slcan_t* slcan);
static void slcan_rsp_error(slcan_t* slcan);

static int slcan_handle_frame(slcan_t* slcan);

static uint8_t hex2nibble(char a);
static uint8_t nibble2hex(uint8_t x);


void slcan_can_rx_cb(struct pprzcan_frame* rxframe, struct pprzaddr_can* src_addr, void* user_data);


void slcan_init(void) {
  slcan.dev = &SLCAN_PORT.device;
  slcan.cmd_buf_idx = 0;
  slcan.can_if.can_ifindex = SLCAN_CAN_IF;
  slcan.timestamp = true;
  pprz_mtx_init(&slcan.mtx); // needed if support for a seconf interface is added
  can_register_callback(slcan_can_rx_cb, &slcan.can_if, &slcan);

}

void slcan_event(void) {
  while(slcan.dev->char_available(slcan.dev->periph) > 0) {
    uint8_t b = slcan.dev->get_byte(slcan.dev->periph);
    slcan.cmd_buf[slcan.cmd_buf_idx] = b;
    slcan.cmd_buf_idx = (slcan.cmd_buf_idx+1) % SLCAN_CMD_BUF_LEN;

    if(b == '\r' && slcan.cmd_buf_idx > 1) {
      slcan_handle_cmd(&slcan);
    }

  }
}

static void slcan_handle_cmd(slcan_t* slcan) {
  uint8_t* args = &slcan->cmd_buf[1];  // where arguments begins
  

  switch (slcan->cmd_buf[0])
  {
  // CAN frames
  case 't':
  case 'T':
  case 'd':
  case 'D':
  case 'r':
  case 'R':
    if(slcan_handle_frame(slcan)) {
      slcan_rsp_error(slcan);
    } else {
      slcan_rsp_ok(slcan);
    }
    break;

  // a bunch of unimplemented commands. Just returns OK.
  case 'S':
  case 's':
  case 'O':
  case 'L':
  case 'C':
  case 'X':
  case 'W':
  case 'M':
  case 'm':
  case 'U':
  case 'Q':
    slcan_rsp_ok(slcan);
    break;

  // unimplemented commands, return ERROR.
  case 'A':
  case 'P':
    slcan_rsp_error(slcan);
    break;

  // Set timestamp ON/OFF
  case 'Z':
    slcan->timestamp = args[0] == '1';
    break;

  // get status
  case 'F':
  {
    char rsp_buf[5];
    const uint8_t status = 0;   // TODO get status
    int len = snprintf(rsp_buf, 5, "F%02X\r", status); // OK included
    slcan->dev->put_buffer(slcan->dev->periph, 0, (uint8_t*)rsp_buf, (uint16_t)len);
    break;
  }

  // get version
  case 'V':
  {
    char rsp_buf[7];
    int len = snprintf(rsp_buf, 7, "V%x%x%x%x\r", 1, 0, 1, 0); // OK included
    slcan->dev->put_buffer(slcan->dev->periph, 0, (uint8_t*)rsp_buf, (uint16_t)len);
    break;
  }

  // get serial number
  case 'N':
  {
    char rsp_buf[7] = "NPPRZ\r"; // OK included
    slcan->dev->put_buffer(slcan->dev->periph, 0, (uint8_t*)rsp_buf, 6);
    break;
  }

  default:
    slcan_rsp_error(slcan);
    break;
  }

  slcan->cmd_buf_idx = 0;

}


static int slcan_handle_frame(slcan_t* slcan) {
  const uint8_t n = slcan->cmd_buf_idx-2;   // number of chars after the command char
  const uint8_t cmd = slcan->cmd_buf[0];
  uint8_t* args = &slcan->cmd_buf[1];  // where arguments begins
  uint8_t* data;

  //slcan_frame_t tx_frame;
  struct pprzcan_frame tx_frame;

  if(cmd == 't' || cmd == 'd' || cmd == 'r') {
    // SID
    if(n < 4) {return -1;}  // error: not enough cmd chars
    tx_frame.can_id = (hex2nibble(args[0]) << 8) |
                      (hex2nibble(args[1]) << 4) |
                      (hex2nibble(args[2]) << 0);
    tx_frame.len = can_dlc_to_len(hex2nibble(args[3]));
    data = &args[4];
  } else {
    // EID
    if(n < 9) {return -1;}  // error: not enough cmd chars
    tx_frame.can_id = (hex2nibble(args[0]) << 28) |
                      (hex2nibble(args[1]) << 24) |
                      (hex2nibble(args[2]) << 20) |
                      (hex2nibble(args[3]) << 16) |
                      (hex2nibble(args[4]) << 12) |
                      (hex2nibble(args[5]) << 8)  |
                      (hex2nibble(args[6]) << 4)  |
                      (hex2nibble(args[7]) << 0);
    tx_frame.can_id |= CAN_FRAME_EFF;
    tx_frame.len = can_dlc_to_len(hex2nibble(args[8]));
    data = &args[9];
  }

  if(cmd == 'r' || cmd == 'R') {
    tx_frame.can_id |= CAN_FRAME_RTR;
  } else {
    if(n < tx_frame.len + (tx_frame.can_id & CAN_FRAME_EFF ? 9: 4)) {return -1;}  // error: not enough cmd chars

    for(int i=0; i < tx_frame.len; i++) {
      tx_frame.data[i] = (hex2nibble(data[2*i]) << 4) | (hex2nibble(data[2*i+1]) << 0);
    }
  }

  can_transmit_frame(&tx_frame, &slcan->can_if);
  return 0;

}



static uint8_t slcan_cmd_from_id(uint32_t can_id, uint8_t flags) {
  if(can_id & CAN_FRAME_EFF) {
    if(flags & CANFD_FDF) {
      return 'D';
    } else {
      if(can_id & CAN_FRAME_RTR) {
        return 'R';
      } else {
        return 'T';
      }
    }
  } else {
    if(flags & CANFD_FDF) {
      return 'd';
    } else {
      if(can_id & CAN_FRAME_RTR) {
        return 'r';
      } else {
        return 't';
      }
    }
  }
}

void slcan_can_rx_cb(struct pprzcan_frame* rxframe, struct pprzaddr_can* src_addr, void* user_data) {
  slcan_t* slcan = (slcan_t*) user_data;
  (void)src_addr;
  pprz_mtx_lock(&slcan->mtx);
  uint16_t idx = 0;
  slcan->tx_buf[idx++] = slcan_cmd_from_id(rxframe->can_id, rxframe->flags);
  if(rxframe->can_id & CAN_FRAME_EFF) {
    slcan->tx_buf[idx++] = nibble2hex((rxframe->can_id >> 28) & 0x01);
    slcan->tx_buf[idx++] = nibble2hex((rxframe->can_id >> 24));
    slcan->tx_buf[idx++] = nibble2hex((rxframe->can_id >> 20));
    slcan->tx_buf[idx++] = nibble2hex((rxframe->can_id >> 16));
    slcan->tx_buf[idx++] = nibble2hex((rxframe->can_id >> 12));
    slcan->tx_buf[idx++] = nibble2hex((rxframe->can_id >> 8) );
    slcan->tx_buf[idx++] = nibble2hex((rxframe->can_id >> 4) );
    slcan->tx_buf[idx++] = nibble2hex((rxframe->can_id >> 0) );
  } else {
    slcan->tx_buf[idx++] = nibble2hex((rxframe->can_id >> 8) & 0x07);
    slcan->tx_buf[idx++] = nibble2hex((rxframe->can_id >> 4));
    slcan->tx_buf[idx++] = nibble2hex((rxframe->can_id >> 0));
  }
  
  slcan->tx_buf[idx++] = nibble2hex(can_len_to_dlc(rxframe->len));
  for(int i=0; i<rxframe->len; i++) {
    slcan->tx_buf[idx++] = nibble2hex(rxframe->data[i] >> 4);
    slcan->tx_buf[idx++] = nibble2hex(rxframe->data[i]);
  }

  if(slcan->timestamp) {
    slcan->tx_buf[idx++] = nibble2hex((rxframe->timestamp >> 12));
    slcan->tx_buf[idx++] = nibble2hex((rxframe->timestamp >> 8));
    slcan->tx_buf[idx++] = nibble2hex((rxframe->timestamp >> 4));
    slcan->tx_buf[idx++] = nibble2hex((rxframe->timestamp >> 0));
  }

  slcan->tx_buf[idx++] = '\r';

  slcan->dev->put_buffer(slcan->dev->periph, 0, slcan->tx_buf, idx);
  slcan->dev->send_message(slcan->dev->periph, 0);

  pprz_mtx_unlock(&slcan->mtx);
}

static void slcan_rsp_ok(slcan_t* slcan) {
  slcan->dev->put_byte(slcan->dev->periph, 0, '\r');
  slcan->dev->send_message(slcan->dev->periph, 0);
}

static void slcan_rsp_error(slcan_t* slcan) {
  slcan->dev->put_byte(slcan->dev->periph, 0, 0x07);
  slcan->dev->send_message(slcan->dev->periph, 0);
}

/**
 * nibble2hex(2) == '2'
 */
static uint8_t nibble2hex(uint8_t x) {
  static uint8_t ConversionTable[] = {
      '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
  };
  return ConversionTable[x & 0x0F];
}

/**
 * hex2nibble('B') = 0x0B
 */
static uint8_t hex2nibble(char a) {
    if (a >= 'A' && a <= 'F') {
        return a - 'A' + 10;
    } else if (a >= 'a' && a <= 'f') {
        return a - 'a' + 10;
    } else if (a >= '0' && a <= '9') {
        return a - '0';
    }
    return 255;
}
