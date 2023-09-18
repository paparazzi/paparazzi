/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file "modules/actuators/actuators_faulhaber.c"
 * @author Freek van Tienen
 * Serial Connection module between ap and a faulhaber motor controller
 */

#include "modules/actuators/actuators_faulhaber.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "modules/datalink/downlink.h"
#include <stdio.h>
#include <string.h>

static struct uart_periph *faulhaber_dev = &(FAULHABER_DEV);

struct faulhaber_parser_t {
  uint8_t state;
  uint8_t node_nb;
  uint8_t cmd_code;
  uint8_t data_length;
  uint8_t data_idx;
  uint8_t data[64];

  uint8_t calc_crc8;
};
static struct faulhaber_parser_t faulhaber_p;
struct faulhaber_t faulhaber;

#define Polynom 0xD5
uint8_t faulhaber_crc8(uint8_t u8Byte, uint8_t u8CRC)
{
  uint8_t i;
  u8CRC = u8CRC ^ u8Byte;
  for (i = 0; i < 8; i++) {
    if (u8CRC & 0x01) {
    u8CRC = (u8CRC >> 1) ^ Polynom;
    }
    else {
    u8CRC >>= 1;
    }
  }
  return u8CRC;
}

static void faulhaber_send_command(struct uart_periph *dev, uint8_t cmd_code, uint8_t *data, uint8_t data_length) {
  uart_put_byte(dev, 0, 'S'); // Character (S) as Start of Frame
  uart_put_byte(dev, 0, data_length + 4); // Telegram length without SOF/EOF (packet length)
  uint8_t crc8 = faulhaber_crc8(data_length + 4, 0xFF); // Start CRC with 0xFF
  uart_put_byte(dev, 0, 0x01); // Node number of the slave (0 = Broadcast)
  crc8 = faulhaber_crc8(0x01, crc8);
  uart_put_byte(dev, 0, cmd_code); // See Tab. 2
  crc8 = faulhaber_crc8(cmd_code, crc8);
  for(uint8_t i = 0; i < data_length; i++) { // Data area (length = packet length – 4)
    uart_put_byte(dev, 0, data[i]);
    crc8 = faulhaber_crc8(data[i], crc8);
  }
  uart_put_byte(dev, 0, crc8); // CRC8 with polynomial 0xD5 over byte 2–N
  uart_put_byte(dev, 0, 'E'); // Character (E) as End of Frame
  uart_send_message(dev, 0);
}

#define UINIT           0
#define GOT_SOF         1
#define GOT_LENGTH      2
#define GOT_NODE_NB     3
#define GET_DATA        4
#define GET_CRC         5
#define GET_EOF         6
#define GOT_FULL_PACKET 7

static void faulhaber_parser(struct faulhaber_parser_t *p, uint8_t c) {
  switch (p->state) {
    case UINIT:
      if (c == 'S') {
        p->state = GOT_SOF;
        p->calc_crc8 = 0xFF;
      }
      break;
    case GOT_SOF:
      if(c - 4 < 0) {
        p->state = UINIT;
      } else {
        p->data_length = c - 4;
        p->calc_crc8 = faulhaber_crc8(c, p->calc_crc8);
        p->state = GOT_LENGTH;
      }
      break;
    case GOT_LENGTH:
      p->node_nb = c;
      p->calc_crc8 = faulhaber_crc8(c, p->calc_crc8);
      p->state = GOT_NODE_NB;
      break;
    case GOT_NODE_NB:
      p->cmd_code = c;
      p->data_idx = 0;
      p->calc_crc8 = faulhaber_crc8(c, p->calc_crc8);
      p->state = GET_DATA;
      break;
    case GET_DATA:
      p->data[p->data_idx++] = c;
      p->calc_crc8 = faulhaber_crc8(c, p->calc_crc8);
      if(p->data_idx >= p->data_length)
        p->state = GET_CRC;
      break;
    case GET_CRC:
      if(p->calc_crc8 == c) {
        p->state = GET_EOF;
      } else {
        p->state = UINIT;
      }
      break;
    case GET_EOF:
      if(c == 'E') {
        p->state = GOT_FULL_PACKET;
      } else {
        p->state = UINIT;
      }
      break;
    default:
      p->state = UINIT;
      break;
  }
}

// Controlword 0x6040.00
// <Statusword 0x6041.00
// 0x6060 (Mode of Operation) (6: HOME, 1: PP)
// <0x6061.00(Mode of Operation feedback)
// 0x607A.00 (Target Position)
// <0x607A.00 (Actual Position)
// Start positioning via the rising edge in bit 4 of the controlword. Also set the optional bits here.

void actuators_faulhaber_init(void) {
  faulhaber_p.state = UINIT;

  faulhaber.mode = FH_MODE_INIT;
  faulhaber.state = 0;
  faulhaber.target_position = 0;
}


void actuators_faulhaber_periodic(void) {
  switch(faulhaber.mode) {

    /* HOME MODE */
    case FH_MODE_HOME:
      switch(faulhaber.state) {
        case 0: {
          // Set the homing mode
          uint8_t data[] = { 0x60, 0x60, 0x00, 0x06 }; // Set 0x6060.00 to 0x06:  Homing mode
          faulhaber_send_command(faulhaber_dev, 0x02, data, 4);
          faulhaber.state++;
          break;
        }
        case 1: {
          // Set the homing method
          uint8_t data[] = { 0x98, 0x60, 0x00, 0x11 }; // Set 0x6098.00 to 0x11:  Homing method to 17
          faulhaber_send_command(faulhaber_dev, 0x02, data, 4);
          faulhaber.state++;
          break;
        }
        case 2: {
          // Enable operation
          uint8_t data[] = { 0x40, 0x60, 0x00, 0x0F, 0x00}; // Set 0x6040.00 to 0x000F: Enable operation
          faulhaber_send_command(faulhaber_dev, 0x02, data, 5);
          faulhaber.state++;
          break;
        }
        case 3: {
          // Start moving
          uint8_t data[] = { 0x40, 0x60, 0x00, 0x1F, 0x00}; // Set 0x6040.00 to 0x001F: Start moving
          faulhaber_send_command(faulhaber_dev, 0x02, data, 5);
          faulhaber.state++;
          break;
        }
        default:
          // Goto position mode
          faulhaber.mode = FH_MODE_IDLE;
          faulhaber.state = 0;
          break;
      }
      break;

    /* POSITION MODE */
    case FH_MODE_POSITION:
      switch(faulhaber.state) {
        case 0: {
          uint8_t data[] = { 0x60, 0x60, 0x00, 0x01 }; // Set 0x6060.00 to 0x01:  Position mode
          faulhaber_send_command(faulhaber_dev, 0x02, data, 4);
          faulhaber.state++;
          break;
        }
        case 1: {
          // Set the target position
          uint8_t data[] = { 0x7A, 0x60, 0x00, (faulhaber.target_position & 0xFF), ((faulhaber.target_position >> 8) & 0xFF), ((faulhaber.target_position >> 16) & 0xFF), ((faulhaber.target_position >> 24) & 0xFF) }; // Set 0x607A.00 to 0x00000000:  Target position 0
          faulhaber_send_command(faulhaber_dev, 0x02, data, 7);
          faulhaber.state++;
          break;
        }
        case 2: {
          // Enable operation
          uint8_t data[] = { 0x40, 0x60, 0x00, 0x0F, 0x00}; // Set 0x6040.00 to 0x000F: Enable operation
          faulhaber_send_command(faulhaber_dev, 0x02, data, 5);
          faulhaber.state++;
          break;
        }
        case 3: {
          // Start moving
          uint8_t data[] = { 0x40, 0x60, 0x00, 0x3F, 0x00}; // Set 0x6040.00 to 0x003F: Start moving
          faulhaber_send_command(faulhaber_dev, 0x02, data, 5);
          faulhaber.state++;
          break;
        }
        default:
          faulhaber.mode = FH_MODE_IDLE;
          faulhaber.state = 0;
          break;

      }
      break;

    /* FH_MODE_IDLE */
    case FH_MODE_IDLE: {
      uint8_t data[] = { 0x64, 0x60, 0x00}; // Get 0x6064.00: Get the actual position
      faulhaber_send_command(faulhaber_dev, 0x01, data, 3);
      faulhaber.state = 0;
      break;
    }

    /* Do nothing */
    default:
      break;
  }

}

static void faulhaber_parse_msg(struct faulhaber_parser_t *p) {
  if(p->cmd_code == 0x01 && p->data[0] == 0x64 && p->data[1] == 0x60 && p->data[2] == 0x00) {
    faulhaber.real_position = p->data[3] | (p->data[4] << 8) | (p->data[5] << 16) | (p->data[6] << 24);
  }
}


void actuators_faulhaber_event(void) {
  while (uart_char_available(faulhaber_dev)) {
    faulhaber_parser(&faulhaber_p, uart_getch(faulhaber_dev));
    if (faulhaber_p.state == GOT_FULL_PACKET) {
      faulhaber_parse_msg(&faulhaber_p);
      faulhaber_p.state = UINIT;
    }
  }
}

void actuators_faulhaber_SetMode(uint8_t mode) {
  faulhaber.mode = mode;
  faulhaber.state = 0;
}
