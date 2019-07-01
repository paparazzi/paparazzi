/*
 * Copyright (C) Fabien Bonneval
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
 * @file "modules/actuators/actuators_ostrich.c"
 * @author Fabien Bonneval
 * Driver for the Ostrich rover controller board
 */

#include "modules/actuators/actuators_ostrich.h"
#include "mcu_periph/uart.h"
#include "generated/airframe.h"

#define SPEED_NEUTRAL 500
#define SPEED_FACTOR 1.0
#define TURN_NEUTRAL 500
#define TURN_FACTOR 0.5

#define SPEED_OF_CMD(_s) ((_s-SPEED_NEUTRAL)*SPEED_FACTOR)
#define TURN_OF_CMD(_w) ((_w-TURN_NEUTRAL)*TURN_FACTOR)

struct ActuatorsOstrich actuators_ostrich;

#define START_BYTE 0x7F

struct SpeedMessagePayload {
  uint16_t vx;
  uint16_t vy;
  uint16_t vtheta;
} __attribute__((packed)) ;

union rawData {
  struct SpeedMessagePayload data;
  uint8_t bytes[6];
} __attribute__((packed)) ;

struct SpeedMessage  {
  uint8_t start_byte;
  uint8_t msg_type;
  uint8_t checksum;
  union rawData raw_data;
} __attribute__((packed)) ;

union RawMessage {
  struct SpeedMessage speed_message;
  uint8_t bytes[9];
} __attribute__((packed)) ;

// uart periph
static struct uart_periph *ostrich_dev;

static uint16_t speed_cmd_to_msg(uint16_t speed_cmd)
{
  return ((SPEED_OF_CMD(speed_cmd) + 3200.0) * 10.0);
}

static uint16_t turn_cmd_to_msg(uint16_t turn_cmd)
{
  return ((TURN_OF_CMD(turn_cmd) + 500.0) * 50);
}

static uint8_t compute_checksum(uint8_t bytes[], int len)
{
  uint8_t checksum = 0;
  for (int i = 0; i < len; i++) {
    checksum += bytes[i];
  }
  return checksum;
}


void actuators_ostrich_init()
{
  ostrich_dev = &(ACTUATORS_OSTRICH_DEV);
  actuators_ostrich.cmds[0] = SPEED_NEUTRAL;
  actuators_ostrich.cmds[1] = SPEED_NEUTRAL;
  actuators_ostrich.cmds[2] = TURN_NEUTRAL;
}

void actuators_ostrich_periodic()
{
  uint16_t speed_msg = speed_cmd_to_msg(actuators_ostrich.cmds[0]);
  uint16_t speed_y_msg = speed_cmd_to_msg(actuators_ostrich.cmds[1]);
  uint16_t turn_msg = turn_cmd_to_msg(actuators_ostrich.cmds[2]);

  union RawMessage raw_message;
  raw_message.speed_message.start_byte = START_BYTE;
  raw_message.speed_message.msg_type = 1;

  raw_message.speed_message.raw_data.data.vx = speed_msg;
  raw_message.speed_message.raw_data.data.vy = speed_y_msg;
  raw_message.speed_message.raw_data.data.vtheta = turn_msg;

  raw_message.speed_message.checksum = compute_checksum(raw_message.speed_message.raw_data.bytes, 6);

  uart_put_buffer(ostrich_dev, 0, raw_message.bytes, 9);
}


