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
#include "modules/core/abi.h"
#include <stdio.h>
#include <string.h>

/* Proportional gain on the velocity for the position controller */
#ifndef FAULHABER_P_GAIN
#define FAULHABER_P_GAIN 0.07
#endif

/* Maximum velocity for the position controller */
#ifndef FAULHABER_MAX_VELOCITY
#define FAULHABER_MAX_VELOCITY 12000
#endif

static struct uart_periph *faulhaber_dev = &(FAULHABER_DEV);

/* Faulhaber message parser */
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
static uint8_t faulhaber_crc8(uint8_t u8Byte, uint8_t u8CRC)
{
  uint8_t i;
  u8CRC = u8CRC ^ u8Byte;
  for (i = 0; i < 8; i++) {
    if (u8CRC & 0x01) {
      u8CRC = (u8CRC >> 1) ^ Polynom;
    } else {
      u8CRC >>= 1;
    }
  }
  return u8CRC;
}

static void faulhaber_send_command(struct uart_periph *dev, uint8_t cmd_code, uint8_t *data, uint8_t data_length)
{
  uart_put_byte(dev, 0, 'S'); // Character (S) as Start of Frame
  uart_put_byte(dev, 0, data_length + 4); // Telegram length without SOF/EOF (packet length)
  uint8_t crc8 = faulhaber_crc8(data_length + 4, 0xFF); // Start CRC with 0xFF
  uart_put_byte(dev, 0, 0x01); // Node number of the slave (0 = Broadcast)
  crc8 = faulhaber_crc8(0x01, crc8);
  uart_put_byte(dev, 0, cmd_code); // See Tab. 2
  crc8 = faulhaber_crc8(cmd_code, crc8);
  for (uint8_t i = 0; i < data_length; i++) { // Data area (length = packet length – 4)
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

static void faulhaber_parser(struct faulhaber_parser_t *p, uint8_t c)
{
  switch (p->state) {
    case UINIT:
      if (c == 'S') {
        p->state = GOT_SOF;
        p->calc_crc8 = 0xFF;
      }
      break;
    case GOT_SOF:
      if (c - 4 < 0) {
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
      if (p->data_idx >= p->data_length) {
        p->state = GET_CRC;
      }
      break;
    case GET_CRC:
      if (p->calc_crc8 == c) {
        p->state = GET_EOF;
      } else {
        p->state = UINIT;
      }
      break;
    case GET_EOF:
      if (c == 'E') {
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

void actuators_faulhaber_init(void)
{
  faulhaber_p.state = UINIT;

  faulhaber.mode = FH_MODE_INIT;
  faulhaber.state = 0;
  faulhaber.setpoint_position = 0;
  faulhaber.real_position = 0;
  faulhaber.homing_completed = false;
  faulhaber.position_ready = false;
  faulhaber.target_reached = false;
  faulhaber.p_gain = FAULHABER_P_GAIN;
  faulhaber.max_velocity = FAULHABER_MAX_VELOCITY;
}

void actuators_faulhaber_periodic(void)
{
  static float last_time = 0;
  switch (faulhaber.mode) {

    /* Initialize and home the motor */
    case FH_MODE_INIT:
      switch(faulhaber.state) {
        case 0: {
          // Start the controller boot timer
          last_time = get_sys_time_float();
          faulhaber.state++;
          break;
        }
        case 1: {
          // Check if the controller is ready, 5 seconds delay
          if (get_sys_time_float() - last_time < 5.0)
            break;
          
          // Enable power on the motor
          static uint8_t data[] = { 0x40, 0x60, 0x00, 0x0E, 0x00}; // Set 0x6040.00 to 0x000E: Try to start
          faulhaber_send_command(faulhaber_dev, 0x02, data, 5);
          last_time = get_sys_time_float();
          faulhaber.state++;
          break;
        }
        case 2: {
          // Wait 10ms for the next command
          if(get_sys_time_float() - last_time < 0.01)
            break;
          
          // Set to homing mode
          static uint8_t data[] = { 0x60, 0x60, 0x00, 0x06 }; // Set 0x6060.00 to 0x06:  Homing mode
          faulhaber_send_command(faulhaber_dev, 0x02, data, 4);
          last_time = get_sys_time_float();
          faulhaber.state++;
          break;
        }
        case 3: {
          // Wait 10ms for the next command
          if(get_sys_time_float() - last_time < 0.01)
            break;
          
          // Set the homing method
          static uint8_t data[] = { 0x98, 0x60, 0x00, 0x11 }; // Set 0x6098.00 to 0x11:  Homing method to 17
          faulhaber_send_command(faulhaber_dev, 0x02, data, 4);
          last_time = get_sys_time_float();
          faulhaber.state++;
          break;
        }
        case 4: {
          // Wait 10ms for the next command
          if(get_sys_time_float() - last_time < 0.01)
            break;
          
          // Enable operation
          static uint8_t data[] = { 0x40, 0x60, 0x00, 0x0F, 0x00}; // Set 0x6040.00 to 0x000F: Enable operation
          faulhaber_send_command(faulhaber_dev, 0x02, data, 5);
          last_time = get_sys_time_float();
          faulhaber.state++;
          break;
        }
        case 5: {
          // Wait 10ms for the next command
          if(get_sys_time_float() - last_time < 0.01)
            break;
          
          // Start moving
          static uint8_t data[] = { 0x40, 0x60, 0x00, 0x1F, 0x00}; // Set 0x6040.00 to 0x001F: Start moving
          faulhaber_send_command(faulhaber_dev, 0x02, data, 5);
          last_time = get_sys_time_float();
          faulhaber.state++;
          break;
        }
        case 6: {
          // Check if homing is completed
          if(faulhaber.homing_completed) {
            // Continue initialization
            last_time = get_sys_time_float();
            faulhaber.state++;
          }
          // Timeout after 40 seconds
          else if(get_sys_time_float() - last_time > 40.0) {
            faulhaber.mode = FH_MODE_ERROR;
            faulhaber.state = 0;
          }
          break;
        }
        case 7: {
          // Wait 10ms for the next command
          if(get_sys_time_float() - last_time < 0.01)
            break;
          
          // Set to velocity mode
          static uint8_t data[] = { 0x60, 0x60, 0x00, 0x03 }; // Set 0x6060.00 to 0x03: Velocity mode
          faulhaber_send_command(faulhaber_dev, 0x02, data, 4);
          last_time = get_sys_time_float();
          faulhaber.state++;
          break;
        }
        case 8: {
          // Wait 10ms for the next command
          if(get_sys_time_float() - last_time < 0.01)
            break;
          
          // Enable operation
          static uint8_t data[] = { 0x40, 0x60, 0x00, 0x0F, 0x00}; // Set 0x6040.00 to 0x000F: Enable operation
          faulhaber_send_command(faulhaber_dev, 0x02, data, 5);
          last_time = get_sys_time_float();
          faulhaber.state++;
          break;
        }
        case 9: {
          // Wait 10ms for the next command
          if(get_sys_time_float() - last_time < 0.01)
            break;
          
          // Go to position mode
          faulhaber.mode = FH_MODE_VELOCITY;
          faulhaber.state = 0;
        }
      }
      break;

    /* FH_MODE_VELOCITY */
    case FH_MODE_VELOCITY: {
      // Request position and set the new velocity target
      switch (faulhaber.state) {
        case 0: {
          // Request the position
          uint8_t data[] = { 0x64, 0x60, 0x00}; // Get 0x6064.00: Get the actual position
          faulhaber_send_command(faulhaber_dev, 0x01, data, 3);
          faulhaber.state++;
          break;
        }
        default: {
          // Calculate the new velocity target
          int32_t poss_err = (faulhaber.setpoint_position - faulhaber.real_position);
          faulhaber.target_velocity = poss_err * faulhaber.p_gain;
          BoundAbs(faulhaber.target_velocity, faulhaber.max_velocity);

          // Set the new velocity target
          uint8_t data[] = { 0xFF, 0x60, 0x00, (faulhaber.target_velocity & 0xFF), ((faulhaber.target_velocity >> 8) & 0xFF), ((faulhaber.target_velocity >> 16) & 0xFF), ((faulhaber.target_velocity >> 24) & 0xFF) }; // Set 0x60FF.00 to [velocity]:  Target velocity
          faulhaber_send_command(faulhaber_dev, 0x02, data, 7);
          faulhaber.state = 0;
          break;
        }
      }
      break;
    }

    /* FH_MODE_REQ_ERR */
    case FH_MODE_REQ_ERR: {
      switch(faulhaber.state) {
        case 0: {
          // Request the status code
          uint8_t data[] = { 0x20, 0x23, 0x00}; // Get 0x2320.00: Get the error code
          faulhaber_send_command(faulhaber_dev, 0x01, data, 3);
          faulhaber.state++;
          break;
        }
        case 1: {
          // Request the status code
          uint8_t data[] = { 0x21, 0x23, 0x00}; // Get 0x2321.00: Get the error mask
          faulhaber_send_command(faulhaber_dev, 0x01, data, 3);
          faulhaber.state++;
          break;
        }
        case 2: {
          // Request the status code
          uint8_t data[] = { 0x01, 0x10, 0x00}; // Get 0x1001.00: Get the error mask
          faulhaber_send_command(faulhaber_dev, 0x01, data, 3);
          faulhaber.state++;
          break;
          
        }
        default: {
          // Do nothing and stop requesting
          break;
        }
      }
      break;
    }

    /* FH_MODE_RESET_ERR */
    case FH_MODE_RESET_ERR: {
      switch(faulhaber.state) {
        case 0: {
          static uint8_t data[] = { 0x40, 0x60, 0x00, 0x0E, 0x00}; // Set 0x6040.00 to 0x000E: Try to start
          faulhaber_send_command(faulhaber_dev, 0x02, data, 5);
          faulhaber.state++;
          break;
        }
        case 1: {
          static uint8_t data[] = { 0x60, 0x60, 0x00, 0x03 }; // Set 0x6060.00 to 0x03: Velocity mode
          faulhaber_send_command(faulhaber_dev, 0x02, data, 4);
          faulhaber.state++;
          break;
        }
        default: {
          // Enable operation
          static uint8_t data[] = { 0x40, 0x60, 0x00, 0x0F, 0x00}; // Set 0x6040.00 to 0x000F: Enable operation
          faulhaber_send_command(faulhaber_dev, 0x02, data, 5);
          faulhaber.state = 0;
          faulhaber.mode = FH_MODE_VELOCITY;
          break;
        }
      }
      break;
    }

    /* Do nothing */
    default:
      break;
  }

}

static void faulhaber_parse_msg(struct faulhaber_parser_t *p)
{
  // Process position message
  if (p->cmd_code == 0x01 && p->data[0] == 0x64 && p->data[1] == 0x60 && p->data[2] == 0x00) {
    faulhaber.real_position = p->data[3] | (p->data[4] << 8) | (p->data[5] << 16) | (p->data[6] << 24);

    struct act_feedback_t feedback = {0};
    feedback.position = ((faulhaber.real_position/ACTUATORS_FAULHABER_COMMAND_SCALE) - get_servo_min_FAULHABER(0)) / (double)(get_servo_max_FAULHABER(
                          0) - get_servo_min_FAULHABER(0)) * M_PI_2; // In radians
    feedback.set.position = true;
    feedback.idx = get_servo_idx_FAULHABER(0);

    // Send ABI message
    AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_FAULHABER_ID, &feedback, 1);
  }
  // Write requests
  else if(p->cmd_code == 0x02) {
    // Do nothing
  }
  // Parse the statuscode message
  else if(p->cmd_code == 0x05) {
    uint16_t status_code = p->data[0] | (p->data[1] << 8);

    // Homing
    if(!faulhaber.homing_completed && status_code&0x1000 && status_code&0x400)
      faulhaber.homing_completed = true;
    
    // Position accepted
    if(!faulhaber.position_ready && status_code&0x1000)
      faulhaber.position_ready = true;
      
    // Target reached
    if(!faulhaber.target_reached && status_code&0x400)
      faulhaber.target_reached = true;
    
    // If the drive got disabled
    if(!(status_code&0x0001) || !(status_code&0x0002) || !(status_code&0x0004)) {
      char error_msg[250];
      int rc = snprintf(error_msg, 200, "[FH]%d %04X", p->cmd_code, status_code);
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, error_msg);
    }
  }
  else {
    // Unknown message
    char error_msg[250];
    int rc = snprintf(error_msg, 200, "[FH]%d ", p->cmd_code);
    for(int i = 0; i < p->data_length; i++) {
      rc += snprintf(error_msg + rc, 200 - rc, "%02X", p->data[i]);
    }

    if (rc > 0) {
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, error_msg);
    }
  }
}


void actuators_faulhaber_event(void)
{
  while (uart_char_available(faulhaber_dev)) {
    faulhaber_parser(&faulhaber_p, uart_getch(faulhaber_dev));
    if (faulhaber_p.state == GOT_FULL_PACKET) {
      faulhaber_parse_msg(&faulhaber_p);
      faulhaber_p.state = UINIT;
    }
  }
}

void actuators_faulhaber_SetMode(uint8_t mode)
{
  faulhaber.mode = mode;
  faulhaber.state = 0;
}
