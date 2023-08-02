/*
 * Copyright (C) 2023 Flo&Fab <surname.name@enac.fr>
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

/** @file "modules/actuators/servo_sts3032.h"
 * @author Flo&Fab <surname.name@enac.fr>
 * feetech sts3032 servo
 */

#ifndef SERVO_STS3032_H
#define SERVO_STS3032_H

#include <stdint.h>
#include "mcu_periph/uart.h"
#include "utils/circular_buffer.h"
#include "generated/airframe.h"


enum sts3032_rx_state {
  STS3032_RX_IDLE,
  STS3032_RX_HEAD_OK,
  STS3032_RX_GOT_LENGTH,
};

struct sts3032 {
  struct uart_periph *periph;


  uint8_t ids[SERVOS_STS3032_NB];
  uint8_t states[SERVOS_STS3032_NB];
  uint16_t pos[SERVOS_STS3032_NB];

  uint32_t time_last_msg; // last send msg time


  enum sts3032_rx_state rx_state;
  uint8_t nb_bytes_expected;
  uint8_t buf_header[2];
  uint8_t rx_id;
  uint8_t rx_checksum;
  bool echo; // wait for sended message
  uint8_t read_addr;
  bool wait_reply;
  uint16_t nb_failed_checksum;

  struct circular_buffer msg_buf;
};


extern struct sts3032 sts;

extern void actuators_sts3032_init(void);
extern void actuators_sts3032_periodic(void);
extern void actuators_sts3032_event(void);
uint8_t id_idx(struct sts3032 *sts, uint8_t id);
void sts3032_write_pos(struct sts3032 *sts, uint8_t id, int16_t position);

/* Actuator macros */
#define ActuatorSTS3032Set(_i, _v) { if(sts3032_enabled) {sts3032_write_pos(&sts, sts.ids[_i], _v);} }
#define ActuatorsSTS3032Init() actuators_sts3032_init()
#define ActuatorsSTS3032Commit() {}

extern int sts3032_enabled;
extern int sts3032_current_id;
extern int sts3032_future_id;
extern int sts3032_lock_eprom;
extern int sts3032_move;
extern int sts3032_response_level;

extern void actuators_sts3032_set_id(float future_id);
extern void actuators_sts3032_lock_eprom(float lock);
extern void actuators_sts3032_move(float pos);
extern void actuators_sts3032_set_response_level(float level);

#endif  // SERVO_STS3032_H
