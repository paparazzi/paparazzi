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

/** @file "modules/actuators/servo_sts3032.c"
 * @author Flo&Fab <surname.name@enac.fr>
 * feetech sts3032 servo 
 */

#include "modules/actuators/actuators_sts3032.h"
#include "modules/datalink/telemetry.h"
#include "string.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/sys_time_arch.h"
#include <stdint.h>
#include "peripherals/sts3032_regs.h"

#ifndef STS3032_IDS
#error "STS3032_IDS must be defined"
#endif

#ifndef STS3032_DEBUG
#define STS3032_DEBUG FALSE
#endif

#ifndef STS3032_RESPONSE_LEVEL
#define STS3032_RESPONSE_LEVEL 0
#endif

struct sts3032 sts;
uint8_t cbuf[50*SERVOS_STS3032_NB];		// buffer large enough to queue a command to each servo

#ifndef STS3032_DELAY_MSG
#define STS3032_DELAY_MSG 1000
#endif

#ifndef STS3032_DELAY_MSG_MIN
#define STS3032_DELAY_MSG_MIN 0
#endif

#define BUF_MAX_LENGHT 15
uint8_t buf_rx_param[255];



/****** settings globals *********/
int sts3032_enabled = 1;
int sts3032_current_id = 1;
int sts3032_future_id = 1;
int sts3032_lock_eprom = 1;
int sts3032_move = 1024;
/**********************************/


static void sts3032_event(struct sts3032 *sts);
static void write_buf(struct sts3032 *sts, uint8_t id, uint8_t *data, uint8_t len, uint8_t fun, uint8_t reply);
static void handle_reply(struct sts3032 *sts, uint8_t id, uint8_t state, uint8_t* buf_param, uint8_t length);

void sts3032_write_pos(struct sts3032 *sts, uint8_t id, int16_t position);
void sts3032_event(struct sts3032 *sts);
void sts3032_read_mem(struct sts3032 *sts, uint8_t id, uint8_t addr, uint8_t len);
void sts3032_init(struct sts3032 *sts, struct uart_periph *periph, uint8_t* cbuf, size_t cbuf_len);
void sts3032_read_pos(struct sts3032 *sts, uint8_t id);
void sts3032_enable_torque(struct sts3032 *sts, uint8_t id, uint8_t enable);
void sts3032_lock_eeprom(struct sts3032 *sts, uint8_t id, uint8_t lock);
void sts3032_set_response_level(struct sts3032 *sts, uint8_t id, uint8_t level);
void sts3032_set_id(struct sts3032 *sts, uint8_t id, uint8_t new_id);


void actuators_sts3032_init(void)
{
  sts.periph = &(STS3032_DEV);
	sts.rx_state = STS3032_RX_IDLE;
	sts.nb_bytes_expected = 1;
	sts.nb_failed_checksum = 0;
	circular_buffer_init(&sts.msg_buf, cbuf, sizeof(cbuf));

  static const uint8_t tmp_ids[SERVOS_STS3032_NB] = STS3032_IDS;
  memcpy(sts.ids, tmp_ids, sizeof(tmp_ids));
}

void actuators_sts3032_event(void)
{
  sts3032_event(&sts);
}

void actuators_sts3032_periodic(void)
{

	for(int i=0; i<SERVOS_STS3032_NB; i++) {
		sts3032_read_pos(&sts, sts.ids[i]);
	}

#if STS3032_DEBUG
	float data[SERVOS_STS3032_NB + 1];
	data[0] = sts.nb_failed_checksum;
	for(int i=0; i<SERVOS_STS3032_NB; i++) {
		data[i+1] = sts.pos[i];
	}
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, SERVOS_STS3032_NB + 1, data);
#endif

}



static void sts3032_event(struct sts3032 *sts) {
	
	uint32_t now = get_sys_time_usec();
	uint32_t dt = now - sts->time_last_msg;
	if((!sts->wait_reply && dt > STS3032_DELAY_MSG_MIN) || dt > STS3032_DELAY_MSG) {
		uint8_t buf[BUF_MAX_LENGHT];
		int size = circular_buffer_get(&sts->msg_buf, buf, BUF_MAX_LENGHT);
		if(size > 0) {
			// first byte of the buffer indicate whether or not a replay is expected
			// the rest is the message itself
			uart_put_buffer(sts->periph, 0, buf+1, size-1);
			sts->wait_reply = buf[0];
			sts->time_last_msg = now;
			sts->echo = true;
		}
	}
	
	while(uart_char_available(sts->periph) >= sts->nb_bytes_expected) {
		switch (sts->rx_state)
		{
		case STS3032_RX_IDLE:
			if(uart_char_available(sts->periph) > 0){
				sts->buf_header[1] = sts->buf_header[0];
				sts->buf_header[0] = uart_getch(sts->periph);
				if(sts->buf_header[0] == 0xFF && sts->buf_header[1]== 0xFF) {
					sts->rx_state = STS3032_RX_HEAD_OK;
					sts->nb_bytes_expected = 2;
					sts->buf_header[0] = 0;
					sts->buf_header[1] = 0;
				}
			}
			break;
		case STS3032_RX_HEAD_OK:
			if(uart_char_available(sts->periph) >= sts->nb_bytes_expected){
				sts->rx_id = uart_getch(sts->periph);
				sts->nb_bytes_expected = uart_getch(sts->periph); //length
				sts->rx_checksum = sts->rx_id + sts->nb_bytes_expected;
				sts->rx_state = STS3032_RX_GOT_LENGTH;
			}
			break;
		case STS3032_RX_GOT_LENGTH:
			if(uart_char_available(sts->periph) >= sts->nb_bytes_expected){
				uint8_t current_state = uart_getch(sts->periph);
				sts->rx_checksum += current_state;
				for(int i = 0; i < sts->nb_bytes_expected-2; i++) {
					buf_rx_param[i] = uart_getch(sts->periph);
					sts->rx_checksum += buf_rx_param[i];
				}
				uint8_t checksum = uart_getch(sts->periph);
				sts->rx_checksum = ~sts->rx_checksum;
				if(sts->rx_checksum == checksum) {
					if(!sts->echo) {
						handle_reply(sts, sts->rx_id, current_state, buf_rx_param, sts->nb_bytes_expected-2);
					}
				}
				else {
					sts->nb_failed_checksum++;
				}
				sts->echo = false;
				sts->rx_state = STS3032_RX_IDLE;
				sts->nb_bytes_expected = 1;
			}
			break;
		default:
			break;
		}
	}
}

static void handle_reply(struct sts3032 *sts, uint8_t id, uint8_t state, uint8_t* buf_param, uint8_t length) {
	uint8_t index = id_idx(sts, id);
	if(index == 255) {
		return;
	}
	sts->states[index] = state; 
	uint8_t offset = 0;
	while(length > offset) {

		switch (sts->read_addr + offset)
		{
		case SMS_STS_PRESENT_POSITION_L:
			if(offset + 2 < length) {
				return;
			}
			sts->pos[index] =  buf_param[offset] | (buf_param[offset + 1] << 8);
			offset += 2;
			break;
		
		default:
		  offset++;
			break;
		}
	}
}


uint8_t id_idx(struct sts3032 *sts, uint8_t id) {
	for(int i = 0; i<SERVOS_STS3032_NB; i++) {
		if(sts->ids[i] == id){
			return i;
		}
	}
	return 255;
}


void sts3032_lock_eeprom(struct sts3032 *sts, uint8_t id, uint8_t lock) {
	// lock = 0: unlock eeprom
	// lock = 1: lock eeprom
	uint8_t buf[2];
  buf[0] = SMS_STS_LOCK;
  buf[1] = lock;
  write_buf(sts, id, buf, 2, INST_WRITE, STS3032_RESPONSE_LEVEL);
}

void sts3032_set_response_level(struct sts3032 *sts, uint8_t id, uint8_t level) {
	// response level: 0: reply only on read and ping
	//                 1: always repond
	uint8_t buf[2];
  buf[0] = SMS_STS_RESPONSE_LEVEL;
  buf[1] = level;
  write_buf(sts, id, buf, 2, INST_WRITE, STS3032_RESPONSE_LEVEL);
}

void sts3032_set_id(struct sts3032 *sts, uint8_t id, uint8_t new_id) {
	uint8_t buf[2];
  buf[0] = SMS_STS_ID;
  buf[1] = new_id;
  write_buf(sts, id, buf, 2, INST_WRITE, STS3032_RESPONSE_LEVEL);
}

void sts3032_write_pos(struct sts3032 *sts, uint8_t id, int16_t position){
  if (position < 0)
	{
		position = -position;
		position |= (1 << 15);
	}
  uint8_t buf[3];
  buf[0] = SMS_STS_GOAL_POSITION_L;
  buf[1] = position & 0xff; 
  buf[2] = position >> 8;
  write_buf(sts, id, buf, 3, INST_WRITE, STS3032_RESPONSE_LEVEL);
}

void sts3032_enable_torque(struct sts3032 *sts, uint8_t id, uint8_t enable) {
	uint8_t buf[2];
  buf[0] = SMS_STS_TORQUE_ENABLE;
  buf[1] = enable; 
	write_buf(sts, id, buf, 2, INST_WRITE, STS3032_RESPONSE_LEVEL);
}

void sts3032_read_pos(struct sts3032 *sts, uint8_t id) {
	sts3032_read_mem(sts, id, SMS_STS_PRESENT_POSITION_L, 2);
}

void sts3032_read_mem(struct sts3032 *sts, uint8_t id, uint8_t addr, uint8_t len) {
	uint8_t buf[2] = {addr, len};
	write_buf(sts, id, buf, 2, INST_READ, 1);
	sts->read_addr = addr;
}

static void write_buf(struct sts3032 *sts, uint8_t id, uint8_t *data, uint8_t len, uint8_t fun, uint8_t reply) {
	uint8_t buf[BUF_MAX_LENGHT];
	uint8_t checksum = id + fun + len + 2;

	buf[0] = reply;	// wait for a reply to this message?

	buf[1] = 0xff;
	buf[2] = 0xff;
	buf[3] = id;
  buf[4] = 2 + len;
	buf[5] = fun;
  memcpy(&buf[6], data, len);
  for (int i = 0; i < len; i++){
		checksum += data[i];
	}
  buf[len + 6] = ~checksum;

	circular_buffer_put(&sts->msg_buf, buf, len + 7);
}


/******************** settings handlers **********************/
void actuators_sts3032_set_id(float future_id) {
	sts3032_future_id = (int)(future_id+0.5);
	sts3032_set_id(&sts, (uint8_t)sts3032_current_id, sts3032_future_id);
}

void actuators_sts3032_lock_eprom(float lock) {
	sts3032_lock_eprom = (int)(lock+0.5);
	sts3032_lock_eeprom(&sts, (uint8_t)sts3032_current_id, sts3032_lock_eprom);
}

void actuators_sts3032_move(float pos) {
	sts3032_move = pos;
	sts3032_write_pos(&sts, sts3032_current_id, sts3032_move);
}

void actuators_sts3032_set_response_level(float level) {
	sts3032_response_level = (int)(level+0.5);
	sts3032_set_response_level(&sts, sts3032_current_id, sts3032_response_level);
}

/************************************************************/
