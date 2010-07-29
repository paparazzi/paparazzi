/*
 * $Id:$
 *
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
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

#include <stdint.h>
#include <string.h>

#include "csc_protocol.h"
#include "can.h"
#include "csc_msg_def.h"


#define CSCP_QUEUE_LEN 8

struct cscp_msg {
	uint8_t id;
	uint8_t len;
	uint8_t data[8];
};

struct cscp_msg_queue {
	int head;
	int tail;
	int full;
	struct cscp_msg msgs[CSCP_QUEUE_LEN];
} cscp_msg_queue;

/*
 * This handle is being used internally in the protocol driver
 * data points to user supplied memory (user allocated struct)
 * this way we make sure that the user allocates only the amount
 * of memory necessary and we don't have to mess around with malloc
 */
struct cscp_callback_handle {
	cscp_callback_t callback;
	void *data;
};

struct cscp_callback_handle cscp_callback_handles[CSC_ID_COUNT];

void cscp_can_rx_callback(uint32_t id, uint8_t *buf, int len);
void cscp_queue_init(void);
int cscp_enqueue(uint32_t msg_id, uint8_t *buf, int len);
int cscp_peek_msg_id(void);
int cscp_dequeue(uint8_t *buf);
void cscp_callback_init(void);

void cscp_init(void)
{
	can_init(cscp_can_rx_callback);
	cscp_queue_init();
	cscp_callback_init();
}

static inline uint32_t cscp_can_id(uint8_t client_id, uint8_t msg_id)
{
	return ((client_id & 0xF) << 7) | (msg_id & 0x7F);
}

static inline uint32_t cscp_msg_id(uint32_t id)
{
	return (id & 0x7F);
}

int cscp_transmit(uint32_t client_id, uint8_t msg_id, const uint8_t *buf, uint8_t len)
{
	uint32_t id = cscp_can_id(client_id, msg_id);
	return can_transmit(id, buf, len);
}

void cscp_can_rx_callback(uint32_t id, uint8_t *buf, int len)
{
	/* just store the incoming data in a buffer with very little processing */

	/* TODO we should handle the return value of enqueue somehow,
	 * the returnvalue tells us if the queue had enough space to
	 * store our message or not
	 */
	uint32_t msg_id = cscp_msg_id(id);
	cscp_enqueue(msg_id, buf, len);
}

void cscp_event(void)
{
	/* this periodig is the main csc protocol event dispatcher */
	int msg_id = cscp_peek_msg_id();

	if(msg_id == -1){
		return;
	}

	if(cscp_callback_handles[msg_id].callback){
		cscp_dequeue(cscp_callback_handles[msg_id].data);
		cscp_callback_handles[msg_id].callback(cscp_callback_handles[msg_id].data);
	}

}

void cscp_queue_init(void)
{
	cscp_msg_queue.head = 0;
	cscp_msg_queue.tail = 0;
	cscp_msg_queue.full = 0;
}

int cscp_enqueue(uint32_t msg_id, uint8_t *buf, int len)
{
	if (cscp_msg_queue.full) {
		return 1;
	}

	cscp_msg_queue.msgs[cscp_msg_queue.tail].id = msg_id;
	cscp_msg_queue.msgs[cscp_msg_queue.tail].len = len;
	memcpy(cscp_msg_queue.msgs[cscp_msg_queue.tail].data, buf, len);
	cscp_msg_queue.tail = (cscp_msg_queue.tail + 1) % CSCP_QUEUE_LEN;
	if (cscp_msg_queue.head == cscp_msg_queue.tail)
		cscp_msg_queue.full = 1;

	return 0;
}

int cscp_peek_msg_id(void)
{
	if(!cscp_msg_queue.full &&
		(cscp_msg_queue.head == cscp_msg_queue.tail))
	{
		return -1;
	}
	return cscp_msg_queue.msgs[cscp_msg_queue.head].id;
}

int cscp_dequeue(uint8_t *buf)
{
	if (!cscp_msg_queue.full &&
		(cscp_msg_queue.head == cscp_msg_queue.tail)) {
		return 1;
	}

	memcpy(buf,
		cscp_msg_queue.msgs[cscp_msg_queue.head].data,
		cscp_msg_queue.msgs[cscp_msg_queue.head].len);
	cscp_msg_queue.head = (cscp_msg_queue.head + 1) % CSCP_QUEUE_LEN;
	cscp_msg_queue.full = 0;

	return 0;
}

void cscp_callback_init(void)
{
	memset(cscp_callback_handles, 0, sizeof(cscp_callback_handles));
}

void cscp_register_callback(uint32_t msg_id, cscp_callback_t callback, uint8_t *data)
{
	cscp_callback_handles[msg_id].callback = callback;
	cscp_callback_handles[msg_id].data = data;
}
