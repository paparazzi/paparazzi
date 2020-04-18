/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/datalink/bitcraze/syslink_dl.c
 *
 * Syslink protocol handling and functionalities
 *
 */

#include "modules/datalink/bitcraze/syslink_dl.h"
#include "subsystems/electrical.h"
#include "mcu_periph/uart.h"
#include <string.h>
#include "led.h"

struct syslink_dl syslink;

/** Protect syslink TX with Mutex when using RTOS */
#include "pprz_mutex.h"
PPRZ_MUTEX(syslink_tx_mtx);

/** Send a syslink message
 */
static void send_message(syslink_message_t *msg)
{
  syslink_compute_cksum(msg);
  uint8_t buf[sizeof(syslink_message_t)+2];
  buf[0] = syslink_stx[0];
  buf[1] = syslink_stx[1];
  buf[2] = msg->type;
  buf[3] = msg->length;
  for (int i = 0; i < msg->length; i++) {
    buf[4+i] = msg->data[i];
  }
  buf[msg->length+4] = msg->cksum[0];
  buf[msg->length+5] = msg->cksum[1];
  uart_put_buffer(&(SYSLINK_DEV), 0, buf, msg->length+6);
  //uart_put_buffer(&(SYSLINK_DEV), 0, (uint8_t*)syslink_stx, 2);
  //uart_put_buffer(&(SYSLINK_DEV), 0, (uint8_t*)(&msg->type), sizeof(msg->type));
  //uart_put_buffer(&(SYSLINK_DEV), 0, (uint8_t*)(&msg->length), sizeof(msg->length));
  //uart_put_buffer(&(SYSLINK_DEV), 0, (uint8_t*)(&msg->data), msg->length);
  //uart_put_buffer(&(SYSLINK_DEV), 0, (uint8_t*)(&msg->cksum), sizeof(msg->cksum));
}

/**
 * Handle battery message
 */
static void handle_battery(syslink_message_t *msg)
{
  if (msg->length != 9) {
    return;
  }

  // check flag
  uint8_t flags = msg->data[0];
  syslink.charging = (bool) (flags & 1);
  syslink.powered = (bool) (flags & 2);

  // update voltage
  float vbat;
  memcpy(&vbat, &msg->data[1], sizeof(float));
  if (syslink.powered && vbat > 3.f) {
    electrical.vsupply = vbat; // remove 0 reading when powered on USB ?
  } else if (!syslink.powered) {
    electrical.vsupply = vbat; // running on battery
  }
}

/**
 * Handle various raw messages
 */
static void handle_raw_other(syslink_message_t *msg)
{
	// This function doesn't actually do anything
	// It is just here to return null responses to most standard messages
  // Setup order from the cflib-python is
  //  - platform link source (name, protocol version)
  //  - log TOC
  //  - mem update
  //  - param TOC
  // -> call connected callback and start getting data
  //  - update param if needed

  crtp_message_t *c = (crtp_message_t *) &msg->length;

  if (c->port == CRTP_PORT_LOG) {
    if (c->channel == 0) { // Table of Contents Access
      uint8_t cmd = c->data[0];
      if (cmd == 0) { // GET_ITEM
        //int id = c->data[1];
        memset(&c->data[2], 0, 3);
        c->data[2] = 1; // type
        c->size = 1 + 5;
        send_message(msg);
      } else if (cmd == 1) { // GET_INFO
        memset(&c->data[1], 0, 7);
        c->size = 1 + 8;
        send_message(msg);
      }
    }
    else if (c->channel == 1) { // Log control
      c->data[2] = 0; // Success
      c->size = 3 + 1;
      // resend message
      send_message(msg);
    }
    else if (c->channel == 2) { // Log data
      // nothing
    }
  }
  else if (c->port == CRTP_PORT_MEM) {
    if (c->channel == 0) { // Info
      int cmd = c->data[0];
      if (cmd == 1) { // GET_NBR_OF_MEMS
        c->data[1] = 0;
        c->size = 2 + 1;
        // resend message
        send_message(msg);
      }
    }
  }
  else if (c->port == CRTP_PORT_PARAM) {
    if (c->channel == 0) { // TOC Access
      c->data[1] = 0; // Last parameter (id = 0)
      memset(&c->data[2], 0, 10);
      c->size = 1 + 8;
      send_message(msg);
    }
    else if (c->channel == 1) { // Param read
      // 0 is ok
      c->data[1] = 0; // value
      c->size = 1 + 2;
      send_message(msg);
    }
  }
  else if (c->port == CRTP_PORT_LINK) {
    if (c->channel == 0) { // Echo
      send_message(msg);
    }
    else if (c->channel == 1) { // Reply platform name
      c->size = CRTP_MAX_DATA_SIZE;
      bzero(c->data, CRTP_MAX_DATA_SIZE);
      strcpy((char *)c->data, "Bitcraze PPRZ");
      send_message(msg);
    }
  }
  else {
    // TODO handle error ?
  }
}

/**
 * Handle raw datalink
 *
 * From Bitcraze documentation:
 *
 * This packet carries the raw radio packet. The NRF51 acts as a radio bridge.
 * Because the NRF51 does not have much memory and the STM32 is capable of bursting
 * a lot of data a flow control rules has been made: The STM32 is allowed to send
 * a RADIO_RAW packet only when one RADIO_RAW packet has been received.
 *
 * The NRF51 is regularly sending CRTP NULL packet or empty packets to the STM32
 * to get the communication working both ways.
 *
 * Note: So far RADIO_RAW is the only syslink packet that has flow control constrain,
 * all other packets can be sent full duplex at any moment.
 */
static void handle_raw(syslink_message_t *msg)
{
  crtp_message_t *c = (crtp_message_t *) &msg->length;

  if (CRTP_NULL(*c)) {
    if (c->size >= 3) {
      //handle_bootloader(sys);
    }
  }
  else if (c->port == CRTP_PORT_COMMANDER) {
    //crtp_commander *cmd = (crtp_commander *) &c->data[0];
    // TODO set RC from cmd message
  }
  else if (c->port == CRTP_PORT_PPRZLINK) {
    // fill rx buffer with CRTP data
    uint8_t data_size = c->size;
    uint16_t available = SYSLINK_RX_BUF_LEN - syslink.device.char_available(&syslink);
    if (available > data_size) {
      // enough room to add new bytes
      if ((uint16_t)syslink.rx_insert_idx + (uint16_t)data_size < SYSLINK_RX_BUF_LEN) {
        // copy in one block
        memcpy(&syslink.rx_buf[syslink.rx_insert_idx], c->data, data_size);
      } else {
        // copy in two parts
        uint16_t split = SYSLINK_RX_BUF_LEN - syslink.rx_insert_idx;
        memcpy(&syslink.rx_buf[syslink.rx_insert_idx], c->data, split);
        memcpy(&syslink.rx_buf[0], &c->data[split], data_size - split);
      }
      syslink.rx_insert_idx = (syslink.rx_insert_idx + data_size) % SYSLINK_RX_BUF_LEN;
    }
  }
  else {
    handle_raw_other(msg);
  }

  // send next raw message if fifo is not empty
  if (syslink.tx_extract_idx != syslink.tx_insert_idx) {
    PPRZ_MUTEX_LOCK(syslink_tx_mtx);
    syslink_message_t msg_raw;
    msg_raw.type = SYSLINK_RADIO_RAW;
    memcpy(&msg_raw.length, &syslink.msg_tx[syslink.tx_extract_idx], sizeof(crtp_message_t));
    send_message(&msg_raw);
    // move fifo indexes
    syslink.tx_extract_idx++;
    if (syslink.tx_extract_idx == CRTP_BUF_LEN) {
      syslink.tx_extract_idx = 0;
    }
    PPRZ_MUTEX_UNLOCK(syslink_tx_mtx);
  }

}

static void handle_radio(syslink_message_t *msg)
{
  if (msg->type == SYSLINK_RADIO_RSSI) {
		uint8_t rssi = msg->data[0]; // Between 40 and 100 meaning -40 dBm to -100 dBm
    syslink.rssi = 140 - rssi * 100 / (100 - 40);
	}
  else if (msg->type == SYSLINK_RADIO_CHANNEL) {
    // ack radio channel
  }
  else if (msg->type == SYSLINK_RADIO_DATARATE) {
    // ack radio datarate
  }
  else if (msg->type == SYSLINK_RADIO_ADDRESS) {
    // ack radio address
  }
}


/** New RX message
 */
static void handle_new_msg(syslink_message_t *msg)
{
	if (msg->type == SYSLINK_PM_ONOFF_SWITCHOFF) {
		// power button is hit
    // don't do anything for now ?
	}
  else if (msg->type == SYSLINK_PM_BATTERY_STATE) {
    handle_battery(msg);
	}
  else if (msg->type == SYSLINK_RADIO_RAW) {
		handle_raw(msg);
	}
  else if ((msg->type & SYSLINK_GROUP) == SYSLINK_RADIO) {
		handle_radio(msg);
	}
  else if ((msg->type & SYSLINK_GROUP) == SYSLINK_OW) {
    // one-wire memory access
    // don't do anything for now
	}
  else {
    // handle errors ?
	}
}


/**
 * Implementation of syslink as generic device
 */

// check free space: nb of CRTP slots x space in a slot
static int syslink_check_free_space(struct syslink_dl *s, long *fd UNUSED, uint16_t len)
{
  int slots = s->tx_extract_idx - s->tx_insert_idx;
  if (slots <= 0) {
    slots += CRTP_BUF_LEN;
  }
  int space = CRTP_MAX_DATA_SIZE * (slots - 1);
  if (space >= len) {
    PPRZ_MUTEX_LOCK(syslink_tx_mtx);
    return space;
  }
  return 0;
}

// implementation of put_buffer, fill CRTP slots
static void syslink_put_buffer(struct syslink_dl *s, long fd UNUSED, const uint8_t *data, uint16_t len)
{
  uint16_t buf_rem = len;
  // fill slots until they are full
  // we assume that the available space have been check before
  while (buf_rem > 0) {
    // get current slot
    crtp_message_t *c = &s->msg_tx[s->tx_insert_idx];
    if (c->size < CRTP_MAX_DATA_SIZE) {
      // fill current buffer
      uint16_t data_size = Min(buf_rem, CRTP_MAX_DATA_SIZE - c->size);
      memcpy(&c->data[c->size - sizeof(c->header)], &data[len - buf_rem], data_size);
      c->size += data_size;
      buf_rem -= data_size;
    }
    else {
      // start a new slot
      uint8_t tmp = (s->tx_insert_idx + 1) % CRTP_BUF_LEN;
      if (tmp == s->tx_extract_idx) {
        // no more slots
        // this should be be possible when check_free_space is called before
        return;
      }
      crtp_message_t *c = &s->msg_tx[tmp];
      uint16_t data_size = Min(buf_rem, CRTP_MAX_DATA_SIZE);
      memcpy(c->data, &data[len - buf_rem], data_size);
      c->size = sizeof(c->header) + data_size;
      buf_rem -= data_size;
      s->tx_insert_idx = tmp;
    }
  }
}

// implementation of put_byte using put_buffer
static void syslink_put_byte(struct syslink_dl *s, long fd, const uint8_t b)
{
  syslink_put_buffer(s, fd, &b, 1);
}

// send_message is not needed as messages are stored in a fifo
static void syslink_send_message(struct syslink_dl *s UNUSED, long fd UNUSED)
{
  PPRZ_MUTEX_UNLOCK(syslink_tx_mtx); // release mutex
}

static uint8_t syslink_getch(struct syslink_dl *s)
{
  uint8_t ret = s->rx_buf[s->rx_extract_idx];
  s->rx_extract_idx = (s->rx_extract_idx + 1) % SYSLINK_RX_BUF_LEN;
  return ret;
}

static int syslink_char_available(struct syslink_dl *s)
{
  int available = s->rx_insert_idx - s->rx_extract_idx;
  if (available < 0) {
    available += SYSLINK_RX_BUF_LEN;
  }
  return available;
}


/** Init function */
void syslink_dl_init(void)
{
  syslink_parse_init(&syslink.state);
  syslink.tx_insert_idx = 0;
  syslink.tx_extract_idx = 0;
  syslink.rssi = 0;
  syslink.charging = false;
  syslink.powered = false;

  for (int i = 0; i < CRTP_BUF_LEN; i++) {
    // prepare raw pprzlink datalink headers
    syslink.msg_tx[i].header = 0;
    syslink.msg_tx[i].port = CRTP_PORT_PPRZLINK;
    syslink.msg_tx[i].channel = i % 4;
    syslink.msg_tx[i].size = sizeof(syslink.msg_tx[i].header);
  }

  // generic device
  syslink.device.periph = (void *)(&syslink);
  syslink.device.check_free_space = (check_free_space_t) syslink_check_free_space;
  syslink.device.put_byte = (put_byte_t) syslink_put_byte;
  syslink.device.put_buffer = (put_buffer_t) syslink_put_buffer;
  syslink.device.send_message = (send_message_t) syslink_send_message;
  syslink.device.char_available = (char_available_t) syslink_char_available;
  syslink.device.get_byte = (get_byte_t) syslink_getch;

  // init mutex if needed
  PPRZ_MUTEX_INIT(syslink_tx_mtx);
}

/** Periodic function */
void syslink_dl_periodic(void)
{
#ifdef CHARGING_LED
  if (syslink.charging) {
    LED_TOGGLE(CHARGING_LED);
  }
  else if (syslink.powered) {
    LED_ON(CHARGING_LED);
  }
  else {
    LED_OFF(CHARGING_LED);
  }
#endif
}

/** Datalink event */
void syslink_dl_event(void)
{
  while (uart_char_available(&(SYSLINK_DEV))) {
    uint8_t ch = uart_getch(&(SYSLINK_DEV));
    if (syslink_parse_char(&syslink.state, ch, &syslink.msg_rx)) {
      handle_new_msg(&syslink.msg_rx);
    }
  }
}


