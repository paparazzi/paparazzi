/*
 * Copyright (C) 2014 Kirk Scheper <kirkscheper@gmail.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING. If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file subsystems/datalink/bluegiga.c
 * Datalink implementation for the BlueGiga Bluetooth radio chip trough SPI
 */

#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/bluegiga.h"
#include "mcu_periph/gpio.h"
#include "mcu_periph/spi.h"

#ifdef MODEM_LED
#include "led.h"
#endif

#include "subsystems/abi.h"

// for memset
#include <string.h>

#ifndef BLUEGIGA_SPI_DEV
#error "bluegiga: must define a BLUEGIGA_SPI_DEV"
#endif

// Bluegiga: DRDY defaults to SuperbitRf DRDY
#ifndef BLUEGIGA_DRDY_GPIO
#define BLUEGIGA_DRDY_GPIO SUPERBITRF_DRDY_PORT
#endif

#ifndef BLUEGIGA_DRDY_GPIO_PIN
#define BLUEGIGA_DRDY_GPIO_PIN SUPERBITRF_DRDY_PIN
#endif

#define TxStrengthOfSender(x) (x[1])
#define RssiOfSender(x)       (x[2])
#define Pprz_StxOfMsg(x)      (x[3])
#define SenderIdOfBGMsg(x)    (x[5])

enum BlueGigaStatus coms_status;
struct bluegiga_periph bluegiga_p;
struct spi_transaction bluegiga_spi;

uint8_t broadcast_msg[20];

void bluegiga_load_tx(struct bluegiga_periph *p, struct spi_transaction *trans);
void bluegiga_transmit(struct bluegiga_periph *p, uint8_t data);
void bluegiga_receive(struct spi_transaction *trans);

// Functions for the generic link device device API
static int dev_check_free_space(struct bluegiga_periph *p, long *fd __attribute__((unused)), uint16_t len)
{
  // check if there is enough space for message
  // NB if BLUEGIGA_BUFFER_SIZE is smaller than 256 then an additional check is needed that len < BLUEGIGA_BUFFER_SIZE
  if (len - 1 <= ((p->tx_extract_idx - p->tx_insert_idx - 1 + BLUEGIGA_BUFFER_SIZE) % BLUEGIGA_BUFFER_SIZE)) {
    return true;
  }

  return false;
}
static void dev_put_buffer(struct bluegiga_periph *p, long fd __attribute__((unused)), uint8_t *data, uint16_t len)
{
  int i;
  for (i = 0; i < len; i++) {
    bluegiga_transmit(p, data[i]);
  }
}
static void dev_put_byte(struct bluegiga_periph *p, long fd __attribute__((unused)), uint8_t byte)
{
  bluegiga_transmit(p, byte);
}
static void dev_send_message(struct bluegiga_periph *p, long fd __attribute__((unused)))
{
  p->end_of_msg = p->tx_insert_idx;
}
static int dev_char_available(struct bluegiga_periph *p)
{
  return bluegiga_ch_available(p);
}

// note, need to run dev_char_available first
static uint8_t dev_get_byte(struct bluegiga_periph *p)
{
  uint8_t ret = p->rx_buf[p->rx_extract_idx];
  bluegiga_increment_buf(&p->rx_extract_idx, 1);
  return ret;
}

// Functions for the generic spi device API
static void trans_cb(struct spi_transaction *trans)
{
  bluegiga_receive(trans);
}

/* check if character available in receive buffer */
bool bluegiga_ch_available(struct bluegiga_periph *p)
{
  return (p->rx_extract_idx != p->rx_insert_idx);
}

/* safe increment of circular buffer */
void bluegiga_increment_buf(uint8_t *buf_idx, uint8_t len)
{
  *buf_idx = (*buf_idx + len) % BLUEGIGA_BUFFER_SIZE;
}

uint32_t a2a_msgs = 0;
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

uint32_t last_ts = 0;
static void send_bluegiga(struct transport_tx *trans, struct link_device *dev)
{
  uint32_t now_ts = get_sys_time_msec();

  if (now_ts > last_ts) {
    uint32_t rate = 1000 * bluegiga_p.bytes_recvd_since_last / (now_ts - last_ts);
    uint32_t a2a_rate = 1000 * a2a_msgs / (now_ts - last_ts);
    pprz_msg_send_BLUEGIGA(trans, dev, AC_ID, &rate, &a2a_rate);

    a2a_msgs = 0;
    bluegiga_p.bytes_recvd_since_last = 0;
    last_ts = now_ts;
  }
}
#endif

void bluegiga_init(struct bluegiga_periph *p)
{
#ifdef MODEM_LED
  LED_INIT(MODEM_LED);
#endif

  // configure the SPI bus
  bluegiga_spi.input_buf      = p->work_rx;
  bluegiga_spi.output_buf     = p->work_tx;
  bluegiga_spi.input_length   = 20;
  bluegiga_spi.output_length  = 20;
  bluegiga_spi.slave_idx      = 0; // Not used for SPI-Slave: always NSS pin
  bluegiga_spi.select         = SPISelectUnselect;
  bluegiga_spi.cpol           = SPICpolIdleHigh;
  bluegiga_spi.cpha           = SPICphaEdge2;
  bluegiga_spi.dss            = SPIDss8bit;
  bluegiga_spi.bitorder       = SPIMSBFirst;
  bluegiga_spi.cdiv           = SPIDiv256;
  bluegiga_spi.after_cb       = (SPICallback) trans_cb;

  // Configure generic link device
  p->device.periph            = (void *)(p);
  p->device.check_free_space  = (check_free_space_t) dev_check_free_space;
  p->device.put_byte          = (put_byte_t) dev_put_byte;
  p->device.put_buffer        = (put_buffer_t) dev_put_buffer;
  p->device.send_message      = (send_message_t) dev_send_message;
  p->device.char_available    = (char_available_t) dev_char_available;
  p->device.get_byte          = (get_byte_t) dev_get_byte;

  // initialize peripheral variables
  p->rx_insert_idx    = 0;
  p->rx_extract_idx   = 0;
  p->tx_insert_idx    = 0;
  p->tx_extract_idx   = 0;

  memset(p->work_rx, 0, bluegiga_spi.input_length);
  memset(p->work_tx, 0, bluegiga_spi.output_length);

  memset(broadcast_msg, 0, 19);

  p->bytes_recvd_since_last = 0;
  p->end_of_msg = p->tx_insert_idx;
  p->connected = 0;

  // set DRDY interrupt pin for spi master triggered on falling edge
  gpio_setup_output(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);
  gpio_set(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);

  coms_status = BLUEGIGA_UNINIT;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_BLUEGIGA, send_bluegiga);
#endif

  // register spi slave read for transaction
  spi_slave_register(&(BLUEGIGA_SPI_DEV), &bluegiga_spi);
}

/* Add one byte to the end of tx circular buffer */
void bluegiga_transmit(struct bluegiga_periph *p, uint8_t data)
{
  long fd = 0;
  if (dev_check_free_space(p, &fd, 1) && coms_status != BLUEGIGA_UNINIT) {
    p->tx_buf[p->tx_insert_idx] = data;
    bluegiga_increment_buf(&p->tx_insert_idx, 1);
  }
}

/* Load waiting data into tx peripheral buffer */
void bluegiga_load_tx(struct bluegiga_periph *p, struct spi_transaction *trans)
{
  uint8_t packet_len;
  // check data available in buffer to send
  packet_len = ((p->end_of_msg - p->tx_extract_idx + BLUEGIGA_BUFFER_SIZE) % BLUEGIGA_BUFFER_SIZE);
  if (packet_len > 19) {
    packet_len = 19;
  }

  if (packet_len && coms_status == BLUEGIGA_IDLE) {
    uint8_t i;
    // attach header with data length of real data in 20 char data string
    p->work_tx[0] = packet_len;

    // copy data from working buffer to spi output buffer
    for (i = 0; i < packet_len; i++) {
      p->work_tx[i + 1] = p->tx_buf[(p->tx_extract_idx + i) % BLUEGIGA_BUFFER_SIZE];
    }
    bluegiga_increment_buf(&p->tx_extract_idx, packet_len);

    // clear unused bytes
    for (i = packet_len + 1; i < trans->output_length; i++) {
      p->work_tx[i] = 0;
    }

    coms_status = BLUEGIGA_SENDING;
  }
}

/* read data from dma if available, set as call back of successful spi exchange
 *
 * TODO Remove use of bluegiga_p global in following function
 */
void bluegiga_receive(struct spi_transaction *trans)
{
  if (trans->status == SPITransSuccess) {
    // handle successful msg send
    if (coms_status == BLUEGIGA_SENDING) {
      // empty transfer buffer
      for (uint8_t i = 0; i < trans->output_length; i++) {
        trans->output_buf[i] = 0;
      }
    } else if (coms_status == BLUEGIGA_SENDING_BROADCAST) {
      // sending second half of broadcast message
      for (uint8_t i = 0; i < broadcast_msg[0]; i++) {
        trans->output_buf[i] = broadcast_msg[i];
      }
      coms_status = BLUEGIGA_SENDING;
      return;
    }

    /*
     * >235 data package from broadcast mode
     * 0x50 communication lost with ground station
     * 0x51 interrupt handled
     * <20  data package from connection
     */

    uint8_t packet_len = 0;
    uint8_t read_offset = 0;
    switch (trans->input_buf[0]) {
      case 0x50:  // communication status changed
        bluegiga_p.connected = trans->input_buf[1];
        if (bluegiga_p.connected) {
          //telemetry_mode_Main = TELEMETRY_PROCESS_Main;
        } else {
          //telemetry_mode_Main = NB_TELEMETRY_MODES;   // send no periodic telemetry
        }
        coms_status = BLUEGIGA_IDLE;
        break;
      case 0x51:  // Interrupt handled
        gpio_set(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);          // Reset interrupt pin
        break;
      default:
        coms_status = BLUEGIGA_IDLE;
        // compute length of transmitted message
        if (trans->input_buf[0] < trans->input_length) {               // normal connection mode
          packet_len = trans->input_buf[0];
          read_offset = 1;
        } else if (trans->input_buf[0] > 0xff - trans->input_length) { // broadcast mode
          packet_len = 0xff - trans->input_buf[0];

          if (packet_len > 3)
          {
#ifdef MODEM_LED
            LED_TOGGLE(MODEM_LED);
#endif

            int8_t tx_strength = TxStrengthOfSender(trans->input_buf);
            int8_t rssi = RssiOfSender(trans->input_buf);
            uint8_t ac_id = SenderIdOfBGMsg(trans->input_buf);

            if (Pprz_StxOfMsg(trans->input_buf) == PPRZ_STX) {
              AbiSendMsgRSSI(RSSI_BLUEGIGA_ID, ac_id, tx_strength, rssi);
            }
            a2a_msgs++;
          }

          read_offset = 3;
        }
    }

    // handle incoming datalink message
    if (packet_len > 0 && packet_len <= trans->input_length - read_offset) {
//#ifdef MODEM_LED
//      LED_TOGGLE(MODEM_LED);
//#endif
      // Handle received message
      for (uint8_t i = 0; i < packet_len; i++) {
        bluegiga_p.rx_buf[(bluegiga_p.rx_insert_idx + i) % BLUEGIGA_BUFFER_SIZE] = trans->input_buf[i + read_offset];
      }
      bluegiga_increment_buf(&bluegiga_p.rx_insert_idx, packet_len);
      bluegiga_p.bytes_recvd_since_last += packet_len;
    }

    // load next message to be sent into work buffer, needs to be loaded before calling spi_slave_register
    bluegiga_load_tx(&bluegiga_p, trans);

    // register spi slave read for next transaction
    spi_slave_register(&(BLUEGIGA_SPI_DEV), trans);
  }
}

/* Send data for broadcast message to the bluegiga module
 * maximum size of message is 22 bytes
 */
void bluegiga_broadcast_msg(struct bluegiga_periph *p, char *msg, uint8_t msg_len)
{
  if (msg_len == 0 || msg_len > 22) {
    return;
  }

  uint8_t max_length = 20;
  p->work_tx[0] = msg_len;

  if (msg_len < max_length) {
    for (uint8_t i = 0; i < msg_len; i++) {
      p->work_tx[i + 1] = msg[i];
    }
    coms_status = BLUEGIGA_SENDING;
  } else {
    for (uint8_t i = 0; i < max_length - 1; i++) {
      p->work_tx[i + 1] = msg[i];
    }

    memcpy(broadcast_msg, msg + max_length - 1, msg_len - (max_length - 1));
    coms_status = BLUEGIGA_SENDING_BROADCAST;
  }

  // trigger bluegiga to read direct command
  gpio_clear(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);     // set interrupt
}
