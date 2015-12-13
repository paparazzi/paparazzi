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

enum BlueGigaStatus coms_status;
struct bluegiga_periph bluegiga_p;
struct spi_transaction bluegiga_spi;

signed char bluegiga_rssi[256];    // values initialized with 127
unsigned char telemetry_copy[20];

void bluegiga_load_tx(struct bluegiga_periph *p);
void bluegiga_transmit(struct bluegiga_periph *p, uint8_t data);
void bluegiga_receive(struct spi_transaction *trans);

// Functions for the generic link device device API
static int dev_check_free_space(struct bluegiga_periph *p, uint8_t len)
{
  // check if there is enough space for message
  // NB if BLUEGIGA_BUFFER_SIZE is smaller than 256 then an additional check is needed that len < BLUEGIGA_BUFFER_SIZE
  if (len - 1 <= ((p->tx_extract_idx - p->tx_insert_idx - 1 + BLUEGIGA_BUFFER_SIZE) % BLUEGIGA_BUFFER_SIZE)) {
    return TRUE;
  }

  return FALSE;
}
static void dev_put_byte(struct bluegiga_periph *p, uint8_t byte)
{
  bluegiga_transmit(p, byte);
}
static void dev_send_message(struct bluegiga_periph *p)
{
  p->end_of_msg = p->tx_insert_idx;
}
static int dev_char_available(struct bluegiga_periph *p)
{
  return bluegiga_ch_available(p);
}
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
bool_t bluegiga_ch_available(struct bluegiga_periph *p)
{
  return (p->rx_extract_idx != p->rx_insert_idx);
}

/* safe increment of circular buffer */
void bluegiga_increment_buf(uint8_t *buf_idx, uint8_t len)
{
  *buf_idx = (*buf_idx + len) % BLUEGIGA_BUFFER_SIZE;
}

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

uint32_t last_ts = 0;
static void send_bluegiga(struct transport_tx *trans, struct link_device *dev)
{
  uint32_t now_ts = get_sys_time_msec();

  if (now_ts > last_ts) {
    uint32_t rate = 1000 * bluegiga_p.bytes_recvd_since_last / (now_ts - last_ts);
    pprz_msg_send_BLUEGIGA(trans, dev, AC_ID, &rate, 20, telemetry_copy);

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
  bluegiga_spi.cdiv           = SPIDiv64;
  bluegiga_spi.after_cb       = (SPICallback) trans_cb;

  // Configure generic link device
  p->device.periph            = (void *)(p);
  p->device.check_free_space  = (check_free_space_t) dev_check_free_space;
  p->device.put_byte          = (put_byte_t) dev_put_byte;
  p->device.send_message      = (send_message_t) dev_send_message;
  p->device.char_available    = (char_available_t) dev_char_available;
  p->device.get_byte          = (get_byte_t) dev_get_byte;

  // initialize peripheral variables
  p->rx_insert_idx    = 0;
  p->rx_extract_idx   = 0;
  p->tx_insert_idx    = 0;
  p->tx_extract_idx   = 0;

  for (int i = 0; i < bluegiga_spi.input_length; i++) {
    p->work_rx[i] = 0;
  }
  for (int i = 0; i < bluegiga_spi.output_length; i++) {
    p->work_tx[i] = 0;
  }
  for (int i = 0; i < 255; i++) {
    bluegiga_rssi[i] = 127;
  }

  p->bytes_recvd_since_last = 0;
  p->end_of_msg = p->tx_insert_idx;

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
  if (dev_check_free_space(p, 1) && coms_status != BLUEGIGA_UNINIT) {
    p->tx_buf[p->tx_insert_idx] = data;
    bluegiga_increment_buf(&p->tx_insert_idx, 1);
  }
}

/* Load waiting data into tx peripheral buffer */
void bluegiga_load_tx(struct bluegiga_periph *p)
{
  // check data available in buffer to send
  uint8_t packet_len = ((p->end_of_msg - p->tx_extract_idx + BLUEGIGA_BUFFER_SIZE) % BLUEGIGA_BUFFER_SIZE);
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
    for (i = packet_len + 1; i < bluegiga_spi.output_length; i++) {
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
    }

    /*
     * 0xff communication lost with ground station
     * 0xfe RSSI value from broadcaster
     * 0xfd Change in broadcast mode
     * 0xfc Receive all recorded RSSI
     * <=20 Data package from ground station
     */

    uint8_t packet_len = 0;
    uint8_t read_offset = 0;
    switch (trans->input_buf[0]) {
      case 0xff:  // communication lost with ground station
#ifdef MODEM_LED
        LED_OFF(MODEM_LED);
#endif
        coms_status = BLUEGIGA_UNINIT;
        gpio_set(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);     // Reset interrupt pin
        break;
      case 0xfe:        // RSSI value from broadcaster
        bluegiga_rssi[trans->input_buf[1]] = trans->input_buf[2];
        packet_len = trans->input_buf[3];
        read_offset = 4;
        break;
      case 0xfd:  // Change in broadcast mode
        gpio_set(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);     // Reset interrupt pin

        // fetch scan status
        if (trans->input_buf[1] == 1) {
          coms_status = BLUEGIGA_BROADCASTING;
        } else {
          coms_status = BLUEGIGA_UNINIT;
        }
        break;
      case 0xfc:  // Receive all recorded RSSI
        for (uint8_t i = 0; i < trans->input_buf[1]; i++) {
          bluegiga_rssi[trans->input_buf[2] + i] = trans->input_buf[3 + i];
        }
        break;
      default:
        packet_len = trans->input_buf[0];                 // length of transmitted message
        read_offset = 1;
    }

    // handle incoming datalink message
    if (packet_len > 0 && packet_len <= trans->input_length) {
      // Handle received message
      for (uint8_t i = 0; i < packet_len; i++) {
        bluegiga_p.rx_buf[(bluegiga_p.rx_insert_idx + i) % BLUEGIGA_BUFFER_SIZE] = trans->input_buf[i + read_offset];
      }
      bluegiga_increment_buf(&bluegiga_p.rx_insert_idx, packet_len);
      bluegiga_p.bytes_recvd_since_last += packet_len;
      coms_status = BLUEGIGA_IDLE;

      for (uint8_t i = 0; i < trans->input_length; i++) {
        telemetry_copy[i] = trans->input_buf[i];
      }
    } else {
      coms_status = BLUEGIGA_IDLE;
    }

    // load next message to be sent into work buffer, needs to be loaded before calling spi_slave_register
    bluegiga_load_tx(&bluegiga_p);

    // register spi slave read for next transaction
    spi_slave_register(&(BLUEGIGA_SPI_DEV), &bluegiga_spi);
  }
}

/* command bluetooth to switch to active scan mode to get rssi values from neighbouring drones */
void bluegiga_scan(struct bluegiga_periph *p)
{

  memset(p->work_tx, 0, 20);
  p->work_tx[0] = 0xfd;   // change broadcast mode header

  coms_status = BLUEGIGA_SENDING;

  // trigger bluegiga to read direct command
  gpio_clear(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);     // set interrupt
}

/* Request list of all recorded RSSI */
void bluegiga_request_all_rssi(struct bluegiga_periph *p)
{

  memset(p->work_tx, 0, 20);
  p->work_tx[0] = 0xfc;

  coms_status = BLUEGIGA_SENDING;

  // trigger bluegiga to read direct command
  gpio_clear(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);     // set interrupt
}
