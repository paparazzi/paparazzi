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
#include "mcu_periph/spi.h"
#include "mcu_periph/gpio.h"

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

// Functions for the generic device API
static int dev_check_free_space(struct bluegiga_periph *p, uint8_t len)
{
  uint8_t i = p->tx_insert_idx;
  uint8_t o = p->tx_extract_idx;
  if (i < o) {
    i += BLUEGIGA_BUFFER_SIZE;
  }

  return (i - o) >= len;
}
static void dev_transmit(struct bluegiga_periph *p __attribute__((unused)), uint8_t byte)
{
  bluegiga_transmit(byte);
}
static void dev_send(struct bluegiga_periph *p __attribute__((unused)))
{
  bluegiga_send();
}
static void trans_cb(struct spi_transaction *trans __attribute__((unused)))
{
  bluegiga_receive();
}

void bluegiga_init(void)
{

#ifdef MODEM_LED
  LED_INIT(MODEM_LED);
#endif

  // configure the SPI bus.
  bluegiga_spi.input_buf      = bluegiga_p.work_rx;
  bluegiga_spi.output_buf     = bluegiga_p.work_tx;
  bluegiga_spi.input_length   = 20;
  bluegiga_spi.output_length  = 20;
  bluegiga_spi.slave_idx      = 0; // Not used for SPI-Slave: always NSS pin
  bluegiga_spi.select         = SPISelectUnselect;
  bluegiga_spi.cpol           = SPICpolIdleHigh;
  bluegiga_spi.cpha           = SPICphaEdge2;
  bluegiga_spi.dss            = SPIDss8bit;
  bluegiga_spi.bitorder       = SPIMSBFirst;
  bluegiga_spi.cdiv           = SPIDiv64;
  bluegiga_spi.after_cb       = (SPICallback)trans_cb;

  // initialize peripheral variables
  bluegiga_p.rx_insert_idx    = 0;
  bluegiga_p.rx_extract_idx   = 0;
  bluegiga_p.tx_insert_idx    = 0;
  bluegiga_p.tx_extract_idx   = 0;

  for (int i = 0; i < bluegiga_spi.input_length; i++) {
    bluegiga_p.work_rx[i] = 0;
  }
  for (int i = 0; i < bluegiga_spi.output_length; i++) {
    bluegiga_p.work_tx[i] = 0;
  }
  for (int i = 0; i < 255; i++) {
    bluegiga_rssi[i] = 127;
  }

  // Configure generic device
  bluegiga_p.device.periph    = (void *)(&bluegiga_p);
  bluegiga_p.device.check_free_space = (check_free_space_t) dev_check_free_space;
  bluegiga_p.device.put_byte  = (put_byte_t) dev_transmit;
  bluegiga_p.device.send_message = (send_message_t) dev_send;

  // set DRDY interrupt pin for spi master triggered on falling edge
  gpio_setup_output(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);
  gpio_set(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);

  // register spi slave read for transaction
  spi_slave_register(&(BLUEGIGA_SPI_DEV), &bluegiga_spi);

  coms_status = BLUEGIGA_UNINIT;
}

/* safe increment of circular buffer */
void bluegiga_increment_buf(uint8_t *buf_idx, uint8_t len)
{
  *buf_idx = (*buf_idx + len) % BLUEGIGA_BUFFER_SIZE;
}

/* Add one byte to the end of tx circular buffer */
void bluegiga_transmit(uint8_t data)
{
  if (BlueGigaCheckFreeSpace() && coms_status != BLUEGIGA_UNINIT) {
    bluegiga_p.tx_buf[bluegiga_p.tx_insert_idx] = data;
    bluegiga_increment_buf(&bluegiga_p.tx_insert_idx, 1);
  }
}

/* Send data in transmit buffer to spi master */
void bluegiga_send()
{
  uint8_t packet_len;

  // check data available in buffer to send
  packet_len = ((bluegiga_p.tx_insert_idx - bluegiga_p.tx_extract_idx + BLUEGIGA_BUFFER_SIZE) % BLUEGIGA_BUFFER_SIZE);
  if (packet_len > 18) {
    packet_len = 18;
  }

  if (packet_len && coms_status == BLUEGIGA_IDLE) {
    uint8_t i;
    // attach header with data length of real data in 20 char data string
    bluegiga_p.work_tx[1] = packet_len;

    // copy data from working buffer to spi output buffer
    for (i = 0; i < packet_len; i++) {
      bluegiga_p.work_tx[i + 2] = bluegiga_p.tx_buf[(bluegiga_p.tx_extract_idx + i) % BLUEGIGA_BUFFER_SIZE];
    }
    bluegiga_increment_buf(&bluegiga_p.tx_extract_idx, packet_len);

    // clear unused bytes
    for (i = packet_len + 2; i < bluegiga_spi.output_length; i++) {
      bluegiga_p.work_tx[i] = 0;
    }

    coms_status = BLUEGIGA_SENDING;
  }
}

/* read data from dma if available, set as call back of successful spi exchange */
void bluegiga_receive(void)
{
  if (bluegiga_spi.status == SPITransSuccess) {
    if (coms_status == BLUEGIGA_SENDING) {
      // Handle successful sent message
      for (uint8_t i = 0; i < bluegiga_spi.output_length; i++) { // Clear tx buffer
        bluegiga_p.work_tx[i] = 0;
      }
    }

    uint8_t packet_len = bluegiga_p.work_rx[3];                 // length of transmitted message

    if (packet_len > bluegiga_spi.input_length) {
      // Direct message from Bluegiga
      // int k_rssi, i;
      switch (packet_len) {
        case 0xff:        // Connection lost with ground station!
          // Stop datalink
          bluegiga_p.rx_insert_idx    = 0;
          bluegiga_p.rx_extract_idx   = 0;
          bluegiga_p.tx_insert_idx    = 0;
          bluegiga_p.tx_extract_idx   = 0;

#ifdef MODEM_LED
          LED_OFF(MODEM_LED);
#endif
          coms_status = BLUEGIGA_UNINIT;
          gpio_set(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);     // Reset interrupt pin
          break;
          /*case 0xfe:        // rssi data
            k_rssi = bluegiga_p.work_rx[2];
            for (i = 0; i < k_rssi; i++) {
              bluegiga_rssi[i] = bluegiga_p.work_rx[3 + i];
            }
            break;*/
        case 0xfd:        // interrupt handled on bluegiga
          gpio_set(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);     // Reset interrupt pin

          // fetch scan status
          if (bluegiga_p.work_rx[2] == 1) {
            coms_status = BLUEGIGA_SCANNING;
          } else {
            coms_status = BLUEGIGA_UNINIT;
          }
          break;
        default:
          break;
      }
    }

    // handle incoming datalink message
    else if (packet_len > 0) {
      bluegiga_rssi[bluegiga_p.work_rx[1]] = bluegiga_p.work_rx[2];
      // Handle received message
      for (uint8_t i = 0; i < packet_len; i++) {
        bluegiga_p.rx_buf[(bluegiga_p.rx_insert_idx + i) % BLUEGIGA_BUFFER_SIZE] = bluegiga_p.work_rx[i + 4];
      }
      bluegiga_increment_buf(&bluegiga_p.rx_insert_idx, packet_len);
      coms_status = BLUEGIGA_IDLE;
    } else {
      coms_status = BLUEGIGA_IDLE;
    }

    // clear rx buffer
    for (uint8_t i = 0; i < bluegiga_spi.input_length; i++) {
      bluegiga_p.work_rx[i] = 0;
    }
    // register spi slave read for next transaction
    spi_slave_register(&(BLUEGIGA_SPI_DEV), &bluegiga_spi);
  }
}

/* command bluetooth to switch to active scan mode to get rssi values from neighboring drones */
void bluegiga_scan(void)
{
  gpio_clear(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);     // set interrupt
}
