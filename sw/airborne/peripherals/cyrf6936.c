/*
 * Copyright (C) 2013 Freek van Tienen <freek.v.tienen@gmail.com>
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
 */

/**
 * @file peripherals/cyrf6936.c
 * Driver for the cyrf6936 2.4GHz radio chip
 */

#include "cyrf6936.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/gpio.h"
#include "mcu_periph/sys_time.h"
#include "modules/radio_control/radio_control.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

/* Static functions used in the different statuses */
static bool cyrf6936_write_register(struct Cyrf6936 *cyrf, const uint8_t addr, const uint8_t data);
static bool cyrf6936_write_block(struct Cyrf6936 *cyrf, const uint8_t addr, const uint8_t data[],
                                   const uint8_t length);
static bool cyrf6936_read_register(struct Cyrf6936 *cyrf, const uint8_t addr);
static bool cyrf6936_read_block(struct Cyrf6936 *cyrf, const uint8_t addr, const uint8_t length);

/**
 * Initializing the cyrf chip
 */
void cyrf6936_init(struct Cyrf6936 *cyrf, struct spi_periph *spi_p, const uint8_t slave_idx, const uint32_t rst_port,
                   const uint16_t rst_pin)
{
  /* Set spi_peripheral and the status */
  cyrf->spi_p = spi_p;
  cyrf->status = CYRF6936_UNINIT;
  cyrf->has_irq = false;

  /* Set the spi transaction */
  cyrf->spi_t.cpol = SPICpolIdleLow;
  cyrf->spi_t.cpha = SPICphaEdge1;
  cyrf->spi_t.dss = SPIDss8bit;
  cyrf->spi_t.bitorder = SPIMSBFirst;
  cyrf->spi_t.cdiv = SPIDiv64;

  cyrf->spi_t.input_length = 0;
  cyrf->spi_t.output_length = 0;
  cyrf->spi_t.input_buf = cyrf->input_buf;
  cyrf->spi_t.output_buf = cyrf->output_buf;
  cyrf->spi_t.slave_idx = slave_idx;
  cyrf->spi_t.select = SPISelectUnselect;
  cyrf->spi_t.status = SPITransDone;

  /* Reset the CYRF6936 chip (busy waiting) */
  gpio_setup_output(rst_port, rst_pin);
  gpio_set(rst_port, rst_pin);
  sys_time_usleep(100);
  gpio_clear(rst_port, rst_pin);
  sys_time_usleep(100);

  /* Get the MFG ID */
  cyrf->status = CYRF6936_GET_MFG_ID;
  cyrf->buffer_idx = 0;
  cyrf6936_write_register(cyrf, CYRF_MFG_ID, 0xFF);
}

/**
 * The on event call for the CYRF6936 chip
 */
void cyrf6936_event(struct Cyrf6936 *cyrf)
{
  int i;
  // Check if cyrf is initialized
  if (cyrf->status == CYRF6936_UNINIT) {
    return;
  }

  // Check if there is still a transaction in progress
  if (cyrf->spi_t.status == SPITransPending || cyrf->spi_t.status == SPITransRunning) {
    return;
  }

  /* Check the status of the cyrf */
  switch (cyrf->status) {

      /* Getting the MFG id */
    case CYRF6936_GET_MFG_ID:
      // When the last transaction isn't failed send the next
      if (cyrf->spi_t.status != SPITransFailed) {
        cyrf->buffer_idx++;
      }

      cyrf->spi_t.status = SPITransDone;

      // Switch for the different states
      switch (cyrf->buffer_idx) {
        case 0:
          cyrf6936_write_register(cyrf, CYRF_MFG_ID, 0xFF);
          break;
        case 1:
          cyrf6936_read_block(cyrf, CYRF_MFG_ID, 6);
          break;
        case 2:
          // Copy the MFG id
          for (i = 0; i < 6; i++) {
            cyrf->mfg_id[i] = cyrf->input_buf[i + 1];
          }

          cyrf6936_write_register(cyrf, CYRF_MFG_ID, 0x00);
          break;
        default:
          cyrf->status = CYRF6936_IDLE;
          break;
      }
      break;

      /* Do a multi write */
    case CYRF6936_MULTIWRITE:
      // When the last transaction isn't failed send the next
      if (cyrf->spi_t.status != SPITransFailed) {
        cyrf->buffer_idx++;
      }

      cyrf->spi_t.status = SPITransDone;

      // When we are done writing
      if (cyrf->buffer_idx == cyrf->buffer_length) {
        cyrf->status = CYRF6936_IDLE;
        break;
      }

      // Write the next register from the buffer
      cyrf6936_write_register(cyrf,
                              ((uint8_t ( *)[2])cyrf->buffer)[cyrf->buffer_idx][0],
                              ((uint8_t ( *)[2])cyrf->buffer)[cyrf->buffer_idx][1]);
      break;

      /* Do a write of the data code */
    case CYRF6936_DATA_CODE:
      break;

      /* Do a write of channel, sop, data and crc */
    case CYRF6936_CHAN_SOP_DATA_CRC:
      // When the last transaction isn't failed send the next
      if (cyrf->spi_t.status != SPITransFailed) {
        cyrf->buffer_idx++;
      }

      cyrf->spi_t.status = SPITransDone;

      // Switch for the different states
      switch (cyrf->buffer_idx) {
        case 0: // Write the CRC LSB
          cyrf6936_write_register(cyrf, CYRF_CRC_SEED_LSB, cyrf->buffer[0]);
          break;
        case 1: // Write the CRC MSB
          cyrf6936_write_register(cyrf, CYRF_CRC_SEED_MSB, cyrf->buffer[1]);
          break;
        case 2: // Write the SOP code
          cyrf6936_write_block(cyrf, CYRF_SOP_CODE, &(cyrf->buffer[2]), 8);
          break;
        case 3: // Write the DATA code
          cyrf6936_write_block(cyrf, CYRF_DATA_CODE, &(cyrf->buffer[10]), 16);
          break;
        case 4: // Write the Channel
          cyrf6936_write_register(cyrf, CYRF_CHANNEL, cyrf->buffer[26]);
          break;
        default:
          cyrf->status = CYRF6936_IDLE;
          break;
      }
      break;

      /* Do a read of the receive irq status, receive status and the receive packet */
    case CYRF6936_RX_IRQ_STATUS_PACKET:
      // When the last transaction isn't failed send the next
      if (cyrf->spi_t.status != SPITransFailed) {
        cyrf->buffer_idx++;
      }

      cyrf->spi_t.status = SPITransDone;

      // Switch for the different states
      switch (cyrf->buffer_idx) {
        case 0: // Read the receive IRQ status
          cyrf6936_read_register(cyrf, CYRF_RX_IRQ_STATUS);
          break;
        case 1: // Read the send IRQ status
          cyrf->rx_irq_status = cyrf->input_buf[1];
          cyrf6936_read_register(cyrf, CYRF_TX_IRQ_STATUS);
          break;
        case 2: // Read the receive status
          cyrf->tx_irq_status = cyrf->input_buf[1];
          cyrf6936_read_register(cyrf, CYRF_RX_STATUS);
          break;
        case 3: // Set the packet length
          cyrf->rx_status = cyrf->input_buf[1];
          cyrf6936_read_register(cyrf, CYRF_RX_COUNT);
          break;
        case 4: // Read the receive packet
          cyrf->rx_count = cyrf->input_buf[1];
          cyrf6936_read_block(cyrf, CYRF_RX_BUFFER, 16);
          break;
        default:
          // Copy the receive packet
          for (i = 0; i < 16; i++) {
            cyrf->rx_packet[i] = cyrf->input_buf[i + 1];
          }

          cyrf->has_irq = true;
          cyrf->status = CYRF6936_IDLE;
          break;
      }
      break;

      /* The CYRF6936 is busy sending a packet */
    case CYRF6936_SEND:
      // When the last transaction isn't failed send the next
      if (cyrf->spi_t.status != SPITransFailed) {
        cyrf->buffer_idx++;
      }

      cyrf->spi_t.status = SPITransDone;

      // Switch for the different states
      switch (cyrf->buffer_idx) {
        case 0: // Set the packet length
          cyrf6936_write_register(cyrf, CYRF_TX_LENGTH, cyrf->buffer[0]);
          break;
        case 1: // Clear the TX buffer
          cyrf6936_write_register(cyrf, CYRF_TX_CTRL, CYRF_TX_CLR);
          break;
        case 2: // Write the send packet
          cyrf6936_write_block(cyrf, CYRF_TX_BUFFER, &cyrf->buffer[1], 16);
          break;
        case 3: // Send the packet
          cyrf6936_write_register(cyrf, CYRF_TX_CTRL, CYRF_TX_GO | CYRF_TXC_IRQEN | CYRF_TXE_IRQEN);
          break;
        default:
          cyrf->status = CYRF6936_IDLE;
          break;
      }
      break;

      /* This should not happen */
    default:
      break;
  }
}

/**
 * Write a byte to a register
 */
static bool cyrf6936_write_register(struct Cyrf6936 *cyrf, const uint8_t addr, const uint8_t data)
{
  return cyrf6936_write_block(cyrf, addr, &data, 1);
}

/**
 * Write multiple bytes to a register
 */
static bool cyrf6936_write_block(struct Cyrf6936 *cyrf, const uint8_t addr, const uint8_t data[],
                                   const uint8_t length)
{
  uint8_t i;
  /* Check if there is already a SPI transaction busy */
  if (cyrf->spi_t.status != SPITransDone) {
    return false;
  }

  /* Set the buffer and commit the transaction */
  cyrf->spi_t.output_length = length + 1;
  cyrf->spi_t.input_length = 0;
  cyrf->output_buf[0] = addr | CYRF_DIR;

  // Copy the data
  for (i = 0; i < length; i++) {
    cyrf->output_buf[i + 1] = data[i];
  }

  // Submit the transaction
  return spi_submit(cyrf->spi_p, &(cyrf->spi_t));
}

/**
 * Read a byte from a register
 */
static bool cyrf6936_read_register(struct Cyrf6936 *cyrf, const uint8_t addr)
{
  return cyrf6936_read_block(cyrf, addr, 1);
}

/**
 * Read multiple bytes from a register
 */
static bool cyrf6936_read_block(struct Cyrf6936 *cyrf, const uint8_t addr, const uint8_t length)
{
  if (cyrf->spi_t.status != SPITransDone) {
    return false;
  }

  /* Set the buffer and commit the transaction */
  cyrf->spi_t.output_length = 1;
  cyrf->spi_t.input_length = length + 1;
  cyrf->output_buf[0] = addr;

  // Submit the transaction
  return spi_submit(cyrf->spi_p, &(cyrf->spi_t));
}

/**
 * Write to one register
 */
bool cyrf6936_write(struct Cyrf6936 *cyrf, const uint8_t addr, const uint8_t data)
{
  const uint8_t data_multi[][2] = {
    {addr, data}
  };
  return cyrf6936_multi_write(cyrf, data_multi, 1);
}

/**
 * Write to multiple registers one byte
 */
bool cyrf6936_multi_write(struct Cyrf6936 *cyrf, const uint8_t data[][2], const uint8_t length)
{
  uint8_t i;
  /* Check if the cyrf6936 isn't busy */
  if (cyrf->status != CYRF6936_IDLE) {
    return false;
  }

  // Set the status
  cyrf->status = CYRF6936_MULTIWRITE;

  /* Set the multi write */
  cyrf->buffer_length = length;
  cyrf->buffer_idx = 0;

  // Copy the buffer
  for (i = 0; i < length; i++) {
    cyrf->buffer[i * 2] = data[i][0];
    cyrf->buffer[i * 2 + 1] = data[i][1];
  }

  /* Write the first regiter */
  if (length > 0) {
    cyrf6936_write_register(cyrf, data[0][0], data[0][1]);
  }

  return true;
}

/**
 * Set the channel, SOP code, DATA code and the CRC seed
 */
bool cyrf6936_write_chan_sop_data_crc(struct Cyrf6936 *cyrf, const uint8_t chan, const uint8_t sop_code[],
                                        const uint8_t data_code[], const uint16_t crc_seed)
{
  uint8_t i;
  /* Check if the cyrf6936 isn't busy */
  if (cyrf->status != CYRF6936_IDLE) {
    return false;
  }

  // Set the status
  cyrf->status = CYRF6936_CHAN_SOP_DATA_CRC;

  // Copy the CRC
  cyrf->buffer[0] = crc_seed & 0xFF;
  cyrf->buffer[1] = (crc_seed >> 8) & 0xFF;

  // Copy the SOP code
  for (i = 0; i < 8; i++) {
    cyrf->buffer[i + 2] = sop_code[i];
  }

  // Copy the DATA code
  for (i = 0; i < 16; i++) {
    cyrf->buffer[i + 10] = data_code[i];
  }

  // Copy the channel
  cyrf->buffer[26] = chan;

  /* Try to write the CRC LSB */
  cyrf->buffer_idx = 0;
  cyrf6936_write_register(cyrf, CYRF_CRC_SEED_LSB, cyrf->buffer[0]);

  return true;
}

/**
 * Read the RX IRQ status register, the rx status register and the rx packet
 */
bool cyrf6936_read_rx_irq_status_packet(struct Cyrf6936 *cyrf)
{
  /* Check if the cyrf6936 isn't busy */
  if (cyrf->status != CYRF6936_IDLE) {
    return false;
  }

  // Set the status
  cyrf->status = CYRF6936_RX_IRQ_STATUS_PACKET;

  /* Try to read the RX status */
  cyrf->buffer_idx = 0;
  cyrf6936_read_register(cyrf, CYRF_RX_IRQ_STATUS);

  return true;
}

/**
 * Send a packet with a certain length
 */
bool cyrf6936_send(struct Cyrf6936 *cyrf, const uint8_t data[], const uint8_t length)
{
  int i;

  /* Check if the cyrf6936 isn't busy */
  if (cyrf->status != CYRF6936_IDLE) {
    return false;
  }

  // Set the status
  cyrf->status = CYRF6936_SEND;

  // Copy the length and the data
  cyrf->buffer[0] = length;
  for (i = 0; i < length; i++) {
    cyrf->buffer[i + 1] = data[i];
  }

  /* Try to set the packet length */
  cyrf->buffer_idx = 0;
  cyrf6936_write_register(cyrf, CYRF_TX_LENGTH, cyrf->buffer[0]);

  return true;
}
