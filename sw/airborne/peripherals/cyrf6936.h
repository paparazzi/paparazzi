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
 * @file peripherals/cyrf6936.h
 * Driver for the cyrf6936 2.4GHz radio chip
 */

#ifndef CYRF6936_H
#define CYRF6936_H

#include "cyrf6936_regs.h"
#include "mcu_periph/spi.h"

#define CYRF6936_MAX_BUFFER     80 /**< The max buffer size in the cyrf6936 structure */

/* The different statuses the cyrf6936 chip can be in */
enum Cyrf6936Status {
  CYRF6936_UNINIT,                  /**< The chip isn't initialized */
  CYRF6936_IDLE,                    /**< The chip is idle and can be used */
  CYRF6936_GET_MFG_ID,              /**< The chip is busy with getting the manufacturer ID */
  CYRF6936_MULTIWRITE,              /**< The chip is writing multiple registers */
  CYRF6936_DATA_CODE,               /**< The chip is writing a data code */
  CYRF6936_CHAN_SOP_DATA_CRC,       /**< The chip is setting the channel, SOP code, DATA code and the CRC seed */
  CYRF6936_RX_IRQ_STATUS_PACKET,    /**< The chip is getting the receive irq status, receive status and the receive packet */
  CYRF6936_SEND                     /**< The chip is busy sending a packet */
};

/* The structure for the cyrf6936 chip that handles all the buffers and requests */
struct Cyrf6936 {
  struct spi_periph *spi_p;                 /**< The SPI peripheral for the connection */
  struct spi_transaction spi_t;             /**< The SPI transaction used for the writing and reading of registers */
  volatile enum Cyrf6936Status status;      /**< The status of the CYRF6936 chip */
  uint8_t input_buf[17];                    /**< The input buffer for the SPI transaction */
  uint8_t output_buf[17];                   /**< The output buffer for the SPI transaction */

  uint8_t buffer[CYRF6936_MAX_BUFFER];      /**< The buffer used to write/read multiple registers */
  uint8_t buffer_length;                    /**< The length of the buffer used for MULTIWRITE */
  uint8_t buffer_idx;                       /**< The index of the buffer used for MULTIWRITE and used as sub-status for other statuses */

  bool_t has_irq;                           /**< When the CYRF6936 is done reading the irq */
  uint8_t mfg_id[6];                        /**< The manufacturer id of the CYRF6936 chip */
  uint8_t tx_irq_status;                    /**< The last send interrupt status */
  uint8_t rx_irq_status;                    /**< The last receive interrupt status */
  uint8_t rx_status;                        /**< The last receive status */
  uint8_t rx_count;                         /**< The length of the received packet */
  uint8_t rx_packet[16];                    /**< The last received packet */
};

extern void cyrf6936_init(struct Cyrf6936 *cyrf, struct spi_periph *spi_p, const uint8_t slave_idx,
                          const uint32_t rst_port, const uint16_t rst_pin);
void cyrf6936_event(struct Cyrf6936 *cyrf);

bool_t cyrf6936_write(struct Cyrf6936 *cyrf, const uint8_t addr, const uint8_t data);
bool_t cyrf6936_multi_write(struct Cyrf6936 *cyrf, const uint8_t data[][2], const uint8_t length);
bool_t cyrf6936_write_chan_sop_data_crc(struct Cyrf6936 *cyrf, const uint8_t chan, const uint8_t sop_code[],
                                        const uint8_t data_code[], const uint16_t crc_seed);
bool_t cyrf6936_read_rx_irq_status_packet(struct Cyrf6936 *cyrf);
bool_t cyrf6936_send(struct Cyrf6936 *cyrf, const uint8_t data[], const uint8_t length);

#endif /* CYRF6936_H */
