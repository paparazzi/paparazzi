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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING. If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/datalink/superbitrf.h
 * DSM2 and DSMX datalink implementation for the cyrf6936 2.4GHz radio chip trough SPI
 */

#ifndef DATALINK_SUPERBITRF_H
#define DATALINK_SUPERBITRF_H

#include "mcu_periph/gpio.h"
#include "peripherals/cyrf6936.h"
#include "pprzlink/pprzlink_device.h"
#include "pprzlink/pprz_transport.h"
#include "modules/datalink/datalink.h"

/* The timings in microseconds */
#define SUPERBITRF_BIND_RECV_TIME       10000       /**< The time to wait for a bind packet on a channel in microseconds */
#define SUPERBITRF_SYNC_RECV_TIME       7000        /**< The time to wait for a sync packet on a channel in microseconds */
#define SUPERBITRF_RECV_TIME            20000       /**< The time to wait for a transfer packet on a channel in microseconds */
#define SUPERBITRF_RECV_SHORT_TIME      6000        /**< The time to wait for a transfer packet short on a channel in microseconds */
#define SUPERBITRF_DATARECV_TIME        10000       /**< The time to wait for a data packet on a channel in microseconds */
#define SUPERBITRF_DATARECVB_TIME       6000        /**< The time to wait for a data packet on a channel during bind in microseconds */

/* The different statuses the superbitRF can be in */
enum SuperbitRFStatus {
  SUPERBITRF_UNINIT,                /**< The chip isn't initialized */
  SUPERBITRF_INIT_BINDING,          /**< The chip is initializing binding mode */
  SUPERBITRF_INIT_TRANSFER,         /**< The chip is initializing transfer mode */
  SUPERBITRF_BINDING,               /**< The chip is in binding mode */
  SUPERBITRF_SYNCING_A,             /**< The chip is in synchronizing mode for channel A */
  SUPERBITRF_SYNCING_B,             /**< The chip is in synchronizing mode for channel B */
  SUPERBITRF_TRANSFER,              /**< The chip is in transfer mode */
};

/* The different resolutions a transmitter can be in */
enum dsm_resolution {
  SUPERBITRF_10_BIT_RESOLUTION           = 0x00,     /**< The transmitter has a 10 bit resolution */
  SUPERBITRF_11_BIT_RESOLUTION           = 0x01,     /**< The transmitter has a 11 bit resolution */
};

/* The different protocols a transmitter can send */
enum dsm_protocol {
  DSM_DSM2_1          = 0x01,     /**< The original DSM2 protocol with 1 packet of data */
  DSM_DSM2_2          = 0x02,     /**< The original DSM2 protocol with 2 packets of data */
  DSM_DSM2P           = 0x10,     /**< Our own DSM2 Paparazzi protocol */
  DSM_DSMXP           = 0x11,     /**< Our own DSMX Paparazzi protocol */
  DSM_DSMX_1          = 0xA2,     /**< The original DSMX protocol with 1 packet of data */
  DSM_DSMX_2          = 0xB2,     /**< The original DSMX protocol with 2 packets of data */
};
#define IS_DSM2(x)          (x == DSM_DSM2P || x == DSM_DSM2_1 || x == DSM_DSM2_2)
#define IS_DSMX(x)          (!IS_DSM2(x))

#define SUPERBITRF_TX_BUFFER_SIZE 128

/* The superbitrf structure */
struct SuperbitRF {
  struct Cyrf6936 cyrf6936;                 /**< The cyrf chip used */
  volatile enum SuperbitRFStatus status;    /**< The status of the superbitRF */
  uint8_t state;                            /**< The states each status can be in */
  uint32_t timer;                           /**< The timer in microseconds */
  bool timer_overflow;                    /**< When the timer overflows */
  uint8_t timeouts;                         /**< The amount of timeouts */
  uint32_t transfer_timeouts;               /**< The amount of timeouts during transfer */
  uint32_t resync_count;                    /**< The amount of resyncs needed during transfer */
  uint8_t packet_loss_bit;                  /**< The packet loss indicating bit */
  bool packet_loss;                       /**< When we have packet loss last packet */

  uint8_t channels[23];                     /**< The channels used for DSM2/DSMX */
  uint8_t channel_idx;                      /**< The current channel index */
  uint8_t channel;                          /**< The current channel number */
  uint32_t irq_count;                       /**< How many interrupts are made */
  uint32_t rx_packet_count;                 /**< How many packets are received(also the invalid) */
  uint32_t tx_packet_count;                 /**< How many packets are send(also the invalid) */
  uint32_t uplink_count;                    /**< How many valid uplink packages are received */
  uint32_t rc_count;                        /**< How many valid RC packages are received */

  uint8_t bind_mfg_id[4];                   /**< The MFG id where the receiver is bound to */
  uint32_t bind_mfg_id32;                   /**< The MFG id where the receiver is bound to in uint32 */
  uint8_t num_channels;                     /**< The number of channels the transmitter has */
  volatile enum dsm_protocol protocol;      /**< The protocol the transmitter uses */
  volatile enum dsm_resolution resolution;  /**< The resolution that the transmitter has */
  uint16_t crc_seed;                        /**< The CRC seed that is calculated with the bind MFG id */
  uint8_t sop_col;                          /**< The sop code column number calculated with the bind MFG id */
  uint8_t data_col;                         /**< The data code column number calculated with the bind MFG id */

  bool rc_frame_available;                /**< When a RC frame is available */
  uint32_t timing1;                         /**< Time between last receive in microseconds */
  uint32_t timing2;                         /**< Time between second last receive in microseconds */
  int16_t rc_values[14];                    /**< The rc values from the packet */

  struct pprz_transport rx_transport;       /**< The receive transport */

  uint8_t tx_buffer[SUPERBITRF_TX_BUFFER_SIZE]; /**< The transmit buffer */
  uint8_t tx_insert_idx;                    /**< The transmit buffer insert index */
  uint8_t tx_extract_idx;                   /**< The transmit buffer extract index */

  /** Generic device interface */
  struct link_device device;
};

/* The superbitrf functions and structures */
extern struct SuperbitRF superbitrf;
extern void superbitrf_init(void);
extern void superbitrf_event(void);
extern void superbitrf_dl_init(void);
extern void superbitrf_dl_event(void);
extern void superbitrf_set_mfg_id(uint32_t id);
extern void superbitrf_set_protocol(uint8_t protocol);

/* The pprz transport structure */
extern struct pprz_transport pprz_srf_tp;

#endif /* DATALINK_SUPERBITRF_H */
