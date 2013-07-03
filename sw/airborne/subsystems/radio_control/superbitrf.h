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
 * @file subsystems/radio_control/superbitrf.h
 * DSM2 and DSMX implementation for the cyrf6936 2.4GHz radio chip trough SPI
 */

#ifndef RADIO_CONTROL_SUPERBITRF_H
#define RADIO_CONTROL_SUPERBITRF_H

#include "peripherals/cyrf6936.h"
#include "mcu_periph/gpio.h"

/* Theoretically you could have 14 channel over DSM2/DSMX */
#ifndef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL 14
#endif

/* The channel ordering is always the same for DSM2 and DSMX */
#define RADIO_THROTTLE   0
#define RADIO_ROLL       1
#define RADIO_PITCH      2
#define RADIO_YAW        3
#define RADIO_GEAR       4
#define RADIO_FLAP       5
#define RADIO_AUX1       5
#define RADIO_AUX2       6
#define RADIO_AUX3       7
#define RADIO_AUX4       8
#define RADIO_AUX5       9
#define RADIO_AUX6       10
#define RADIO_AUX7       11
#define RADIO_AUX8       12
#define RADIO_AUX9       13

/* Map the MODE default to the gear switch */
#ifndef RADIO_MODE
#define RADIO_MODE       RADIO_GEAR
#endif

/* The timings in microseconds */
#define SUPERBITRF_BIND_RECV_TIME       10000       /**< The time to wait for a bind packet on a channel in microseconds */
#define SUPERBITRF_SYNC_RECV_TIME       12000       /**< The time to wait for a sync packet on a channel in microseconds */
#define SUPERBITRF_RECV_TIME            24000       /**< The time to wait for a transfer packet on a channel in microseconds */

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

/* The superbitrf structure */
struct SuperbitRF {
  struct Cyrf6936 cyrf6936;                 /**< The cyrf chip used */
  volatile enum SuperbitRFStatus status;    /**< The status of the superbitRF */
  uint8_t state;                            /**< The states each status can be in */
  uint32_t timer;                           /**< The timer in usec */
  uint8_t timeouts;                         /**< The amount of timeouts */

  uint8_t channels[23];                     /**< The channels used for DSM2/DSMX */
  uint8_t channel_idx;                      /**< The current channel index */
  uint8_t channel;                          /**< The current channel number */
  uint32_t packet_count;                    /**< How many packets are received(also the invalid) */

  uint8_t bind_mfg_id[4];                   /**< The MFG id where the receiver is bound to */
  uint8_t num_channels;                     /**< The number of channels the transmitter has */
  uint8_t protocol;                         /**< The protocol the transmitter uses */
  volatile enum dsm_resolution resolution;  /**< The resolution that the transmitter has */
  uint16_t crc_seed;                        /**< The CRC seed that is calculated with the bind MFG id */
  uint8_t sop_col;                          /**< The sop code column number calculated with the bind MFG id */
  uint8_t data_col;                         /**< The data code column number calculated with the bind MFG id */

  bool_t frame_available;                   /**< When a frame is available */
  int16_t rc_values[14];                    /**< The rc values from the packet */
};

/* The superbitrf functions and structures */
extern struct SuperbitRF superbitrf;
void superbitrf_event(void);

/* Macro that normalize superbitrf rc_values to radio values */
#define NormalizeRcDl(_in, _out, _count) {                              \
  uint8_t i;                                                            \
  for(i = 0; i < _count; i++) {                                         \
    if(i == RADIO_THROTTLE) {                                           \
    	_out[i] = (_in[i] + MAX_PPRZ) / 2;                              \
    	Bound(_out[i], 0, MAX_PPRZ);                                    \
    } else {                                                            \
    	_out[i] = -_in[i];                                              \
    	Bound(_out[i], MIN_PPRZ, MAX_PPRZ);                             \
    }                                                                   \
  }                                                                     \
}

/* The radio control event handler */
#define RadioControlEvent(_received_frame_handler) {                \
  cyrf6936_event(&superbitrf.cyrf6936);                             \
  superbitrf_event();                                               \
  if(superbitrf.frame_available) {                                  \
	  radio_control.frame_cpt++;                                    \
      radio_control.time_since_last_frame = 0;                      \
      radio_control.radio_ok_cpt = 0;                               \
      radio_control.status = RC_OK;                                 \
      NormalizeRcDl(superbitrf.rc_values,radio_control.values);     \
      _received_frame_handler();                                    \
      superbitrf.frame_available = FALSE;                           \
  }                                                                 \
}

#endif /* RADIO_CONTROL_SUPERBITRF_H */
