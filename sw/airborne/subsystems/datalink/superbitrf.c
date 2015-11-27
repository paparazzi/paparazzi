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
 * @file subsystems/datalink/superbitrf.c
 * DSM2 and DSMX datalink implementation for the cyrf6936 2.4GHz radio chip trough SPI
 */

#include "subsystems/datalink/superbitrf.h"

#include <string.h>
#include "paparazzi.h"
#include "led.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/gpio.h"
#include "subsystems/settings.h"

/* Default SuperbitRF SPI DEV */
#ifndef SUPERBITRF_SPI_DEV
#define SUPERBITRF_SPI_DEV      spi1
#endif
PRINT_CONFIG_VAR(SUPERBITRF_SPI_DEV)

/* Default SuperbitRF RST PORT and PIN */
#ifndef SUPERBITRF_RST_PORT
#define SUPERBITRF_RST_PORT     GPIOC
#endif
PRINT_CONFIG_VAR(SUPERBITRF_RST_PORT)
#ifndef SUPERBITRF_RST_PIN
#define SUPERBITRF_RST_PIN      GPIO12
#endif
PRINT_CONFIG_VAR(SUPERBITRF_RST_PIN)

/* Default SuperbitRF DRDY(IRQ) PORT and PIN */
#ifndef SUPERBITRF_DRDY_PORT
#define SUPERBITRF_DRDY_PORT     GPIOB
#endif
PRINT_CONFIG_VAR(SUPERBITRF_DRDY_PORT)
#ifndef SUPERBITRF_DRDY_PIN
#define SUPERBITRF_DRDY_PIN      GPIO1
#endif
PRINT_CONFIG_VAR(SUPERBITRF_DRDY_PIN)

/* Default forcing in DSM2 mode is false */
#ifndef SUPERBITRF_FORCE_DSM2
#define SUPERBITRF_FORCE_DSM2   TRUE
#endif
PRINT_CONFIG_VAR(SUPERBITRF_FORCE_DSM2)

/* The superbitRF structure */
struct SuperbitRF superbitrf;

/* The internal functions */
static inline void superbitrf_radio_to_channels(uint8_t *data, uint8_t nb_channels, bool_t is_11bit, int16_t *channels);
static inline void superbitrf_receive_packet_cb(bool_t error, uint8_t status, uint8_t packet[]);
static inline void superbitrf_send_packet_cb(bool_t error);
static inline void superbitrf_gen_dsmx_channels(void);

/* The startup configuration for the cyrf6936 */
static const uint8_t cyrf_stratup_config[][2] = {
  {CYRF_MODE_OVERRIDE, CYRF_RST},                                         // Reset the device
  {CYRF_CLK_EN, CYRF_RXF},                                                // Enable the clock
  {CYRF_AUTO_CAL_TIME, 0x3C},                                             // From manual, needed for initialization
  {CYRF_AUTO_CAL_OFFSET, 0x14},                                           // From manual, needed for initialization
  {CYRF_RX_CFG, CYRF_LNA | CYRF_FAST_TURN_EN},                            // Enable low noise amplifier and fast turning
  {CYRF_TX_OFFSET_LSB, 0x55},                                             // From manual, typical configuration
  {CYRF_TX_OFFSET_MSB, 0x05},                                             // From manual, typical configuration
  {CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END},                     // Force in Synth RX mode
  {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_4},  // Enable 64 chip codes, SDR mode and amplifier +4dBm
  {CYRF_DATA64_THOLD, 0x0E},                                              // From manual, typical configuration
  {CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX},                                    // Set in Synth RX mode (again, really needed?)
};
/* The binding configuration for the cyrf6936 */
static const uint8_t cyrf_bind_config[][2] = {
  {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_4},  // Enable 64 chip codes, SDR mode and amplifier +4dBm
  {CYRF_FRAMING_CFG, CYRF_SOP_LEN | 0xE},                                 // Set SOP CODE to 64 chips and SOP Correlator Threshold to 0xE
  {CYRF_RX_OVERRIDE, CYRF_FRC_RXDR | CYRF_DIS_RXCRC},                     // Force receive data rate and disable receive CRC checker
  {CYRF_EOP_CTRL, 0x02},                                                  // Only enable EOP symbol count of 2
  {CYRF_TX_OVERRIDE, CYRF_DIS_TXCRC},                                     // Disable transmit CRC generate
};
/* The transfer configuration for the cyrf6936 */
static const uint8_t cyrf_transfer_config[][2] = {
  {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_8DR | CYRF_PA_4},  // Enable 64 chip codes, 8DR mode and amplifier +4dBm
  {CYRF_FRAMING_CFG, CYRF_SOP_EN | CYRF_SOP_LEN | CYRF_LEN_EN | 0xE},     // Set SOP CODE enable, SOP CODE to 64 chips, Packet length enable, and SOP Correlator Threshold to 0xE
  {CYRF_TX_OVERRIDE, 0x00},                                               // Reset TX overrides
  {CYRF_RX_OVERRIDE, 0x00},                                               // Reset RX overrides
};
/* Abort the receive of the cyrf6936 */
const uint8_t cyrf_abort_receive[][2] = {
  {CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END},
  {CYRF_RX_ABORT, 0x00}
};
/* Start the receive of the cyrf6936 */
const uint8_t cyrf_start_receive[][2] = {
  {CYRF_RX_IRQ_STATUS, CYRF_RXOW_IRQ}, // Clear the IRQ
  {CYRF_RX_CTRL, CYRF_RX_GO | CYRF_RXC_IRQEN | CYRF_RXE_IRQEN} // Start receiving and set the IRQ
};

/* The PN codes used for DSM2 and DSMX channel hopping */
static const uint8_t pn_codes[5][9][8] = {
  { /* Row 0 */
    /* Col 0 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8},
    /* Col 1 */ {0x88, 0x17, 0x13, 0x3B, 0x2D, 0xBF, 0x06, 0xD6},
    /* Col 2 */ {0xF1, 0x94, 0x30, 0x21, 0xA1, 0x1C, 0x88, 0xA9},
    /* Col 3 */ {0xD0, 0xD2, 0x8E, 0xBC, 0x82, 0x2F, 0xE3, 0xB4},
    /* Col 4 */ {0x8C, 0xFA, 0x47, 0x9B, 0x83, 0xA5, 0x66, 0xD0},
    /* Col 5 */ {0x07, 0xBD, 0x9F, 0x26, 0xC8, 0x31, 0x0F, 0xB8},
    /* Col 6 */ {0xEF, 0x03, 0x95, 0x89, 0xB4, 0x71, 0x61, 0x9D},
    /* Col 7 */ {0x40, 0xBA, 0x97, 0xD5, 0x86, 0x4F, 0xCC, 0xD1},
    /* Col 8 */ {0xD7, 0xA1, 0x54, 0xB1, 0x5E, 0x89, 0xAE, 0x86}
  },
  { /* Row 1 */
    /* Col 0 */ {0x83, 0xF7, 0xA8, 0x2D, 0x7A, 0x44, 0x64, 0xD3},
    /* Col 1 */ {0x3F, 0x2C, 0x4E, 0xAA, 0x71, 0x48, 0x7A, 0xC9},
    /* Col 2 */ {0x17, 0xFF, 0x9E, 0x21, 0x36, 0x90, 0xC7, 0x82},
    /* Col 3 */ {0xBC, 0x5D, 0x9A, 0x5B, 0xEE, 0x7F, 0x42, 0xEB},
    /* Col 4 */ {0x24, 0xF5, 0xDD, 0xF8, 0x7A, 0x77, 0x74, 0xE7},
    /* Col 5 */ {0x3D, 0x70, 0x7C, 0x94, 0xDC, 0x84, 0xAD, 0x95},
    /* Col 6 */ {0x1E, 0x6A, 0xF0, 0x37, 0x52, 0x7B, 0x11, 0xD4},
    /* Col 7 */ {0x62, 0xF5, 0x2B, 0xAA, 0xFC, 0x33, 0xBF, 0xAF},
    /* Col 8 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97}
  },
  { /* Row 2 */
    /* Col 0 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97},
    /* Col 1 */ {0x8E, 0x4A, 0xD0, 0xA9, 0xA7, 0xFF, 0x20, 0xCA},
    /* Col 2 */ {0x4C, 0x97, 0x9D, 0xBF, 0xB8, 0x3D, 0xB5, 0xBE},
    /* Col 3 */ {0x0C, 0x5D, 0x24, 0x30, 0x9F, 0xCA, 0x6D, 0xBD},
    /* Col 4 */ {0x50, 0x14, 0x33, 0xDE, 0xF1, 0x78, 0x95, 0xAD},
    /* Col 5 */ {0x0C, 0x3C, 0xFA, 0xF9, 0xF0, 0xF2, 0x10, 0xC9},
    /* Col 6 */ {0xF4, 0xDA, 0x06, 0xDB, 0xBF, 0x4E, 0x6F, 0xB3},
    /* Col 7 */ {0x9E, 0x08, 0xD1, 0xAE, 0x59, 0x5E, 0xE8, 0xF0},
    /* Col 8 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E}
  },
  { /* Row 3 */
    /* Col 0 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E},
    /* Col 1 */ {0x80, 0x69, 0x26, 0x80, 0x08, 0xF8, 0x49, 0xE7},
    /* Col 2 */ {0x7D, 0x2D, 0x49, 0x54, 0xD0, 0x80, 0x40, 0xC1},
    /* Col 3 */ {0xB6, 0xF2, 0xE6, 0x1B, 0x80, 0x5A, 0x36, 0xB4},
    /* Col 4 */ {0x42, 0xAE, 0x9C, 0x1C, 0xDA, 0x67, 0x05, 0xF6},
    /* Col 5 */ {0x9B, 0x75, 0xF7, 0xE0, 0x14, 0x8D, 0xB5, 0x80},
    /* Col 6 */ {0xBF, 0x54, 0x98, 0xB9, 0xB7, 0x30, 0x5A, 0x88},
    /* Col 7 */ {0x35, 0xD1, 0xFC, 0x97, 0x23, 0xD4, 0xC9, 0x88},
    /* Col 8 */ {0x88, 0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40}
  },
  { /* Row 4 */
    /* Col 0 */ {0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93},
    /* Col 1 */ {0xDC, 0x68, 0x08, 0x99, 0x97, 0xAE, 0xAF, 0x8C},
    /* Col 2 */ {0xC3, 0x0E, 0x01, 0x16, 0x0E, 0x32, 0x06, 0xBA},
    /* Col 3 */ {0xE0, 0x83, 0x01, 0xFA, 0xAB, 0x3E, 0x8F, 0xAC},
    /* Col 4 */ {0x5C, 0xD5, 0x9C, 0xB8, 0x46, 0x9C, 0x7D, 0x84},
    /* Col 5 */ {0xF1, 0xC6, 0xFE, 0x5C, 0x9D, 0xA5, 0x4F, 0xB7},
    /* Col 6 */ {0x58, 0xB5, 0xB3, 0xDD, 0x0E, 0x28, 0xF1, 0xB0},
    /* Col 7 */ {0x5F, 0x30, 0x3B, 0x56, 0x96, 0x45, 0xF4, 0xA1},
    /* Col 8 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8}
  },
};
static const uint8_t pn_bind[] = { 0x98, 0x88, 0x1B, 0xE4, 0x30, 0x79, 0x03, 0x84 };

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_superbit(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t status = superbitrf.status;
  uint8_t cyrf6936_status = superbitrf.cyrf6936.status;
  pprz_msg_send_SUPERBITRF(trans, dev, AC_ID,
                           &status,
                           &cyrf6936_status,
                           &superbitrf.irq_count,
                           &superbitrf.rx_packet_count,
                           &superbitrf.tx_packet_count,
                           &superbitrf.transfer_timeouts,
                           &superbitrf.resync_count,
                           &superbitrf.uplink_count,
                           &superbitrf.rc_count,
                           &superbitrf.timing1,
                           &superbitrf.timing2,
                           &superbitrf.bind_mfg_id32,
                           6,
                           superbitrf.cyrf6936.mfg_id);
}
#endif

// Functions for the generic device API
static bool_t superbitrf_check_free_space(struct SuperbitRF *p, uint8_t len)
{
  int16_t space = p->tx_extract_idx - p->tx_insert_idx;
  if (space <= 0) {
    space += SUPERBITRF_TX_BUFFER_SIZE;
  }
  return (uint16_t)(space - 1) >= len;
}

static void superbitrf_transmit(struct SuperbitRF *p, uint8_t byte)
{
  p->tx_buffer[p->tx_insert_idx] = byte;
  p->tx_insert_idx = (p->tx_insert_idx + 1) % SUPERBITRF_TX_BUFFER_SIZE;
}

static void superbitrf_send(struct SuperbitRF *p __attribute__((unused))) { }

static int null_function(struct SuperbitRF *p __attribute__((unused))) { return 0; }

/**
 * Initialize the superbitrf
 */
void superbitrf_init(void)
{
  // Set the status to uninitialized and set the timer to 0
  superbitrf.status = SUPERBITRF_UNINIT;
  superbitrf.state = 0;
  superbitrf.timer = 0;
  superbitrf.rx_packet_count = 0;
  superbitrf.tx_packet_count = 0;

  // Setup the transmit buffer
  superbitrf.tx_insert_idx = 0;
  superbitrf.tx_extract_idx = 0;

  superbitrf.bind_mfg_id32 = 0;
  superbitrf.num_channels = 0;
  superbitrf.protocol = 0;

  // Configure generic device
  superbitrf.device.periph = (void *)(&superbitrf);
  superbitrf.device.check_free_space = (check_free_space_t) superbitrf_check_free_space;
  superbitrf.device.put_byte = (put_byte_t) superbitrf_transmit;
  superbitrf.device.send_message = (send_message_t) superbitrf_send;
  superbitrf.device.char_available = (char_available_t) null_function; // not needed
  superbitrf.device.get_byte = (get_byte_t) null_function; // not needed

  // Initialize the binding pin
  gpio_setup_input(SPEKTRUM_BIND_PIN_PORT, SPEKTRUM_BIND_PIN);

  // Initialize the IRQ/DRDY pin
  gpio_setup_input(SUPERBITRF_DRDY_PORT, SUPERBITRF_DRDY_PIN);

  // Initialize the cyrf6936 chip
  cyrf6936_init(&superbitrf.cyrf6936, &(SUPERBITRF_SPI_DEV), 2, SUPERBITRF_RST_PORT, SUPERBITRF_RST_PIN);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SUPERBITRF, send_superbit);
#endif
}

void superbitrf_set_mfg_id(uint32_t id)
{
  superbitrf.bind_mfg_id32 = id;
  superbitrf.bind_mfg_id[0] = (superbitrf.bind_mfg_id32 & 0xFF);
  superbitrf.bind_mfg_id[1] = (superbitrf.bind_mfg_id32 >> 8 & 0xFF);
  superbitrf.bind_mfg_id[2] = (superbitrf.bind_mfg_id32 >> 16 & 0xFF);
  superbitrf.bind_mfg_id[3] = (superbitrf.bind_mfg_id32 >> 24 & 0xFF);

  // Calculate some values based on the bind MFG id
  superbitrf.crc_seed = ~((superbitrf.bind_mfg_id[0] << 8) + superbitrf.bind_mfg_id[1]);
  superbitrf.sop_col = (superbitrf.bind_mfg_id[0] + superbitrf.bind_mfg_id[1] + superbitrf.bind_mfg_id[2] + 2) & 0x07;
  superbitrf.data_col = 7 - superbitrf.sop_col;
}

void superbitrf_set_protocol(uint8_t protocol)
{
  superbitrf.protocol = protocol;
  superbitrf.resolution = (superbitrf.protocol & 0x10) >> 4;
}

/**
 * The superbitrf on event call
 */
void superbitrf_event(void)
{
  uint8_t i, pn_row, data_code[16];
  static uint8_t packet_size, tx_packet[16];
  static bool_t start_transfer = TRUE;

#ifdef RADIO_CONTROL_LED
  static uint32_t slowLedCpt = 0;
#endif

  // Check if the cyrf6936 isn't busy and the uperbitrf is initialized
  if (superbitrf.cyrf6936.status != CYRF6936_IDLE) {
    return;
  }

  // When the device is initialized handle the IRQ
  if (superbitrf.status != SUPERBITRF_UNINIT) {
    // First handle the IRQ
    if (gpio_get(SUPERBITRF_DRDY_PORT, SUPERBITRF_DRDY_PIN) == 0) {
      // Receive the packet
      cyrf6936_read_rx_irq_status_packet(&superbitrf.cyrf6936);
      superbitrf.irq_count++;
    }

    /* Check if it is a valid receive */
    if (superbitrf.cyrf6936.has_irq && (superbitrf.cyrf6936.rx_irq_status & CYRF_RXC_IRQ)) {
      // Handle the packet received
      superbitrf_receive_packet_cb((superbitrf.cyrf6936.rx_irq_status & CYRF_RXE_IRQ), superbitrf.cyrf6936.rx_status,
                                   superbitrf.cyrf6936.rx_packet);
      superbitrf.rx_packet_count++;

      // Reset the packet receiving
      superbitrf.cyrf6936.has_irq = FALSE;
    }

    /* Check if it has a valid send */
    if (superbitrf.cyrf6936.has_irq && (superbitrf.cyrf6936.tx_irq_status & CYRF_TXC_IRQ)) {
      // Handle the send packet
      superbitrf_send_packet_cb((superbitrf.cyrf6936.rx_irq_status & CYRF_TXE_IRQ));
      superbitrf.tx_packet_count++;

      // Reset the packet receiving
      superbitrf.cyrf6936.has_irq = FALSE;
    }
  }

  // Check the status of the superbitrf
  switch (superbitrf.status) {

      /* When the superbitrf isn't initialized */
    case SUPERBITRF_UNINIT:
      // Try to write the startup config
      if (cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_stratup_config, 11)) {
        // Check if need to go to bind or transfer
        if (gpio_get(SPEKTRUM_BIND_PIN_PORT, SPEKTRUM_BIND_PIN) == 0) {
          start_transfer = FALSE;
        }

        superbitrf.status = SUPERBITRF_INIT_BINDING;
      }
      break;

      /* When the superbitrf is initializing binding */
    case SUPERBITRF_INIT_BINDING:
      /* Switch the different states */
      switch (superbitrf.state) {
        case 0:
          // Try to write the binding config
          cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_bind_config, 5);
          superbitrf.state++;
          break;
        case 1:
          // Set the data code and channel
          memcpy(data_code, pn_codes[0][8], 8);
          memcpy(data_code + 8, pn_bind, 8);
          cyrf6936_write_chan_sop_data_crc(&superbitrf.cyrf6936, 1, pn_codes[0][0], data_code, 0x0000);

          superbitrf.status = SUPERBITRF_BINDING;
          break;
        default:
          // Should not happen
          superbitrf.state = 0;
          break;
      }
      break;

      /* When the superbitrf is initializing transfer */
    case SUPERBITRF_INIT_TRANSFER:
      // Generate the DSMX channels
      superbitrf_gen_dsmx_channels();

      // Try to write the transfer config
      if (cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_transfer_config, 4)) {
        superbitrf.resync_count = 0;
        superbitrf.packet_loss = FALSE;
        superbitrf.packet_loss_bit = 0;
        superbitrf.status = SUPERBITRF_SYNCING_A;
        superbitrf.state = 1;
      }
      break;

      /* When the superbitrf is in binding mode */
    case SUPERBITRF_BINDING:

#ifdef RADIO_CONTROL_LED
      slowLedCpt++;
      if (slowLedCpt > 100000) {

        LED_TOGGLE(RADIO_CONTROL_LED);
        slowLedCpt = 0;
      }
#endif

      /* Switch the different states */
      switch (superbitrf.state) {
        case 0:
          // When there is a timeout
          if (superbitrf.timer < get_sys_time_usec()) {
            superbitrf.state++;
          }
          break;
        case 1:
          // Abort the receive
          cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_abort_receive, 2);

          superbitrf.state++;
          break;
        case 2:
          // Switch channel
          superbitrf.channel = (superbitrf.channel + 2) % 0x4F; //TODO fix define
          cyrf6936_write(&superbitrf.cyrf6936, CYRF_CHANNEL, superbitrf.channel);

          superbitrf.state += 2; // Already aborted
          break;
        case 3:
          // Abort the receive
          cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_abort_receive, 2);

          superbitrf.state++;
          break;
        case 4:
          // Start receiving
          cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
          superbitrf.state++;
          break;
        default:
          // Check if need to go to transfer
          if (start_transfer) {
            // Initialize the binding values
            // set values based on mfg id
            // if bind_mfg_id32 is loaded from persistent settings use that,
            if (superbitrf.bind_mfg_id32) {
              superbitrf_set_mfg_id(superbitrf.bind_mfg_id32);
            }
#ifdef RADIO_TRANSMITTER_ID
            // otherwise load airframe file value
            else {
              PRINT_CONFIG_VAR(RADIO_TRANSMITTER_ID)
              superbitrf_set_mfg_id(RADIO_TRANSMITTER_ID);
            }
#endif
#ifdef RADIO_TRANSMITTER_CHAN
            PRINT_CONFIG_VAR(RADIO_TRANSMITTER_CHAN)
            if (superbitrf.num_channels == 0) {
              superbitrf.num_channels = RADIO_TRANSMITTER_CHAN;
            }
#endif
            if (superbitrf.protocol == 0) {
              superbitrf_set_protocol(superbitrf.protocol);
            }
#ifdef RADIO_TRANSMITTER_PROTOCOL
            else {
              PRINT_CONFIG_VAR(RADIO_TRANSMITTER_PROTOCOL)
              superbitrf_set_protocol(RADIO_TRANSMITTER_PROTOCOL);
            }
#endif

            // Start transfer
            superbitrf.state = 0;
            superbitrf.status = SUPERBITRF_INIT_TRANSFER;
            break;
          }

          // Set the timer
          superbitrf.timer = (get_sys_time_usec() + SUPERBITRF_BIND_RECV_TIME) % 0xFFFFFFFF;
          superbitrf.state = 0;
          break;
      }
      break;

      /* When the receiver is synchronizing with the transmitter */
    case SUPERBITRF_SYNCING_A:
    case SUPERBITRF_SYNCING_B:

#ifdef RADIO_CONTROL_LED
      slowLedCpt++;
      if (slowLedCpt > 5000) {

        LED_TOGGLE(RADIO_CONTROL_LED);
        slowLedCpt = 0;
      }
#endif

      /* Switch the different states */
      switch (superbitrf.state) {
        case 0:
          // When there is a timeout
          if (superbitrf.timer < get_sys_time_usec()) {
            superbitrf.state++;
          }
          break;
        case 1:
          // Abort the receive
          cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_abort_receive, 2);
          superbitrf.state++;
          break;
        case 2:
          // Switch channel, sop code, data code and crc
          superbitrf.channel_idx = (IS_DSM2(superbitrf.protocol)
                                    || SUPERBITRF_FORCE_DSM2) ? (superbitrf.channel_idx + 1) % 2 : (superbitrf.channel_idx + 1) % 23;
          pn_row = (IS_DSM2(superbitrf.protocol)
                    || SUPERBITRF_FORCE_DSM2) ? superbitrf.channels[superbitrf.channel_idx] % 5 :
                   (superbitrf.channels[superbitrf.channel_idx] - 2) % 5;

          cyrf6936_write_chan_sop_data_crc(&superbitrf.cyrf6936, superbitrf.channels[superbitrf.channel_idx],
                                           pn_codes[pn_row][superbitrf.sop_col],
                                           pn_codes[pn_row][superbitrf.data_col],
                                           superbitrf.crc_seed);
          superbitrf.state++;
          break;
        case 3:
          // Create a new packet when no packet loss
          if (!superbitrf.packet_loss) {
            superbitrf.packet_loss_bit = !superbitrf.packet_loss_bit;
            if (IS_DSM2(superbitrf.protocol) || SUPERBITRF_FORCE_DSM2) {
              tx_packet[0] = ~superbitrf.bind_mfg_id[2];
              tx_packet[1] = (~superbitrf.bind_mfg_id[3]) + 1 + superbitrf.packet_loss_bit;
            } else {
              tx_packet[0] = superbitrf.bind_mfg_id[2];
              tx_packet[1] = (superbitrf.bind_mfg_id[3]) + 1 + superbitrf.packet_loss_bit;
            }

            packet_size = (superbitrf.tx_insert_idx - superbitrf.tx_extract_idx + SUPERBITRF_TX_BUFFER_SIZE %
                           SUPERBITRF_TX_BUFFER_SIZE);
            if (packet_size > 14) {
              packet_size = 14;
            }

            for (i = 0; i < packet_size; i++) {
              tx_packet[i + 2] = superbitrf.tx_buffer[(superbitrf.tx_extract_idx + i) % SUPERBITRF_TX_BUFFER_SIZE];
            }
          }

          // Send a packet
          cyrf6936_send(&superbitrf.cyrf6936, tx_packet, packet_size + 2);

          // Update the packet extraction
          if (!superbitrf.packet_loss) {
            superbitrf.tx_extract_idx = (superbitrf.tx_extract_idx + packet_size) % SUPERBITRF_TX_BUFFER_SIZE;
          }

          superbitrf.state++;
          break;
        case 4:
          //TODO: check timeout? (Waiting for send)
          break;
        case 5:
          superbitrf.state = 7;
          break;
          // Start receiving
          cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
          superbitrf.timer = (get_sys_time_usec() + SUPERBITRF_DATARECVB_TIME) % 0xFFFFFFFF;
          superbitrf.state++;
          break;
        case 6:
          // Wait for telemetry data
          if (superbitrf.timer < get_sys_time_usec()) {
            superbitrf.state++;
          }
          break;
        case 7:
          // When DSMX we don't need to switch
          if (IS_DSMX(superbitrf.protocol) && !SUPERBITRF_FORCE_DSM2) {
            superbitrf.state++;
            superbitrf.channel = superbitrf.channels[superbitrf.channel_idx];
            break;
          }

          // Switch channel, sop code, data code and crc
          superbitrf.channel = (superbitrf.channel + 2) % 0x4F; //TODO fix define
          superbitrf.crc_seed = ~superbitrf.crc_seed;
          pn_row = (IS_DSM2(superbitrf.protocol)
                    || SUPERBITRF_FORCE_DSM2) ? superbitrf.channel % 5 : (superbitrf.channel - 2) % 5;

          cyrf6936_write_chan_sop_data_crc(&superbitrf.cyrf6936, superbitrf.channel,
                                           pn_codes[pn_row][superbitrf.sop_col],
                                           pn_codes[pn_row][superbitrf.data_col],
                                           superbitrf.crc_seed);

          superbitrf.state++;
          break;
        case 8:
          // Start receiving
          cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
          superbitrf.state++;
          break;
        default:
          // Set the timer
          superbitrf.timer = (get_sys_time_usec() + SUPERBITRF_SYNC_RECV_TIME) % 0xFFFFFFFF;
          superbitrf.state = 0;
          break;
      }
      break;

      /* Normal transfer mode */
    case SUPERBITRF_TRANSFER:

#ifdef RADIO_CONTROL_LED
      LED_ON(RADIO_CONTROL_LED);
#endif

      /* Switch the different states */
      switch (superbitrf.state) {
        case 0:
          // Fixing timer overflow
          if (superbitrf.timer_overflow && get_sys_time_usec() <= superbitrf.timer) {
            superbitrf.timer_overflow = FALSE;
          }

          // When there is a timeout
          if (superbitrf.timer < get_sys_time_usec() && !superbitrf.timer_overflow) {
            superbitrf.transfer_timeouts++;
            superbitrf.timeouts++;
            superbitrf.state++;
          }

          // We really lost the communication
          if (superbitrf.timeouts > 100) {
            superbitrf.state = 0;
            superbitrf.resync_count++;
            superbitrf.status = SUPERBITRF_SYNCING_A;
          }
          break;
        case 1:
          // Abort the receive
          cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_abort_receive, 2);
          superbitrf.state++;

          // Set the timer
          superbitrf.timer = (get_sys_time_usec() + SUPERBITRF_DATARECV_TIME) % 0xFFFFFFFF;
          if (superbitrf.timer < get_sys_time_usec()) {
            superbitrf.timer_overflow = TRUE;
          } else {
            superbitrf.timer_overflow = FALSE;
          }

          // Only send on channel 2
          if (superbitrf.crc_seed != ((superbitrf.bind_mfg_id[0] << 8) + superbitrf.bind_mfg_id[1])) {
            superbitrf.state = 8;
          }
          break;
        case 2:
          // Wait before sending (FIXME??)
          superbitrf.state++;
          break;
        case 3:
          // Create a new packet when no packet loss
          if (!superbitrf.packet_loss) {
            superbitrf.packet_loss_bit = !superbitrf.packet_loss_bit;
            if (IS_DSM2(superbitrf.protocol) || SUPERBITRF_FORCE_DSM2) {
              tx_packet[0] = ~superbitrf.bind_mfg_id[2];
              tx_packet[1] = ((~superbitrf.bind_mfg_id[3]) + 1 + superbitrf.packet_loss_bit) % 0xFF;
            } else {
              tx_packet[0] = superbitrf.bind_mfg_id[2];
              tx_packet[1] = ((superbitrf.bind_mfg_id[3]) + 1 + superbitrf.packet_loss_bit) % 0xFF;
            }

            packet_size = (superbitrf.tx_insert_idx - superbitrf.tx_extract_idx + SUPERBITRF_TX_BUFFER_SIZE %
                           SUPERBITRF_TX_BUFFER_SIZE);
            if (packet_size > 14) {
              packet_size = 14;
            }

            for (i = 0; i < packet_size; i++) {
              tx_packet[i + 2] = superbitrf.tx_buffer[(superbitrf.tx_extract_idx + i) % SUPERBITRF_TX_BUFFER_SIZE];
            }
          }

          // Send a packet
          cyrf6936_send(&superbitrf.cyrf6936, tx_packet, packet_size + 2);

          // Update the packet extraction
          if (!superbitrf.packet_loss) {
            superbitrf.tx_extract_idx = (superbitrf.tx_extract_idx + packet_size) % SUPERBITRF_TX_BUFFER_SIZE;
          }

          superbitrf.state++;
          break;
        case 4:
          //TODO: check timeout? (Waiting for send)
          break;
        case 5:
          // Start receiving
          cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
          superbitrf.state++;
          break;
        case 6:
          // Fixing timer overflow
          if (superbitrf.timer_overflow && get_sys_time_usec() <= superbitrf.timer) {
            superbitrf.timer_overflow = FALSE;
          }

          // Waiting for data receive
          if (superbitrf.timer < get_sys_time_usec() && !superbitrf.timer_overflow) {
            superbitrf.state++;
          }
          break;
        case 7:
          // Abort the receive
          cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_abort_receive, 2);
          superbitrf.state++;
          break;
        case 8:
          // Switch channel, sop code, data code and crc
          superbitrf.channel_idx = (IS_DSM2(superbitrf.protocol)
                                    || SUPERBITRF_FORCE_DSM2) ? (superbitrf.channel_idx + 1) % 2 : (superbitrf.channel_idx + 1) % 23;
          superbitrf.channel = superbitrf.channels[superbitrf.channel_idx];
          superbitrf.crc_seed = ~superbitrf.crc_seed;
          pn_row = (IS_DSM2(superbitrf.protocol)
                    || SUPERBITRF_FORCE_DSM2) ? superbitrf.channel % 5 : (superbitrf.channel - 2) % 5;

          cyrf6936_write_chan_sop_data_crc(&superbitrf.cyrf6936, superbitrf.channel,
                                           pn_codes[pn_row][superbitrf.sop_col],
                                           pn_codes[pn_row][superbitrf.data_col],
                                           superbitrf.crc_seed);

          superbitrf.state++;
          break;
        case 9:
          // Start receiving
          cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
          superbitrf.state++;
          break;
        default:
          // Set the timer
          if (superbitrf.crc_seed != ((superbitrf.bind_mfg_id[0] << 8) + superbitrf.bind_mfg_id[1])) {
            superbitrf.timer = (superbitrf.timer - SUPERBITRF_DATARECV_TIME + SUPERBITRF_RECV_TIME) % 0xFFFFFFFF;
          } else {
            superbitrf.timer = (superbitrf.timer - SUPERBITRF_DATARECV_TIME + SUPERBITRF_RECV_SHORT_TIME) % 0xFFFFFFFF;
          }
          if (superbitrf.timer < get_sys_time_usec()) {
            superbitrf.timer_overflow = TRUE;
          } else {
            superbitrf.timer_overflow = FALSE;
          }

          superbitrf.state = 0;
          break;
      }
      break;

      /* Should not come here */
    default:
      break;
  }
}

/**
 * When we receive a packet this callback is called
 */
static inline void superbitrf_receive_packet_cb(bool_t error, uint8_t status, uint8_t packet[])
{
  int i;
  uint16_t sum;

  /* Switch on the status of the superbitRF */
  switch (superbitrf.status) {

      /* When we are in binding mode */
    case SUPERBITRF_BINDING:
      // Check if the MFG id is exactly the same
      if (packet[0] != packet[4] || packet[1] != packet[5] || packet[2] != packet[6] || packet[3] != packet[7]) {
        // Start receiving without changing channel
        superbitrf.state = 3;
        break;
      }

      // Calculate the first sum
      sum = 384 - 0x10;
      for (i = 0; i < 8; i++) {
        sum += packet[i];
      }

      // Check the first sum
      if (packet[8] != sum >> 8 || packet[9] != (sum & 0xFF)) {
        // Start receiving without changing channel
        superbitrf.state = 3;
        break;
      }

      // Calculate second sum
      for (i = 8; i < 14; i++) {
        sum += packet[i];
      }

      // Check the second sum
      if (packet[14] != sum >> 8 || packet[15] != (sum & 0xFF)) {
        // Start receiving without changing channel
        superbitrf.state = 3;
        break;
      }

      // Update the mfg id, number of channels and protocol
      uint32_t mfg_id = ((~packet[3] & 0xFF) << 24 | (~packet[2] & 0xFF) << 16 |
                         (~packet[1] & 0xFF) << 8 | (~packet[0] & 0xFF));
      superbitrf_set_mfg_id(mfg_id);

      superbitrf.num_channels = packet[11];
      superbitrf_set_protocol(packet[12]);

      // Store all the persistent settings.
      // In case we have the superbit setting file loaded and persistent settings
      // enabled in the airframe file this will store our binding information and
      // survive a reboot.
      settings_StoreSettings(1);

      // Update the status of the receiver
      superbitrf.state = 0;
      superbitrf.status = SUPERBITRF_INIT_TRANSFER;
      break;

      /* When we receive a packet during syncing first channel A */
    case SUPERBITRF_SYNCING_A:
      // Check the MFG id
      if (error && !(status & CYRF_BAD_CRC)) {
        // Start receiving TODO: Fix nicely
        cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
        break;
      }
      if ((IS_DSM2(superbitrf.protocol) || SUPERBITRF_FORCE_DSM2) &&
          (packet[0] != (~superbitrf.bind_mfg_id[2] & 0xFF) || (packet[1] != (~superbitrf.bind_mfg_id[3] & 0xFF) &&
              packet[1] != (~superbitrf.bind_mfg_id[3] & 0xFF) + 1))) {
        // Start receiving TODO: Fix nicely
        cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
        break;
      }
      if ((IS_DSMX(superbitrf.protocol) && !SUPERBITRF_FORCE_DSM2) &&
          (packet[0] != (superbitrf.bind_mfg_id[2] & 0xFF) || (packet[1] != (superbitrf.bind_mfg_id[3] & 0xFF) &&
              packet[1] != (superbitrf.bind_mfg_id[3] & 0xFF) + 1))) {
        // Start receiving TODO: Fix nicely
        cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
        break;
      }

      // If the CRC is wrong invert
      if (error && (status & CYRF_BAD_CRC)) {
        superbitrf.crc_seed = ~superbitrf.crc_seed;
      }

      // When we receive a data packet
      if (packet[1] != (~superbitrf.bind_mfg_id[3] & 0xFF) && packet[1] != (superbitrf.bind_mfg_id[3] & 0xFF)) {
        superbitrf.uplink_count++;

        // Check if it is a data loss packet
        if (packet[1] != (~superbitrf.bind_mfg_id[3] + 1 + superbitrf.packet_loss_bit) % 0xFF
            && packet[1] != (superbitrf.bind_mfg_id[3] + 1 + superbitrf.packet_loss_bit) % 0xFF) {
          superbitrf.packet_loss = TRUE;
        } else {
          superbitrf.packet_loss = FALSE;
        }

        // When it is a data packet, parse the packet if not busy already
        if (!dl_msg_available && !superbitrf.packet_loss) {
          for (i = 2; i < superbitrf.cyrf6936.rx_count; i++) {
            parse_pprz(&superbitrf.rx_transport, packet[i]);

            // When we have a full message
            if (superbitrf.rx_transport.trans_rx.msg_received) {
              pprz_parse_payload(&superbitrf.rx_transport);
              superbitrf.rx_transport.trans_rx.msg_received = FALSE;
            }
          }
        }
        break;
      }

      if (IS_DSM2(superbitrf.protocol) || SUPERBITRF_FORCE_DSM2) {
        superbitrf.channels[0] = superbitrf.channel;
        superbitrf.channels[1] = superbitrf.channel;

        superbitrf.state = 1;
        superbitrf.status = SUPERBITRF_SYNCING_B;
      } else {
        superbitrf.timeouts = 0;
        superbitrf.state = 1;
        superbitrf.status = SUPERBITRF_TRANSFER;
      }
      break;

      /* When we receive a packet during syncing second channel B */
    case SUPERBITRF_SYNCING_B:
      // Check the MFG id
      if (error && !(status & CYRF_BAD_CRC)) {
        // Start receiving TODO: Fix nicely
        cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
        break;
      }
      if ((IS_DSM2(superbitrf.protocol) || SUPERBITRF_FORCE_DSM2) &&
          (packet[0] != (~superbitrf.bind_mfg_id[2] & 0xFF) || (packet[1] != (~superbitrf.bind_mfg_id[3] & 0xFF) &&
              packet[1] != (~superbitrf.bind_mfg_id[3] & 0xFF) + 1 && packet[1] != (~superbitrf.bind_mfg_id[3] & 0xFF) + 2))) {
        // Start receiving TODO: Fix nicely
        cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
        break;
      }
      if ((IS_DSMX(superbitrf.protocol) && !SUPERBITRF_FORCE_DSM2) &&
          (packet[0] != (superbitrf.bind_mfg_id[2] & 0xFF) || (packet[1] != (superbitrf.bind_mfg_id[3] & 0xFF) &&
              packet[1] != (superbitrf.bind_mfg_id[3] & 0xFF) + 1 && packet[1] != (superbitrf.bind_mfg_id[3] & 0xFF) + 2))) {
        // Start receiving TODO: Fix nicely
        cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
        break;
      }

      // If the CRC is wrong invert
      if (error && (status & CYRF_BAD_CRC)) {
        superbitrf.crc_seed = ~superbitrf.crc_seed;
      }

      // When we receive a data packet
      if (packet[1] != (~superbitrf.bind_mfg_id[3] & 0xFF) && packet[1] != (superbitrf.bind_mfg_id[3] & 0xFF)) {
        superbitrf.uplink_count++;

        // When it is a data packet, parse the packet if not busy already
        if (!dl_msg_available) {
          for (i = 2; i < superbitrf.cyrf6936.rx_count; i++) {
            parse_pprz(&superbitrf.rx_transport, packet[i]);

            // When we have a full message
            if (superbitrf.rx_transport.trans_rx.msg_received) {
              pprz_parse_payload(&superbitrf.rx_transport);
              superbitrf.rx_transport.trans_rx.msg_received = FALSE;
            }
          }
        }
        break;
      }

      // Set the channel
      if (superbitrf.channels[0] != superbitrf.channel) {
        superbitrf.channels[0] = superbitrf.channel;
        superbitrf.channel_idx = 0;
      } else {
        superbitrf.channels[1] = superbitrf.channel;
        superbitrf.channel_idx = 1;
      }

      // When the channels aren't the same go to transfer mode
      if (superbitrf.channels[1] != superbitrf.channels[0]) {
        superbitrf.state = 1;
        superbitrf.status = SUPERBITRF_TRANSFER;
        superbitrf.timeouts = 0;
      }
      break;

      /* When we receive a packet during transfer */
    case SUPERBITRF_TRANSFER:
      // Check the MFG id
      if (error) {
        // Start receiving TODO: Fix nicely
        cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
        break;
      }
      if ((IS_DSM2(superbitrf.protocol) || SUPERBITRF_FORCE_DSM2) &&
          (packet[0] != (~superbitrf.bind_mfg_id[2] & 0xFF) || (packet[1] != (~superbitrf.bind_mfg_id[3] & 0xFF) &&
              packet[1] != (~superbitrf.bind_mfg_id[3] & 0xFF) + 1 && packet[1] != (~superbitrf.bind_mfg_id[3] & 0xFF) + 2))) {
        // Start receiving TODO: Fix nicely
        cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
        break;
      }
      if ((IS_DSMX(superbitrf.protocol) && !SUPERBITRF_FORCE_DSM2) &&
          (packet[0] != (superbitrf.bind_mfg_id[2] & 0xFF) || (packet[1] != (superbitrf.bind_mfg_id[3] & 0xFF) &&
              packet[1] != (superbitrf.bind_mfg_id[3] & 0xFF) + 1 && packet[1] != (superbitrf.bind_mfg_id[3] & 0xFF) + 2))) {
        // Start receiving TODO: Fix nicely
        cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_start_receive, 2);
        break;
      }

      // Check if it is a RC packet
      if (packet[1] == (~superbitrf.bind_mfg_id[3] & 0xFF) || packet[1] == (superbitrf.bind_mfg_id[3] & 0xFF)) {
        superbitrf.rc_count++;

        // Parse the packet
        superbitrf_radio_to_channels(&packet[2], superbitrf.num_channels, superbitrf.resolution, superbitrf.rc_values);
        superbitrf.rc_frame_available = TRUE;

        // Calculate the timing (seperately for the channel switches)
        if (superbitrf.crc_seed != ((superbitrf.bind_mfg_id[0] << 8) + superbitrf.bind_mfg_id[1])) {
          superbitrf.timing2 = get_sys_time_usec() - (superbitrf.timer - SUPERBITRF_RECV_TIME);
        } else {
          superbitrf.timing1 = get_sys_time_usec() - (superbitrf.timer - SUPERBITRF_RECV_SHORT_TIME);
        }

        // Go to next receive
        superbitrf.state = 1;
        superbitrf.timeouts = 0;
      } else {
        superbitrf.uplink_count++;

        // Check if it is a data loss packet
        if (packet[1] != (~superbitrf.bind_mfg_id[3] + 1 + superbitrf.packet_loss_bit)
            && packet[1] != (superbitrf.bind_mfg_id[3] + 1 + superbitrf.packet_loss_bit)) {
          superbitrf.packet_loss = TRUE;
        } else {
          superbitrf.packet_loss = FALSE;
        }

        superbitrf.packet_loss = FALSE;

        // When it is a data packet, parse the packet if not busy already
        if (!dl_msg_available && !superbitrf.packet_loss) {
          for (i = 2; i < superbitrf.cyrf6936.rx_count; i++) {
            parse_pprz(&superbitrf.rx_transport, packet[i]);

            // When we have a full message
            if (superbitrf.rx_transport.trans_rx.msg_received) {
              pprz_parse_payload(&superbitrf.rx_transport);
              superbitrf.rx_transport.trans_rx.msg_received = FALSE;
            }
          }
        }

        // Update the state
        superbitrf.state = 7;
      }
      break;

      /* Should not come here */
    default:
      break;
  }
}

static inline void superbitrf_send_packet_cb(bool_t error __attribute__((unused)))
{
  /* Switch on the status of the superbitRF */
  switch (superbitrf.status) {

      /* When we are synchronizing */
    case SUPERBITRF_SYNCING_A:
    case SUPERBITRF_SYNCING_B:
      // When we successfully or unsuccessfully send a data packet
      if (superbitrf.state == 4) {
        superbitrf.state++;
      }
      break;

      /* When we are in transfer mode */
    case SUPERBITRF_TRANSFER:
      // When we successfully or unsuccessfully send a packet
      if (superbitrf.state == 4) {
        superbitrf.state++;
      }
      break;

      /* Should not come here */
    default:
      break;
  }
}

/**
 * Parse a radio channel packet
 */
static inline void superbitrf_radio_to_channels(uint8_t *data, uint8_t nb_channels, bool_t is_11bit, int16_t *channels)
{
  int i;
  uint8_t bit_shift = (is_11bit) ? 11 : 10;
  int16_t value_max = (is_11bit) ? 0x07FF : 0x03FF;

  for (i = 0; i < 7; i++) {
    const int16_t tmp = ((data[2 * i] << 8) + data[2 * i + 1]) & 0x7FFF;
    const uint8_t chan = (tmp >> bit_shift) & 0x0F;
    const int16_t val  = (tmp & value_max);

    if (chan < nb_channels) {
      channels[chan] = val;

      // Scale the channel
      if (is_11bit) {
        channels[chan] -= 0x400;
        channels[chan] *= MAX_PPRZ / 0x2AC;
      } else {
        channels[chan] -= 0x200;
        channels[chan] *= MAX_PPRZ / 0x156;
      }
    }
  }
}

/**
 * Generate the channels
 */
static inline void superbitrf_gen_dsmx_channels(void)
{
  // Calculate the DSMX channels
  int idx = 0;
  uint32_t id = ~((superbitrf.bind_mfg_id[0] << 24) | (superbitrf.bind_mfg_id[1] << 16) |
                  (superbitrf.bind_mfg_id[2] << 8) | (superbitrf.bind_mfg_id[3] << 0));
  uint32_t id_tmp = id;

  // While not all channels are set
  while (idx < 23) {
    int i;
    int count_3_27 = 0, count_28_51 = 0, count_52_76 = 0;

    id_tmp = id_tmp * 0x0019660D + 0x3C6EF35F; // Randomization
    uint8_t next_ch = ((id_tmp >> 8) % 0x49) + 3;       // Use least-significant byte and must be larger than 3
    if (((next_ch ^ id) & 0x01) == 0) {
      continue;
    }

    // Go trough all already set channels
    for (i = 0; i < idx; i++) {
      // Channel is already used
      if (superbitrf.channels[i] == next_ch) {
        break;
      }

      // Count the channel groups
      if (superbitrf.channels[i] <= 27) {
        count_3_27++;
      } else if (superbitrf.channels[i] <= 51) {
        count_28_51++;
      } else {
        count_52_76++;
      }
    }

    // When channel is already used continue
    if (i != idx) {
      continue;
    }

    // Set the channel when channel groups aren't full
    if ((next_ch < 28 && count_3_27 < 8)                        // Channels 3-27: max 8
        || (next_ch >= 28 && next_ch < 52 && count_28_51 < 7)     // Channels 28-52: max 7
        || (next_ch >= 52 && count_52_76 < 8)) {                  // Channels 52-76: max 8
      superbitrf.channels[idx++] = next_ch;
    }
  }
}



