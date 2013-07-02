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
 * @file subsystems/radio_control/superbitrf.c
 * DSM2 and DSMX implementation for the cyrf6936 2.4GHz radio chip trough SPI
 */

#include "superbitrf.h"

#include "subsystems/radio_control.h"
#include "mcu_periph/spi.h"
#include <libopencm3/stm32/gpio.h>

/* Default SuperbitRF SPI DEV */
#ifndef SUPERBITRF_SPI_DEV
#define SUPERBITRF_SPI_DEV      spi1
#endif

/* Default SuperbitRF RST PORT and PIN */
#ifndef SUPERBITRF_RST_PORT
#define SUPERBITRF_RST_PORT     GPIOC
#endif
#ifndef SUPERBITRF_RST_PIN
#define SUPERBITRF_RST_PIN      GPIO12
#endif

/* Default SuperbitRF DRDY(IRQ) PORT and PIN */
#ifndef SUPERBITRF_DRDY_PORT
#define SUPERBITRF_DRDY_PORT     GPIOB
#endif
#ifndef SUPERBITRF_DRDY_PIN
#define SUPERBITRF_DRDY_PIN      GPIO1
#endif

/* The superbitRF structure */
struct SuperbitRF superbitrf;

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

/**
 * Initialize the superbitrf
 */
void radio_control_impl_init(void) {
  // Set the status to uninitialized
  superbitrf.status = SUPERBITRF_UNINIT;

  // Initialize the binding pin
  gpio_setup_input(SPEKTRUM_BIND_PIN_PORT, SPEKTRUM_BIND_PIN);

  // Initialize the cyrf6936 chip
  cyrf6936_init(&superbitrf.cyrf6936, &(SUPERBITRF_SPI_DEV), 1, SUPERBITRF_RST_PORT, SUPERBITRF_RST_PIN);
}

/**
 * The superbitrf on event call
 */
void superbitrf_event(void) {
  // Check the status
  switch(superbitrf.status) {

  /* When the superbitrf isn't initialized */
  case SUPERBITRF_UNINIT:
    // Try to write the startup config
    if(cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_stratup_config, 11)) {
      // Check if need to go to bind or transfer
      if(gpio_get(SPEKTRUM_BIND_PIN_PORT, SPEKTRUM_BIND_PIN) == 0)
        superbitrf.status = SUPERBITRF_INIT_BINDING;
      else
        superbitrf.status = SUPERBITRF_INIT_TRANSFER;
    }
    break;

  /* When the superbitrf is initializing binding */
  case SUPERBITRF_INIT_BINDING:
    // Try to write the binding config
    if(cyrf6936_multi_write(&superbitrf.cyrf6936, cyrf_bind_config, 5))
      superbitrf.status = SUPERBITRF_BINDING;
    break;

  /* When the superbitrf is in binding mode */
  case SUPERBITRF_BINDING:
    break;

  /* Should not come here */
  default:
    break;
  }

}
