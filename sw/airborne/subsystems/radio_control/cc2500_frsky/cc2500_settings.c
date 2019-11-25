/*
 * Copyright (C) 2019 Tom van Dijk <tomvand@users.noreply.github.com>
 *
 * This code is based on the betaflight cc2500 and FrskyX implementation.
 * https://github.com/betaflight/betaflight
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

#include "cc2500_settings.h"

#include "cc2500_rx_spi.h"

#include "mcu_periph/gpio.h"

#include <string.h>


// main/config/config.h:
void bf_writeEEPROM(void) { } // TODO Handled by Paparazzi's persistent settings mechanism.


// main/pg/rx_spi.h:
static struct gpio_t extiIo;
static struct gpio_t ledIo;
static struct gpio_t bindIo;
static rxSpiConfig_t spiconfig;
const rxSpiConfig_t* rxSpiConfig(void) {
  return &spiconfig;
}


// main/pg/rx_spi_cc2500.h:
static rxCc2500SpiConfig_t cc2500spiconfig;
const rxCc2500SpiConfig_t* rxCc2500SpiConfig(void) {
  return &cc2500spiconfig;
}
rxCc2500SpiConfig_t* rxCc2500SpiConfigMutable(void) {
  return &cc2500spiconfig;
}


void cc2500_settings_init(void) {
  // rxSpiConfig
  spiconfig.rx_spi_protocol = CC2500_RX_SPI_PROTOCOL;
  extiIo.port = CC2500_GDO0_GPIO_PORT;
  extiIo.pin = CC2500_GDO0_GPIO;
  spiconfig.extiIoTag = &extiIo;
  ledIo.port = CC2500_RX_LED_GPIO_PORT;
  ledIo.pin = CC2500_RX_LED_GPIO;
  spiconfig.ledIoTag = &ledIo;
  spiconfig.ledInversion = CC2500_RX_LED_INVERT;
  bindIo.port = CC2500_BIND_BTN_GPIO_PORT;
  bindIo.pin = CC2500_BIND_BTN_GPIO;
  spiconfig.bindIoTag = &bindIo;

  // rxCc2500SpiConfig
  cc2500spiconfig.autoBind = CC2500_AUTOBIND;
  cc2500spiconfig.bindTxId[0] = 0;
  cc2500spiconfig.bindTxId[1] = 0;
  cc2500spiconfig.bindOffset = 0;
  memset(cc2500spiconfig.bindHopData, 0, sizeof(cc2500spiconfig.bindHopData));
  cc2500spiconfig.rxNum = 0;
  cc2500spiconfig.a1Source = FRSKY_SPI_A1_SOURCE_VBAT;
  cc2500spiconfig.chipDetectEnabled = TRUE;
}
