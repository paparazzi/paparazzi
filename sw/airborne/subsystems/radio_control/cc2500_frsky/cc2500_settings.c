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

#include BOARD_CONFIG
#include "mcu_periph/gpio.h"
#include "generated/airframe.h"
#include "subsystems/settings.h"

#include <string.h>


#define _LED_GPIO(l) l ## _GPIO
#define LED_GPIO(l) _LED_GPIO(l)
#define _LED_GPIO_PIN(l) l ## _GPIO_PIN
#define LED_GPIO_PIN(l) _LED_GPIO_PIN(l)
#define _LED_GPIO_ON(l) l ## _GPIO_ON
#define LED_GPIO_ON(l) _LED_GPIO_ON(l)
#define _LED_GPIO_OFF(l) l ## _GPIO_OFF
#define LED_GPIO_OFF(l) _LED_GPIO_OFF(l)

#define _BUTTON_GPIO(b) b ## _GPIO
#define BUTTON_GPIO(b) _BUTTON_GPIO(b)
#define _BUTTON_PIN(b) b ## _PIN
#define BUTTON_PIN(b) _BUTTON_PIN(b)


#ifndef CC2500_RX_SPI_PROTOCOL
#define CC2500_RX_SPI_PROTOCOL RX_SPI_FRSKY_X_LBT
#endif

#ifndef CC2500_AUTOBIND
#define CC2500_AUTOBIND FALSE
#endif

#ifndef CC2500_TELEMETRY_SENSORS
#define CC2500_TELEMETRY_SENSORS (SENSOR_VOLTAGE | SENSOR_CURRENT | SENSOR_FUEL | SENSOR_ALTITUDE | SENSOR_VARIO)
#endif

static void cc2500_persistent_write(void);


// main/config/config.h:
void bf_writeEEPROM(void) {
  // Settings storage handled by paparazzi's persistent settings
  cc2500_persistent_write();
  settings_StoreSettings(1);
}


// main/pg/rx.h:
static rxConfig_t rxconfig;
const rxConfig_t* rxConfig(void) {
  return &rxconfig;
}


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


// main/telemetry/telemetry.h:
static telemetryConfig_t telemetryconfig;

const telemetryConfig_t* telemetryConfig(void) {
  return &telemetryconfig;
}


// main/telemetry/telemetry.h:
bool telemetryIsSensorEnabled(sensor_e sensor) {
  return sensor & CC2500_TELEMETRY_SENSORS;
}


// Paparazzi code
struct cc2500_settings_persistent_s cc2500_settings_persistent;

static void cc2500_persistent_write(void) {
  cc2500_settings_persistent.bindVars =
      cc2500spiconfig.bindTxId[0] |
      cc2500spiconfig.bindTxId[1] << 8 |
      cc2500spiconfig.bindOffset << 16 |
      cc2500spiconfig.rxNum << 24;
  for (int i = 0; i < 48; i += 4) {
    cc2500_settings_persistent.bindHopData[i / 4] =
        cc2500spiconfig.bindHopData[i] |
        cc2500spiconfig.bindHopData[i + 1] << 8 |
        cc2500spiconfig.bindHopData[i + 2] << 16 |
        cc2500spiconfig.bindHopData[i + 3] << 24;
  }
  cc2500_settings_persistent.bindHopData[12] =
      cc2500spiconfig.bindHopData[48] |
      cc2500spiconfig.bindHopData[49] << 8 |
      0xFF << 24;
}

static void cc2500_persistent_read(void) {
  // Check that persistent data is loaded
  // highest bindHopData byte is initialized 0 at boot
  if (cc2500_settings_persistent.bindHopData[12] >> 24) {
    cc2500spiconfig.bindTxId[0] = cc2500_settings_persistent.bindVars & 0xFF;
    cc2500spiconfig.bindTxId[1] = (cc2500_settings_persistent.bindVars >> 8) & 0xFF;
    cc2500spiconfig.bindOffset = (cc2500_settings_persistent.bindVars >> 16) & 0xFF;
    cc2500spiconfig.rxNum = (cc2500_settings_persistent.bindVars >> 24) & 0xFF;
  }
  for (int i = 0; i < 48; i += 4) {
    cc2500spiconfig.bindHopData[i] = cc2500_settings_persistent.bindHopData[i / 4] & 0xFF;
    cc2500spiconfig.bindHopData[i + 1] = (cc2500_settings_persistent.bindHopData[i / 4] >> 8) & 0xFF;
    cc2500spiconfig.bindHopData[i + 2] = (cc2500_settings_persistent.bindHopData[i / 4] >> 16) & 0xFF;
    cc2500spiconfig.bindHopData[i + 3] = (cc2500_settings_persistent.bindHopData[i / 4] >> 24) & 0xFF;
  }
  cc2500spiconfig.bindHopData[48] = cc2500_settings_persistent.bindHopData[12] & 0xFF;
  cc2500spiconfig.bindHopData[49] = (cc2500_settings_persistent.bindHopData[12] >> 8) & 0xFF;
}



void cc2500_settings_init(void) {
  // rxConfig
  rxconfig.rssi_channel = 0;
  rxconfig.midrc = 1500;
  rxconfig.max_aux_channel = 0; // TODO
  rxconfig.rssi_src_frame_lpf_period = 30;

  // rxSpiConfig
  spiconfig.rx_spi_protocol = CC2500_RX_SPI_PROTOCOL;
  extiIo.port = CC2500_GDO0_GPIO;
  extiIo.pin = CC2500_GDO0_PIN;
  spiconfig.extiIoTag = &extiIo;

#ifdef CC2500_RX_LED
  ledIo.port = LED_GPIO(CC2500_RX_LED);
  ledIo.pin = LED_GPIO_PIN(CC2500_RX_LED);
  ledIo.hi = LED_GPIO_ON(CC2500_RX_LED);
  ledIo.lo = LED_GPIO_OFF(CC2500_RX_LED);
  spiconfig.ledIoTag = &ledIo;
#else
  (void) ledIo;
  spiconfig.ledIoTag = NULL;
#endif
  spiconfig.ledInversion = FALSE; // Handled by paparazzi LED_X_GPIO_ON|_OFF

#ifdef CC2500_BIND_BUTTON
  bindIo.port = BUTTON_GPIO(CC2500_BIND_BUTTON);
  bindIo.pin = BUTTON_PIN(CC2500_BIND_BUTTON);
  spiconfig.bindIoTag = &bindIo;
#else
  (void) bindIo;
  spiconfig.bindIoTag = NULL;
#endif

  // rxCc2500SpiConfig
  cc2500spiconfig.autoBind = CC2500_AUTOBIND;
  cc2500spiconfig.bindTxId[0] = 0;
  cc2500spiconfig.bindTxId[1] = 0;
  cc2500spiconfig.bindOffset = 0;
  memset(cc2500spiconfig.bindHopData, 0, sizeof(cc2500spiconfig.bindHopData));
  cc2500spiconfig.rxNum = 0;
  cc2500spiconfig.a1Source = FRSKY_SPI_A1_SOURCE_VBAT;
  cc2500spiconfig.chipDetectEnabled = TRUE;

  settings_init();
  cc2500_persistent_read();

  // telemetryConfig
  telemetryconfig.pidValuesAsTelemetry = FALSE;
  telemetryconfig.report_cell_voltage = FALSE;
}
