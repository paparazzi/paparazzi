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

#ifndef RADIO_CONTROL_CC2500_SETTINGS_H
#define RADIO_CONTROL_CC2500_SETTINGS_H

#include "cc2500_compat.h"

#include <stdint.h>

/* Paparazzi settings */
void cc2500_settings_init(void);


/* betaflight settings API */
// main/config/config.h:
void bf_writeEEPROM(void);
#define writeEEPROM() bf_writeEEPROM()


// main/pg/rx_spi.h:
typedef struct rxSpiConfig_s {
    // RX protocol
    uint8_t rx_spi_protocol;                // type of SPI RX protocol
//                                            // nrf24: 0 = v202 250kbps. (Must be enabled by FEATURE_RX_NRF24 first.)
//    uint32_t rx_spi_id;
//    uint8_t rx_spi_rf_channel_count;
//
//    // SPI Bus
//    ioTag_t csnTag;
//    uint8_t spibus;
//
    ioTag_t bindIoTag;
    ioTag_t ledIoTag;
    uint8_t ledInversion;

    ioTag_t extiIoTag;
} rxSpiConfig_t;

const rxSpiConfig_t* rxSpiConfig(void);


// main/pg/rx_spi_cc2500.h:
typedef struct rxCc2500SpiConfig_s {
    uint8_t autoBind;
    uint8_t bindTxId[2];
    int8_t  bindOffset;
    uint8_t bindHopData[50];
    uint8_t rxNum;
//    uint8_t a1Source;
    uint8_t chipDetectEnabled;
//    ioTag_t txEnIoTag;
//    ioTag_t lnaEnIoTag;
//    ioTag_t antSelIoTag;
} rxCc2500SpiConfig_t;

const rxCc2500SpiConfig_t* rxCc2500SpiConfig(void);
rxCc2500SpiConfig_t* rxCc2500SpiConfigMutable(void);


#endif // RADIO_CONTROL_CC2500_SETTINGS_H
