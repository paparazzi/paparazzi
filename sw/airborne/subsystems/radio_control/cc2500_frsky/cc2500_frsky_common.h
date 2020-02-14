#include "cc2500_compat.h"
#include "cc2500_settings.h"

// betaflight/src/main/rx/cc2500_frsky_common.h  @ 766c90b
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

//#include "rx/rx_spi.h"

bool frSkySpiInit(const rxSpiConfig_t *rxSpiConfig, rxRuntimeConfig_t *rxRuntimeConfig);
rx_spi_received_e frSkySpiDataReceived(uint8_t *packet);
rx_spi_received_e frSkySpiProcessFrame(uint8_t *packet);
void frSkySpiSetRcData(uint16_t *rcData, const uint8_t *payload);
