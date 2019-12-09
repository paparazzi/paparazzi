#include "cc2500_compat.h"

#include "cc2500_rx_spi.h"

// betaflight/src/main/rx/cc2500_common.h  @ 50bbe0b on Jul 1
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

uint16_t cc2500getRssiDbm(void);
void cc2500setRssiDbm(uint8_t value);
bool cc2500getGdo(void);
#if defined(USE_RX_CC2500_SPI_PA_LNA) && defined(USE_RX_CC2500_SPI_DIVERSITY)
void cc2500switchAntennae(void);
#endif
#if defined(USE_RX_CC2500_SPI_PA_LNA)
void cc2500TxEnable(void);
void cc2500TxDisable(void);
#endif
bool cc2500SpiInit(void);
