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

#include "modules/datalink/downlink.h" // TODO remove before PR

#include "cc2500.h"

#include "generated/modules.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/spi.h"

#include <assert.h>

#define USE_RX_CC2500

static struct spi_periph *cc2500_spi_p;
static struct spi_transaction cc2500_spi_t;
static uint8_t cc2500_input_buf[64];
static uint8_t cc2500_output_buf[64];


void cc2500_init(void) {
  /* Set spi peripheral */
  cc2500_spi_p = &(CC2500_SPI_DEV);
  /* Set the spi transaction */
  cc2500_spi_t.input_buf = cc2500_input_buf;
  cc2500_spi_t.output_buf = cc2500_output_buf;
  cc2500_spi_t.input_length = 0;
  cc2500_spi_t.output_length = 0;
  cc2500_spi_t.select = SPISelectUnselect;
  cc2500_spi_t.cpol = SPICpolIdleLow;
  cc2500_spi_t.cpha = SPICphaEdge1;
  cc2500_spi_t.dss = SPIDss8bit;
  cc2500_spi_t.bitorder = SPIMSBFirst;
  cc2500_spi_t.cdiv = SPIDiv32; // Found experimentally
  cc2500_spi_t.status = SPITransDone;
  cc2500_spi_t.slave_idx = CC2500_SPI_SLAVE_IDX;
}


static void cc2500_delayMicroseconds(uint32_t us) {
  float start = get_sys_time_float();
  while(get_sys_time_float() < start + (us / 1.0e6)) ;
}
// Fix naming conflict with modules/radio_control/cc2500_frsky delayMicroseconds
#ifdef delayMicroseconds
#undef delayMicroseconds
#endif
#define delayMicroseconds(us) cc2500_delayMicroseconds(us)


static void rxSpiReadCommandMulti(uint8_t command, uint8_t commandData, uint8_t *retData, uint8_t length) {
  // Check that the SPI transaction is not busy
  assert(cc2500_spi_t.status == SPITransDone);
  // Set up the transaction
  cc2500_spi_t.output_length = 2; // command + commandData
  cc2500_spi_t.input_length = length + 1; // STATUS BYTE + RETURN DATA
  cc2500_spi_t.output_buf[0] = command;
  cc2500_spi_t.output_buf[1] = commandData;
  // Submit the transaction
  spi_submit(cc2500_spi_p, &cc2500_spi_t);
  // Spin until transaction is completed
  while(cc2500_spi_t.status != SPITransSuccess) ; // TODO not ideal in event function...
  cc2500_spi_t.status = SPITransDone;
  // Copy the input buffer
  for (uint8_t i = 0; i < length; ++i) {
    retData[i] = cc2500_spi_t.input_buf[i + 1]; // Skips status byte, betaflight code does not work when this byte is included.
  }
}

static uint8_t rxSpiReadCommand(uint8_t command, uint8_t commandData) {
  uint8_t retData; // DATA
  rxSpiReadCommandMulti(command, commandData, &retData, 1);
  return retData;
}


static void rxSpiWriteCommandMulti(uint8_t command, uint8_t *data, uint8_t length) {
  // Check that the SPI transaction is not busy
  assert(cc2500_spi_t.status == SPITransDone);
  // Set up the transaction
  cc2500_spi_t.output_length = length + 1; // command + data[length]
  cc2500_spi_t.input_length = 0;
  cc2500_spi_t.output_buf[0] = command;
  // Copy the data to the output buffer
  for (uint8_t i = 0; i < length; ++i) {
    cc2500_spi_t.output_buf[i + 1] = data[i];
  }
  // Submit the transaction
  spi_submit(cc2500_spi_p, &cc2500_spi_t);
  // Spin until transaction is completed
  while(cc2500_spi_t.status != SPITransSuccess) ; // TODO not ideal in event function...
  cc2500_spi_t.status = SPITransDone;
}

static void rxSpiWriteCommand(uint8_t command, uint8_t data) {
  rxSpiWriteCommandMulti(command, &data, 1);
}

static void rxSpiWriteByte(uint8_t data) {
  rxSpiWriteCommandMulti(data, NULL, 0);
}



// betaflight/src/main/drivers/rx/rx_cc2500.c @  0a16f4d on Oct 1, 2018

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

/*
* CC2500 SPI drivers
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

//#include "platform.h"

#ifdef USE_RX_CC2500

//#include "build/build_config.h"
//
//#include "pg/rx.h"
//#include "pg/rx_spi.h"
//
//#include "drivers/io.h"
//#include "drivers/rx/rx_spi.h"
//#include "drivers/system.h"
//#include "drivers/time.h"

//#include "rx_cc2500.h"

#define NOP 0xFF

void cc2500ReadFifo(uint8_t *dpbuffer, uint8_t len)
{
    rxSpiReadCommandMulti(CC2500_3F_RXFIFO | CC2500_READ_BURST, NOP, dpbuffer, len);
//    DOWNLINK_SEND_CC2500_PACKET(DefaultChannel, DefaultDevice,len, dpbuffer);
}

void cc2500WriteFifo(uint8_t *dpbuffer, uint8_t len)
{
    cc2500Strobe(CC2500_SFTX); // 0x3B SFTX
    rxSpiWriteCommandMulti(CC2500_3F_TXFIFO | CC2500_WRITE_BURST,
                                 dpbuffer, len);
    cc2500Strobe(CC2500_STX); // 0x35
}

void cc2500ReadRegisterMulti(uint8_t address, uint8_t *data, uint8_t length)
{
    rxSpiReadCommandMulti(address, NOP, data, length);
}

void cc2500WriteRegisterMulti(uint8_t address, uint8_t *data,
                                  uint8_t length)
{
    rxSpiWriteCommandMulti(address, data, length);
}

uint8_t cc2500ReadReg(uint8_t reg)
{
    return rxSpiReadCommand(reg | 0x80, NOP);
}

void cc2500Strobe(uint8_t address) { rxSpiWriteByte(address); }

void cc2500WriteReg(uint8_t address, uint8_t data)
{
    rxSpiWriteCommand(address, data);
}

void cc2500SetPower(uint8_t power)
{
    const uint8_t patable[8] = {
        0xC5, // -12dbm
        0x97, // -10dbm
        0x6E, // -8dbm
        0x7F, // -6dbm
        0xA9, // -4dbm
        0xBB, // -2dbm
        0xFE, // 0dbm
        0xFF  // 1.5dbm
    };
    if (power > 7)
        power = 7;
    cc2500WriteReg(CC2500_3E_PATABLE, patable[power]);
}

uint8_t cc2500Reset(void)
{
    cc2500Strobe(CC2500_SRES);
    delayMicroseconds(1000); // 1000us
    // CC2500_SetTxRxMode(TXRX_OFF);
    // RX_EN_off;//off tx
    // TX_EN_off;//off rx
    return cc2500ReadReg(CC2500_0E_FREQ1) == 0xC4; // check if reset
}
#endif
