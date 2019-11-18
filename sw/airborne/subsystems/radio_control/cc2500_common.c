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

#include "cc2500_common.h"

#include "subsystems/radio_control.h"
#include "mcu_periph/gpio.h"
#include "peripherals/cc2500.h"
#include <assert.h>

#include <string.h>
#include "subsystems/datalink/downlink.h"


// Compatibility code

#define USE_RX_FRSKY_SPI

#define OWNER_RX_SPI_EXTI 0
static uint16_t rssiSource;

struct gpio_t {
  uint32_t port;
  uint16_t pin;
};
typedef struct gpio_t *IO_t;

static void IOInit(IO_t io, uint8_t owner, uint8_t index);
static void IOInit(IO_t io, uint8_t owner, uint8_t index) {
  (void) io;
  (void) owner;
  (void) index;
}

#define IOCFG_IN_FLOATING 0
static void IOConfigGPIO(IO_t io, uint8_t cfg) {
  assert(cfg == IOCFG_IN_FLOATING);
  gpio_setup_input(io->port, io->pin);
}

static bool IORead(IO_t gpio) {
  return gpio_get(gpio->port, gpio->pin);
}


typedef enum {
    RSSI_SOURCE_NONE = 0,
    RSSI_SOURCE_ADC,
    RSSI_SOURCE_RX_CHANNEL,
    RSSI_SOURCE_RX_PROTOCOL,
    RSSI_SOURCE_MSP,
    RSSI_SOURCE_FRAME_ERRORS,
    RSSI_SOURCE_RX_PROTOCOL_CRSF,
} rssiSource_e;

static void setRssi(uint16_t rssiValue, rssiSource_e source) {
  (void) rssiValue;
  (void) source;
}


struct cc2500_settings_t {
  bool chipDetectEnabled;
};
static struct cc2500_settings_t cc2500_settings;

static struct cc2500_settings_t* rxCc2500SpiConfig(void);
static struct cc2500_settings_t* rxCc2500SpiConfig(void) {
  return &cc2500_settings;
}


struct cc2500_spiconfig_t {
  struct gpio_t extiIoTag_gpio;
  IO_t extiIoTag;
};
static struct cc2500_spiconfig_t cc2500_spiconfig;

static struct cc2500_spiconfig_t* rxSpiConfig(void);
static struct cc2500_spiconfig_t* rxSpiConfig(void) {
  return &cc2500_spiconfig;
}

static IO_t IOGetByTag(IO_t io) {
  return io;
}



// Paparazzi code

static uint32_t reset_value = 0;
static uint32_t counter = 0;

void radio_control_impl_init(void) {
  cc2500_settings.chipDetectEnabled = TRUE;
  cc2500_spiconfig.extiIoTag_gpio.port = CC2500_GDO0_GPIO_PORT;
  cc2500_spiconfig.extiIoTag_gpio.pin = CC2500_GDO0_GPIO;
  cc2500_spiconfig.extiIoTag = &(cc2500_spiconfig.extiIoTag_gpio);

  cc2500_init();
  reset_value = cc2500Reset();
}

void radio_control_impl_event(void (* _received_frame_handler)(void)) {
  (void) _received_frame_handler;
  counter++;
  if((counter % 10000) == 0) {
    DOWNLINK_SEND_CC2500(DefaultChannel, DefaultDevice,
        &reset_value, &counter, &counter, &counter);
  }
  if((counter % 100000) == 0) {
    static char text[] = "Hello GCS!";
    DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(text), text);
  }
}


// betaflight/src/main/rx/cc2500_common.c  @ 4a79046

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

#include <stdbool.h>

//#include "platform.h"

#if defined(USE_RX_FRSKY_SPI) || defined(USE_RX_SFHSS_SPI)

//#include "common/maths.h"
//
//#include "drivers/io.h"
//#include "drivers/rx/rx_cc2500.h"
//#include "drivers/time.h"
//
//#include "config/config.h"
//
//#include "pg/pg.h"
//#include "pg/pg_ids.h"
//#include "pg/rx.h"
//#include "pg/rx_spi.h"
//#include "pg/rx_spi_cc2500.h"
//
//#include "rx/rx.h"
//#include "rx/rx_spi.h"
//
//#include "cc2500_common.h"

static IO_t gdoPin;
#if defined(USE_RX_CC2500_SPI_PA_LNA)
static IO_t txEnPin;
static IO_t rxLnaEnPin;
#if defined(USE_RX_CC2500_SPI_DIVERSITY)
static IO_t antSelPin;
#endif
#endif
static int16_t rssiDbm;

uint16_t cc2500getRssiDbm(void)
{
    return rssiDbm;
}

void cc2500setRssiDbm(uint8_t value)
{
    if (value >= 128) {
        rssiDbm = ((((uint16_t)value) * 18) >> 5) - 82;
    } else {
        rssiDbm = ((((uint16_t)value) * 18) >> 5) + 65;
    }

    setRssi(rssiDbm << 3, RSSI_SOURCE_RX_PROTOCOL);
}

bool cc2500getGdo(void)
{
    return IORead(gdoPin);
}

#if defined(USE_RX_CC2500_SPI_PA_LNA) && defined(USE_RX_CC2500_SPI_DIVERSITY)
void cc2500switchAntennae(void)
{
    static bool alternativeAntennaSelected = true;

    if (antSelPin) {
        if (alternativeAntennaSelected) {
            IOLo(antSelPin);
        } else {
            IOHi(antSelPin);
        }
        alternativeAntennaSelected = !alternativeAntennaSelected;
    }
}
#endif
#if defined(USE_RX_CC2500_SPI_PA_LNA)
void cc2500TxEnable(void)
{
    if (txEnPin) {
        IOHi(txEnPin);
    }
}

void cc2500TxDisable(void)
{
    if (txEnPin) {
        IOLo(txEnPin);
    }
}
#endif

static bool cc2500SpiDetect(void)
{
    const uint8_t chipPartNum = cc2500ReadReg(CC2500_30_PARTNUM | CC2500_READ_BURST); //CC2500 read registers chip part num
    const uint8_t chipVersion = cc2500ReadReg(CC2500_31_VERSION | CC2500_READ_BURST); //CC2500 read registers chip version
    if (chipPartNum == 0x80 && chipVersion == 0x03) {
        return true;
    }

    return false;
}

bool cc2500SpiInit(void)
{
    if (rxCc2500SpiConfig()->chipDetectEnabled && !cc2500SpiDetect()) {
        return false;
    }

    // gpio init here
    gdoPin = IOGetByTag(rxSpiConfig()->extiIoTag);

    if (!gdoPin) {
        return false;
    }

    IOInit(gdoPin, OWNER_RX_SPI_EXTI, 0);
    IOConfigGPIO(gdoPin, IOCFG_IN_FLOATING);
#if defined(USE_RX_CC2500_SPI_PA_LNA)
    if (rxCc2500SpiConfig()->lnaEnIoTag) {
        rxLnaEnPin = IOGetByTag(rxCc2500SpiConfig()->lnaEnIoTag);
        IOInit(rxLnaEnPin, OWNER_RX_SPI_CC2500_LNA_EN, 0);
        IOConfigGPIO(rxLnaEnPin, IOCFG_OUT_PP);

        IOHi(rxLnaEnPin); // always on at the moment
    }
    if (rxCc2500SpiConfig()->txEnIoTag) {
        txEnPin = IOGetByTag(rxCc2500SpiConfig()->txEnIoTag);
        IOInit(txEnPin, OWNER_RX_SPI_CC2500_TX_EN, 0);
        IOConfigGPIO(txEnPin, IOCFG_OUT_PP);
    } else {
        txEnPin = IO_NONE;
    }
#if defined(USE_RX_CC2500_SPI_DIVERSITY)
    if (rxCc2500SpiConfig()->antSelIoTag) {
        antSelPin = IOGetByTag(rxCc2500SpiConfig()->antSelIoTag);
        IOInit(antSelPin, OWNER_RX_SPI_CC2500_ANT_SEL, 0);
        IOConfigGPIO(antSelPin, IOCFG_OUT_PP);

        IOHi(antSelPin);
    } else {
        antSelPin = IO_NONE;
    }
#endif
#endif // USE_RX_CC2500_SPI_PA_LNA

#if defined(USE_RX_CC2500_SPI_PA_LNA)
    cc2500TxDisable();
#endif // USE_RX_CC2500_SPI_PA_LNA

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }

    return true;
}
#endif

