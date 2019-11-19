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

/*
 * This file contains compatibility code for the betaflight cc2500 frsky
 * drivers.
 *
 * Notes:
 * - Function macros are used so that betaflight names can be used when this
 *   header is included, while the actual functions can have a bf_ prefix to
 *   prevent possible name collisions.
 */

#ifndef RADIO_CONTROL_CC2500_COMPAT_H
#define RADIO_CONTROL_CC2500_COMPAT_H


#pragma GCC diagnostic ignored "-Wmissing-prototypes"
#pragma GCC diagnostic ignored "-Wstrict-prototypes"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define USE_RX_SPI
#define USE_RX_FRSKY_SPI


// main/common/time.h:
typedef int32_t timeDelta_t;
typedef uint32_t timeMs_t ;
typedef uint64_t timeUs_t;
#define TIMEUS_MAX UINT64_MAX


// main/drivers/time.h:
void bf_delayMicroseconds(timeUs_t us);
#define delayMicroseconds(us) bf_delayMicroseconds(us)

void bf_delay(timeMs_t ms);
#define delay(ms) bf_delay(ms)

timeMs_t bf_millis(void);
#define millis() bf_millis()


// main/rx/rx.h:
typedef struct rxRuntimeState_s {
//    rxProvider_t        rxProvider;
//    SerialRXType        serialrxProvider;
    uint8_t             channelCount; // number of RC channels as reported by current input driver
//    uint16_t            rxRefreshRate;
//    rcReadRawDataFnPtr  rcReadRawFn;
//    rcFrameStatusFnPtr  rcFrameStatusFn;
//    rcProcessFrameFnPtr rcProcessFrameFn;
//    uint16_t            *channelData;
//    void                *frameData;
} rxRuntimeState_t;

rxRuntimeState_t* rxRuntimeState(void);

typedef enum {
    RSSI_SOURCE_NONE = 0,
    RSSI_SOURCE_ADC,
    RSSI_SOURCE_RX_CHANNEL,
    RSSI_SOURCE_RX_PROTOCOL,
    RSSI_SOURCE_MSP,
    RSSI_SOURCE_FRAME_ERRORS,
    RSSI_SOURCE_RX_PROTOCOL_CRSF,
} rssiSource_e;
extern rssiSource_e rssiSource;

void bf_setRssi(uint16_t rssiValue, rssiSource_e source);
#define setRssi(rssiValue, source) bf_setRssi(rssiValue, source)


// main/rx/rx_spi.h:
typedef enum {
    RX_SPI_FRSKY_D,
    RX_SPI_FRSKY_X,
    RX_SPI_FRSKY_X_LBT,
    RX_SPI_PROTOCOL_COUNT
} rx_spi_protocol_e;

typedef enum {
    RX_SPI_RECEIVED_NONE = 0,
    RX_SPI_RECEIVED_BIND = (1 << 0),
    RX_SPI_RECEIVED_DATA = (1 << 1),
    RX_SPI_ROCESSING_REQUIRED = (1 << 2),
} rx_spi_received_e;


// main/drivers/io.h:
struct gpio_t {
  uint32_t port;
  uint16_t pin;
};
typedef struct gpio_t *IO_t;
typedef IO_t ioTag_t;
#define IO_NONE NULL

IO_t bf_IOGetByTag(ioTag_t io);
#define IOGetByTag(io) bf_IOGetByTag(io)

void bf_IOInit(IO_t io, uint8_t owner, uint8_t index);
#define IOInit(io, owner, index) bf_IOInit(io, owner, index)

enum ioconfig_t {
  IOCFG_OUT_PP,
  IOCFG_IN_FLOATING,
  IOCFG_IPU,
};
void bf_IOConfigGPIO(IO_t io, enum ioconfig_t cfg);
#define IOConfigGPIO(io, cfg) bf_IOConfigGPIO(io, cfg)

bool bf_IORead(IO_t gpio);
#define IORead(gpio) bf_IORead(gpio)

void bf_IOHi(IO_t io);
#define IOHi(io) bf_IOHi(io)
void bf_IOLo(IO_t io);
#define IOLo(io) bf_IOLo(io)
void bf_IOToggle(IO_t io);
#define IOToggle(io) bf_IOToggle(io)


// main/drivers/resources.h:
typedef enum {
    OWNER_RX_SPI_EXTI,
    OWNER_RX_SPI_BIND,
    OWNER_LED,
} resourceOwner_e;

#endif // RADIO_CONTROL_CC2500_COMPAT_H
