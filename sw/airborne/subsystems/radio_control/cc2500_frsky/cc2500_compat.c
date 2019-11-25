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

#include "cc2500_compat.h"

#include "mcu_periph/adc.h"
#include "mcu_periph/gpio.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/electrical.h"

#include <stdbool.h>
#include <assert.h>

// main/config/feature.h:
bool bf_featureIsEnabled(const uint32_t mask) {
  uint32_t features = 0;
  return features & mask;
}


// main/drivers/time.h:
void bf_delayMicroseconds(timeUs_t us) {
  assert(us <= UINT32_MAX);
  sys_time_usleep((uint32_t)us);
}

void bf_delay(timeMs_t ms) {
  bf_delayMicroseconds((uint64_t)ms * 1000);
}

timeUs_t bf_micros(void) {
  return get_sys_time_usec();
}

timeMs_t bf_millis(void) {
  return get_sys_time_msec();
}


// main/drivers/adc.h:
uint16_t bf_adcGetChannel(uint8_t channel) {
  (void) channel;
  // Return current, as voltage is already reported by getLegacyBatteryVoltage
  return (uint16_t)(electrical.current * 10000.0); // Assumed in 0.1mA
}


// main/drivers/rx_spi.h:
bool bf_rxSpiDeviceInit(void) {
  return TRUE;
}


// main/rx/rx.h:
static rxRuntimeState_t runtimeState;

rxRuntimeState_t* rxRuntimeState(void) {
  return &runtimeState;
}

rssiSource_e rssiSource;

void bf_setRssi(uint16_t rssiValue, rssiSource_e source) {
  (void) rssiValue;
  (void) source;
}

void bf_setRssiDirect(uint16_t newRssi, rssiSource_e source) {
  (void) newRssi;
  (void) source;
}


// main/drivers/io.h:
IO_t bf_IOGetByTag(ioTag_t io) {
  return (IO_t)io;
}

void bf_IOInit(IO_t io, uint8_t owner, uint8_t index) {
  (void) io;
  (void) owner;
  (void) index;
}

void bf_IOConfigGPIO(IO_t io, enum ioconfig_t cfg) {
  if (!io) return;
  switch(cfg) {
    case IOCFG_OUT_PP:
      gpio_setup_output(io->port, io->pin);
      break;
    case IOCFG_IN_FLOATING:
      gpio_setup_input(io->port, io->pin);
      break;
    case IOCFG_IPU:
      gpio_setup_input_pullup(io->port, io->pin);
      break;
    default:
      assert("Invalid IO config" == NULL);
      break;
  }
}

bool bf_IORead(IO_t gpio) {
  if (!gpio) return 0;
  return gpio_get(gpio->port, gpio->pin);
}

void bf_IOHi(IO_t io) {
  if (!io) return;
  gpio_set(io->port, io->pin);
}

void bf_IOLo(IO_t io) {
  if (!io) return;
  gpio_clear(io->port, io->pin);
}

void bf_IOToggle(IO_t io) {
  if (!io) return;
  gpio_toggle(io->port, io->pin);
}


// main/telemetry/smartport.h:
bool initSmartPortTelemetryExternal(smartPortWriteFrameFn *smartPortWriteFrameExternal) {
  (void) smartPortWriteFrameExternal;
  return FALSE;
}

smartPortPayload_t *smartPortDataReceive(uint16_t c, bool *clearToSend, smartPortReadyToSendFn *checkQueueEmpty, bool withChecksum) {
  (void) c;
  (void) clearToSend;
  (void) checkQueueEmpty;
  (void) withChecksum;
  return NULL;
}

void processSmartPortTelemetry(smartPortPayload_t *payload, volatile bool *hasRequest, const uint32_t *requestTimeout) {
  (void) payload;
  (void) hasRequest;
  (void) requestTimeout;
}


// main/sensors/battery.h
uint16_t bf_getLegacyBatteryVoltage(void) {
  return (uint16_t)(electrical.vsupply * 100.0); // 0.01V
}

