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

#include "state.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/gpio.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/electrical.h"

#include <stdbool.h>
#include <assert.h>
#include <math.h>


// (unknown):
struct attitude_t bf_attitude = { { 0, 0, 0 } }; // Dummy values


// main/config/config.h:
struct pidProfile_s *currentPidProfile; // Dummy values


// main/config/feature.h:
bool bf_featureIsEnabled(const uint32_t mask) {
  uint32_t features = FEATURE_RX_SPI | FEATURE_TELEMETRY;
  return features & mask;
}


// main/common/filters.h:
#define M_PI_FLOAT  3.14159265358979323846f
float pt1FilterGain(float f_cut, float dT)
{
    float RC = 1 / ( 2 * M_PI_FLOAT * f_cut);
    return dT / (RC + dT);
}

void pt1FilterInit(pt1Filter_t *filter, float k) {
    filter->state = 0.0f;
    filter->k = k;
}

void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k) {
    filter->k = k;
}

float pt1FilterApply(pt1Filter_t *filter, float input) {
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}


// main/drivers/time.h:
void bf_delayMicroseconds(timeUs_t us) {
  sys_time_usleep(us);
}

void bf_delay(timeMs_t ms) {
  bf_delayMicroseconds(ms * 1000);
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
  io->hi(io->port, io->pin);
}

void bf_IOLo(IO_t io) {
  if (!io) return;
  io->lo(io->port, io->pin);
}

void bf_IOToggle(IO_t io) {
  if (!io) return;
  gpio_toggle(io->port, io->pin);
}


// main/fc/controlrate_profile.h:
controlRateConfig_t *currentControlRateProfile; // Dummy values


// main/flight/position.h:
int32_t bf_getEstimatedAltitudeCm(void) {
  return (int32_t)(stateGetPositionEnu_f()->z * 100.0);
}

int16_t bf_getEstimatedVario(void) {
  return (int16_t)(stateGetSpeedEnu_f()->z * 100.0);
}


// main/sensors/battery.h
bool bf_isBatteryVoltageConfigured(void) {
  return TRUE;
}

uint16_t bf_getLegacyBatteryVoltage(void) {
  return (uint16_t)((electrical.vsupply * 100.0 + 5) / 10.0); // ???
}

uint16_t bf_getBatteryVoltage(void) {
  return (uint16_t)(electrical.vsupply * 100.0); // 0.01V
}

uint8_t bf_getBatteryCellCount(void) {
  return 0;
}

bool bf_isAmperageConfigured(void) {
  return TRUE;
}

int32_t bf_getAmperage(void) {
  return (int32_t)(electrical.current * 100.0);
}

int32_t bf_getMAhDrawn(void) {
  return (int32_t)(electrical.charge * 1000.0);
}
