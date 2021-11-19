/*
 * Copyright (C) Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/range_finder/cf_deck_multi_ranger.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Multi-ranger deck from Bitcraze for Crazyflie drones
 */

#include "modules/range_finder/cf_deck_multi_ranger.h"
#include "peripherals/pca95xx.h"
#include "peripherals/vl53l1x_nonblocking.h"
#include "peripherals/vl53l1x_api.h"
#include "modules/core/abi.h"
#include "modules/datalink/downlink.h"

// By default, do early init to be compatible with the flow_deck
// Blocking i2c is only possible with ChibiOS
// This module should be called before other modules using the VL53L1X sensor
// without the possibility to turn off the sensor (i.e. flow_deck)
#ifndef MULTI_RANGER_EARLY_INIT
#define MULTI_RANGER_EARLY_INIT TRUE
#endif

/* VL53L1X configuration */
// Time budget for single measurement
// Allowed values: 15, 20, 33, 50, 100, 200, 500
// see VL53L1X_SetTimingBudgetInMs
#ifndef MULTI_RANGER_TIMINGBUDGET_MS
#define MULTI_RANGER_TIMINGBUDGET_MS 100
#endif

// Allowed values: 1 (short, max ~1.3m), 2 (long, max ~4m)
// see VL53L1X_SetDistanceMode
#ifndef MULTI_RANGER_DISTANCEMODE
#define MULTI_RANGER_DISTANCEMODE 2
#endif

// Time between measurements
// Should be larger than or equal to timing budget
// see VL53L1X_SetInterMeasurementInMs
// Note: may be limited by module periodic frequency
#ifndef MULTI_RANGER_INTERMEASUREMENT_MS
#define MULTI_RANGER_INTERMEASUREMENT_MS MULTI_RANGER_TIMINGBUDGET_MS
#endif
#if MULTI_RANGER_INTERMEASUREMENT_MS < MULTI_RANGER_TIMINGBUDGET_MS
#warning MULTI_RANGER_INTERMEASUREMENT_MS should be greater than or equal to MULTI_RANGER_TIMINGBUDGET_MS
#endif

// PCA I/O pins to enable sensors
#define MULTI_RANGER_PIN_FRONT  PCA95XX_P4
#define MULTI_RANGER_PIN_BACK   PCA95XX_P1
#define MULTI_RANGER_PIN_RIGHT  PCA95XX_P2
#define MULTI_RANGER_PIN_LEFT   PCA95XX_P6
#define MULTI_RANGER_PIN_UP     PCA95XX_P0
#define MULTI_RANGER_PIN_ALL    (MULTI_RANGER_PIN_FRONT | MULTI_RANGER_PIN_BACK | MULTI_RANGER_PIN_RIGHT | MULTI_RANGER_PIN_LEFT | MULTI_RANGER_PIN_UP)

enum MultiRangerStatus {
  MULTI_RANGER_UNINIT,
  MULTI_RANGER_CONF_IO,
#ifdef MULTI_RANGER_EXTRA_DEV
  MULTI_RANGER_CONF_EXTRA,
#endif
  MULTI_RANGER_CONF_FRONT,
  MULTI_RANGER_CONF_BACK,
  MULTI_RANGER_CONF_RIGHT,
  MULTI_RANGER_CONF_LEFT,
  MULTI_RANGER_CONF_UP,
  MULTI_RANGER_READ_FRONT,
  MULTI_RANGER_READ_BACK,
  MULTI_RANGER_READ_RIGHT,
  MULTI_RANGER_READ_LEFT,
  MULTI_RANGER_READ_UP
};

enum MultiRangerDev {
  MULTI_RANGER_FRONT = 0,
  MULTI_RANGER_BACK,
  MULTI_RANGER_RIGHT,
  MULTI_RANGER_LEFT,
  MULTI_RANGER_UP,
  MULTI_RANGER_NB
};

// Default orientation (azimuth, bearing) in rad of sensors relative to body
#ifndef MULTI_RANGER_ARRAY_ORIENTATION
#define MULTI_RANGER_ARRAY_ORIENTATION {{0.f, 0.f}, {0.f, M_PI}, {0.f, M_PI_2}, {0.f, -M_PI_2}, {M_PI_2, 0.f}}
#endif
static const float multi_ranger_array_orientation[][2] = MULTI_RANGER_ARRAY_ORIENTATION;

struct SingleRanger {
  VL53L1_Dev_t dev;     ///< sensor driver
  float distance;       ///< raw distance measurement
  float azimuth;        ///< azimuth [rad] relative to body frame
  float bearing;        ///< bearing [rad] relative to body frame
  uint8_t read_state;   ///< current reading state
};

struct cf_deck_multi_ranger {
  enum MultiRangerStatus status;
  // VL53L1X devices
  struct SingleRanger ranger[MULTI_RANGER_NB];    ///< sensor array
  // I/O expander
  struct pca95xx pca;
};

static struct cf_deck_multi_ranger multi_ranger;

/**
 * Boot a device
 */
static void multi_ranger_boot_device(VL53L1_Dev_t *dev UNUSED)
{
#ifndef SITL
  VL53L1X_BootDevice(dev, MULTI_RANGER_TIMINGBUDGET_MS, MULTI_RANGER_DISTANCEMODE, MULTI_RANGER_INTERMEASUREMENT_MS);
#endif
}

/**
 * Module init
 */
void multi_ranger_init(void)
{
  multi_ranger.status = MULTI_RANGER_UNINIT;

  // init I/O expander
  pca95xx_init(&multi_ranger.pca, &(MULTI_RANGER_I2C_DEV), PCA95XX_DEFAULT_ADDRESS);
#if MULTI_RANGER_EARLY_INIT
  pca95xx_configure(&multi_ranger.pca, ~(MULTI_RANGER_PIN_ALL), true); // configure output
  pca95xx_set_output(&multi_ranger.pca, ~(MULTI_RANGER_PIN_ALL), true); // select none
#endif

  // init vl53l1x array
  for (uint8_t i = 0; i < MULTI_RANGER_NB; i++) {
    multi_ranger.ranger[i].dev.i2c_p = &(MULTI_RANGER_I2C_DEV);
    multi_ranger.ranger[i].dev.i2c_trans.slave_addr = VL53L1_DEFAULT_ADDRESS;
    multi_ranger.ranger[i].dev.read_status = VL53L1_READ_IDLE;
    multi_ranger.ranger[i].distance = 0.f;
    multi_ranger.ranger[i].azimuth = multi_ranger_array_orientation[i][0];
    multi_ranger.ranger[i].bearing = multi_ranger_array_orientation[i][1];
  }
}

/**
 * Read data from a device
 * @return true when read cycle is finished
 */
static bool multi_ranger_read(struct SingleRanger *ranger UNUSED)
{
#ifndef SITL
  uint16_t range_mm;
  bool new_data = false;
  bool ret = VL53L1X_NonBlocking_ReadDataEvent(&ranger->dev, &range_mm, &new_data);
  if (new_data) {
    ranger->distance = range_mm / 1000.f;
    AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_MULTI_RANGER_DECK_ID, ranger->distance, ranger->azimuth, ranger->bearing);
  }
  return ret;
#endif
}

/**
 * Module periodic function
 */
void multi_ranger_periodic(void)
{
  switch (multi_ranger.status) {
    case MULTI_RANGER_UNINIT:
      pca95xx_configure(&multi_ranger.pca, ~(MULTI_RANGER_PIN_ALL), false); // configure output
      multi_ranger.status++;
      break;
    case MULTI_RANGER_CONF_IO:
      pca95xx_set_output(&multi_ranger.pca, MULTI_RANGER_PIN_FRONT, false); // select front
      multi_ranger.status++;
      break;
    case MULTI_RANGER_CONF_FRONT:
      multi_ranger_boot_device(&multi_ranger.ranger[MULTI_RANGER_FRONT].dev);
      pca95xx_set_output(&multi_ranger.pca, MULTI_RANGER_PIN_FRONT | MULTI_RANGER_PIN_BACK, false); // select back
      multi_ranger.status++;
      break;
    case MULTI_RANGER_CONF_BACK:
      multi_ranger_boot_device(&multi_ranger.ranger[MULTI_RANGER_BACK].dev);
      pca95xx_set_output(&multi_ranger.pca, MULTI_RANGER_PIN_FRONT | MULTI_RANGER_PIN_BACK | MULTI_RANGER_PIN_RIGHT, false); // select right
      multi_ranger.status++;
      break;
    case MULTI_RANGER_CONF_RIGHT:
      multi_ranger_boot_device(&multi_ranger.ranger[MULTI_RANGER_RIGHT].dev);
      pca95xx_set_output(&multi_ranger.pca, MULTI_RANGER_PIN_FRONT | MULTI_RANGER_PIN_BACK | MULTI_RANGER_PIN_RIGHT | MULTI_RANGER_PIN_LEFT, false); // select left
      multi_ranger.status++;
      break;
    case MULTI_RANGER_CONF_LEFT:
      multi_ranger_boot_device(&multi_ranger.ranger[MULTI_RANGER_LEFT].dev);
      pca95xx_set_output(&multi_ranger.pca, MULTI_RANGER_PIN_ALL, false); // select up
      multi_ranger.status++;
      break;
    case MULTI_RANGER_CONF_UP:
      multi_ranger_boot_device(&multi_ranger.ranger[MULTI_RANGER_UP].dev);
      multi_ranger.status++;
      break;
    case MULTI_RANGER_READ_FRONT:
      if (VL53L1X_NonBlocking_IsIdle(&multi_ranger.ranger[MULTI_RANGER_FRONT].dev)) {
        VL53L1X_NonBlocking_RequestData(&multi_ranger.ranger[MULTI_RANGER_FRONT].dev);
        multi_ranger.status++;
      }
      break;
    case MULTI_RANGER_READ_BACK:
      if (VL53L1X_NonBlocking_IsIdle(&multi_ranger.ranger[MULTI_RANGER_BACK].dev)) {
        VL53L1X_NonBlocking_RequestData(&multi_ranger.ranger[MULTI_RANGER_BACK].dev);
        multi_ranger.status++;
      }
      break;
    case MULTI_RANGER_READ_RIGHT:
      if (VL53L1X_NonBlocking_IsIdle(&multi_ranger.ranger[MULTI_RANGER_RIGHT].dev)) {
        VL53L1X_NonBlocking_RequestData(&multi_ranger.ranger[MULTI_RANGER_RIGHT].dev);
        multi_ranger.status++;
      }
      break;
    case MULTI_RANGER_READ_LEFT:
      if (VL53L1X_NonBlocking_IsIdle(&multi_ranger.ranger[MULTI_RANGER_LEFT].dev)) {
        VL53L1X_NonBlocking_RequestData(&multi_ranger.ranger[MULTI_RANGER_LEFT].dev);
        multi_ranger.status++;
      }
      break;
    case MULTI_RANGER_READ_UP:
      if (VL53L1X_NonBlocking_IsIdle(&multi_ranger.ranger[MULTI_RANGER_UP].dev)) {
        VL53L1X_NonBlocking_RequestData(&multi_ranger.ranger[MULTI_RANGER_UP].dev);
        multi_ranger.status = MULTI_RANGER_READ_FRONT;
      }
      break;
    default:
      break;
  }
}

void multi_ranger_event(void)
{
  // call non blocking read/event functions
  multi_ranger_read(&multi_ranger.ranger[MULTI_RANGER_FRONT]);
  multi_ranger_read(&multi_ranger.ranger[MULTI_RANGER_BACK]);
  multi_ranger_read(&multi_ranger.ranger[MULTI_RANGER_RIGHT]);
  multi_ranger_read(&multi_ranger.ranger[MULTI_RANGER_LEFT]);
  multi_ranger_read(&multi_ranger.ranger[MULTI_RANGER_UP]);
}

void multi_ranger_report(void)
{
  float dist_array[MULTI_RANGER_NB];
  for (int i = 0; i < MULTI_RANGER_NB; i++) {
    dist_array[i] = multi_ranger.ranger[i].distance;
  }
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, MULTI_RANGER_NB, dist_array);
}

