/*
 * Copyright (C) 2026 OpenUAS
 * Thanks to Florian Sansou florian.sansou@enac.fr for initial implementation
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 * 
 */

/**
 * @file modules/sensors/baro_spa06.c
 * @brief Module glue for the Goertek SPA06-003 / SPL06-001 barometer
 *
 * Wires the spa06 peripheral driver (see peripherals/spa06.c) into the
 * module system: init/periodic/event hooks, and publishes each accepted
 * measurement as ABI BARO_ABS + TEMPERATURE messages (BARO_SPA_SENDER_ID).
 * Optionally reports over telemetry via the BMP_STATUS message, either
 * periodically (default) or per sample (SPA06_SYNC_SEND, debugging only).
 */

#include "baro_spa06.h"

#include "modules/core/abi.h"
#include "math/pprz_isa.h"
#ifdef SPA06_SYNC_SEND
  #include "mcu_periph/uart.h"
  #include "pprzlink/messages.h"
  #include "modules/datalink/downlink.h"
#endif

#if DOWNLINK && !defined(SPA06_SYNC_SEND)
  #include "modules/datalink/telemetry.h"
#endif
PRINT_CONFIG_VAR(SPA06_SYNC_SEND)

#ifndef SPA06_USE_SPI
  #define SPA06_USE_SPI FALSE
#endif
PRINT_CONFIG_VAR(SPA06_USE_SPI)
PRINT_CONFIG_VAR(SPA06_DEV)

/** Default I2C slave address (SDO low); override with SPA06_I2C_ADDR_ALT for SDO high */
#ifndef SPA06_SLAVE_ADDR
  #define SPA06_SLAVE_ADDR SPA06_I2C_ADDR
#endif

/** Default SPI slave select index, only used when SPA06_USE_SPI is TRUE */
#ifndef SPA06_SLAVE_IDX
  #define SPA06_SLAVE_IDX SPI_SLAVE0
#endif

float baro_spa06_alt = 0;              ///< ISA pressure altitude of the last sample [m]
bool baro_spa06_alt_valid = false;     ///< true once the first valid sample has been processed
static float baro_spa06_temp = 0;      ///< last sensor temperature [deg Celcius]
static float baro_spa06_press = 0;     ///< last compensated pressure [Pa]

/** The barometer driver instance */
struct spa06_t baro_spa06;

#if DOWNLINK && !defined(SPA06_SYNC_SEND)
/**
 * @brief Periodic telemetry: report the latest barometer data
 *
 * Re-uses the BMP_STATUS message since it has fitting fields:
 * raw pressure, raw temperature, pressure [Pa] and temperature [0.1 deg Celcius].
 */
static void send_baro_spa_data(struct transport_tx *trans, struct link_device *dev)
{
  int32_t up = baro_spa06.raw_pressure;
  int32_t ut = baro_spa06.raw_temperature;
  int32_t baro_pressure = (int32_t)(baro_spa06_press);           // Pa
  int32_t baro_temperature = (int32_t)(baro_spa06_temp * 10.0f); // 0.1 deg Celcius
  pprz_msg_send_BMP_STATUS(trans, dev, AC_ID, &up, &ut, &baro_pressure, &baro_temperature);
}
#endif

/**
 * @brief Bind the driver to the configured bus and register telemetry
 *
 * Bus selection is compile time: SPA06_USE_SPI picks SPI (SPA06_DEV +
 * SPA06_SLAVE_IDX) or I2C (SPA06_DEV + SPA06_SLAVE_ADDR). The sensor itself
 * is detected and configured asynchronously by the periodic/event pair.
 */
void baro_spa06_init(void)
{
#if SPA06_USE_SPI
  baro_spa06.bus = SPA06_SPI;
  baro_spa06.spi.p = &SPA06_DEV;
  baro_spa06.spi.slave_idx = SPA06_SLAVE_IDX;
#else
  baro_spa06.bus = SPA06_I2C;
  baro_spa06.i2c.p = (struct i2c_periph*)&SPA06_DEV;
  baro_spa06.i2c.slave_addr = SPA06_SLAVE_ADDR;
#endif
  spa06_init(&baro_spa06);
#if DOWNLINK && !defined(SPA06_SYNC_SEND)
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_BMP_STATUS, send_baro_spa_data);
#endif
}

/** @brief Drive the sensor state machine (module periodic hook, 25Hz) */
void baro_spa06_periodic(void)
{
  spa06_periodic(&baro_spa06);
}

/**
 * @brief Process finished bus transactions and publish new measurements
 *
 * When the driver flags a valid sample: sends ABI BARO_ABS [Pa] and
 * TEMPERATURE [deg Celcius], updates the baro_* globals (including the ISA
 * pressure altitude) and clears data_available for the next sample.
 */
void baro_spa06_event(void)
{
  spa06_event(&baro_spa06);

  if (baro_spa06.data_available) {
    uint32_t now_ts = get_sys_time_usec();
    // send ABI message
    AbiSendMsgBARO_ABS(BARO_SPA_SENDER_ID, now_ts, baro_spa06.pressure);
    AbiSendMsgTEMPERATURE(BARO_SPA_SENDER_ID, baro_spa06.temperature);
    baro_spa06.data_available = false;
    baro_spa06_alt = pprz_isa_altitude_of_pressure(baro_spa06.pressure);
    baro_spa06_alt_valid = true;
    baro_spa06_press = baro_spa06.pressure;      // Pa
    baro_spa06_temp = baro_spa06.temperature;    // deg Celcius

#if defined(SPA06_SYNC_SEND)
    int32_t up = (int32_t)(baro_spa06.raw_pressure);
    int32_t ut = (int32_t)(baro_spa06.raw_temperature);
    int32_t p = (int32_t) baro_spa06.pressure;
    int32_t t = (int32_t)(baro_spa06.temperature * 10.0f); // 0.1 deg Celcius
    DOWNLINK_SEND_BMP_STATUS(DefaultChannel, DefaultDevice, &up, &ut, &p, &t);
#endif
  }
}
