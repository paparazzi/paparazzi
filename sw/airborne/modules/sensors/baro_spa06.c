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
 */

#include "baro_spa06.h"

#include "modules/core/abi.h"
#include "mcu_periph/sys_time.h"
#include "math/pprz_isa.h"

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

/** The barometer driver instance */
struct spa06_t baro_spa06;

/**
 * @brief Bind the driver to the configured bus
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
 * TEMPERATURE [deg Celcius], updates the baro_spa06_alt globals (including the ISA
 * pressure altitude) and clears data_available for the next sample.
 */
void baro_spa06_event(void)
{
  spa06_event(&baro_spa06);

  if (baro_spa06.data_available) {
    uint32_t now_ts = get_sys_time_usec();
    // send ABI message
    AbiSendMsgBARO_ABS_RAW(BARO_SPA_SENDER_ID, now_ts, baro_spa06.pressure);
    AbiSendMsgTEMPERATURE_RAW(BARO_SPA_SENDER_ID, now_ts, baro_spa06.temperature, TEMPERATURE_TYPE_BARO);
    baro_spa06.data_available = false;
    baro_spa06_alt = pprz_isa_altitude_of_pressure(baro_spa06.pressure);
    baro_spa06_alt_valid = true;
  }
}
