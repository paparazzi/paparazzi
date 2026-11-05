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
 * @file modules/sensors/baro_spa06.h
 * @brief Module glue for the Goertek SPA06-003 / SPL06-001 barometer
 *
 * Publishes pressure and temperature as ABI messages (BARO_SPA_SENDER_ID);
 * see baro_spa06.c for details. Configure the bus with SPA06_USE_SPI,
 * SPA06_DEV and SPA06_SLAVE_ADDR / SPA06_SLAVE_IDX.
 */

#ifndef BARO_SPA06_H
#define BARO_SPA06_H

#include "peripherals/spa06.h"

/** The barometer driver instance, exposed for debugging/telemetry */
extern struct spa06_t baro_spa06;

extern float baro_spa06_alt;       ///< ISA pressure altitude of the last sample [m]
extern bool baro_spa06_alt_valid;  ///< true once the first valid sample has been processed

/** @brief Bind the driver to the configured bus and register telemetry (module init hook) */
void baro_spa06_init(void);
/** @brief Drive the sensor state machine (module periodic hook) */
void baro_spa06_periodic(void);
/** @brief Process finished bus transactions and publish new measurements (module event hook) */
void baro_spa06_event(void);

#endif
