/*
 * Copyright (C) 2026 OpenUAS
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
 * @file peripherals/dps310_i2c.h
 * @brief Driver for the Infineon DPS310 barometer (I2C)
 */

#ifndef DPS310_I2C_H
#define DPS310_I2C_H

#include "peripherals/dps310_regs.h"
#include "mcu_periph/i2c.h"

/**
 * @brief States of the non-blocking driver state machine
 *
 * Traversed once in order during initialization, then the driver stays in
 * DPS310_STATUS_READ_DATA. Any failure path leads back to DPS310_STATUS_UNINIT.
 */
enum Dps310Status {
  DPS310_STATUS_UNINIT,         ///< restart point: reset driver state, then detect the sensor
  DPS310_STATUS_GET_ID,         ///< read the product/revision ID register
  DPS310_STATUS_TEMP_FIX,       ///< apply the 5-write temperature errata fix (genuine DPS310 only)
  DPS310_STATUS_WAIT_RDY,       ///< wait for SENSOR_RDY and COEF_RDY (~40ms after power-up)
  DPS310_STATUS_GET_COEF_SRCE,  ///< read which temperature sensor the factory calibration used
  DPS310_STATUS_GET_CALIB,      ///< read the 18 calibration coefficient bytes
  DPS310_STATUS_CONFIGURE_REGS, ///< write PRS_CFG, TMP_CFG, MEAS_CFG (idle) and CFG_REG in one burst
  DPS310_STATUS_CONFIGURE_MEAS, ///< enable continuous pressure + temperature measurements
  DPS310_STATUS_READ_DATA       ///< operational: poll the 6 result registers
};

struct Dps310_I2c {
  struct i2c_periph *i2c_p;           ///< I2C peripheral used for communication
  struct i2c_transaction i2c_trans;   ///< I2C transaction (only one in flight at a time)
  enum Dps310Status status;           ///< state machine status
  bool initialized;                   ///< config done flag
  bool is_broken;                     ///< sensor not found or persistently failing, retried after a backoff period
  uint8_t init_error_cnt;             ///< number of consecutive transaction failures
  uint32_t timer;                     ///< broken-sensor retry backoff timer
  volatile bool data_available;       ///< data ready flag
  struct dps310_reg_calib_data calib; ///< calibration data
  uint8_t temp_coef_srce;             ///< TMP_COEF_SRCE bit read from the sensor, mirrored into TMP_CFG bit 7
  uint8_t temp_fix_step;              ///< current step of the temperature errata fix sequence
  int32_t raw_pressure;               ///< uncompensated pressure
  int32_t raw_temperature;            ///< uncompensated temperature
  float pressure;                     ///< pressure in Pascal
  float temperature;                  ///< temperature in deg Celsius
};

extern void dps310_i2c_init(struct Dps310_I2c *dps, struct i2c_periph *i2c_p, uint8_t addr);
extern void dps310_i2c_periodic(struct Dps310_I2c *dps);
extern void dps310_i2c_event(struct Dps310_I2c *dps);

#endif /* DPS310_I2C_H */
