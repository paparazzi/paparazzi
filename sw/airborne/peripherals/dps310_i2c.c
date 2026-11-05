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
 * @file peripherals/dps310_i2c.c
 * @brief Driver for the Infineon DPS310 barometer (I2C)
 *
 * Non-blocking state machine driver:
 * @verbatim
 * UNINIT -> GET_ID -> (TEMP_FIX) -> WAIT_RDY -> GET_COEF_SRCE -> GET_CALIB
 *        -> CONFIGURE_REGS -> CONFIGURE_MEAS -> READ_DATA (looping)
 * @endverbatim
 *
 * dps310_i2c_periodic() submits at most one I2C transaction per call and
 * dps310_i2c_event() consumes the result and advances the state machine.
 * The sensor runs in continuous background mode (16 Hz pressure and
 * temperature at 16x oversampling); READ_DATA polls the latest results.
 */

#include "peripherals/dps310_i2c.h"
#include "mcu_periph/sys_time.h"

/* Reliability limits */
#define DPS310_BROKEN_RETRY_US   5000000   ///< retry a failed/absent sensor every 5s instead of giving up forever
#define DPS310_MAX_ERROR_CNT     10        ///< consecutive transaction failures before escalating
#define DPS310_PRESSURE_MIN_PA   30000.0f  ///< specified measurement range is 300-1200 hPa,
#define DPS310_PRESSURE_MAX_PA   120000.0f ///< anything outside means the data got corrupted

/* Compensation scale factors for 16x oversampling (datasheet table "Compensation Scale Factors").
 * Tied to the PM_PRC/TMP_PRC settings written in DPS310_STATUS_CONFIGURE_REGS: change them together. */
#define DPS310_SCALE_FACTOR_KT   253952.0f ///< temperature, 16x oversampling
#define DPS310_SCALE_FACTOR_KP   253952.0f ///< pressure, 16x oversampling

/* The 18-byte calibration read must fit in the I2C transaction buffer */
#if I2C_BUF_LEN < 18
#error "DPS310: I2C_BUF_LEN is too small for the 18 byte calibration coefficient read"
#endif

/**
 * @brief DPS310 temperature errata fix sequence
 *
 * Some DPS310 silicon revisions report temperature roughly 60 degC too high,
 * which also corrupts the temperature-compensated pressure. Writing this
 * specific sequence of undocumented register writes fixes the issue.
 * The register addresses are non-contiguous, so each {register, value} pair
 * must be written in its own I2C transaction.
 */
static const uint8_t dps310_temp_fix_seq[][2] = {
  { 0x0E, 0xA5 },
  { 0x0F, 0x96 },
  { 0x62, 0x02 },
  { 0x0E, 0x00 },
  { 0x0F, 0x00 }
};

#define DPS310_TEMP_FIX_SEQ_LEN (sizeof(dps310_temp_fix_seq) / sizeof(dps310_temp_fix_seq[0]))

/**
 * @brief Sign-extend a raw two's complement value of arbitrary bit length
 * @param raw    Unsigned register value holding the two's complement number
 * @param length Number of significant bits (e.g. 12, 16, 20, 24)
 * @return Sign-extended signed value
 */
static int32_t getTwosComplement(uint32_t raw, uint8_t length)
{
  if (raw & ((uint32_t)1 << (length - 1))) {
    return ((int32_t)raw) - ((int32_t)1 << length);
  }
  return raw;
}

/**
 * @brief Unpack the 18 calibration coefficient bytes (registers 0x10-0x21)
 * @param dps The dps310 instance, expects the coefficient block in the transaction buffer
 *
 * The coefficients are packed as mixed 12/16/20 bit two's complement values,
 * see the COEF register description in the datasheet.
 */
static void parse_calib_data(struct Dps310_I2c *dps)
{
  // Keep the volatile qualifier of the live transaction buffer instead of casting it away
  volatile uint8_t *coef = dps->i2c_trans.buf;

  dps->calib.c0 = getTwosComplement(((uint32_t)coef[0] << 4) | (((uint32_t)coef[1] >> 4) & 0x0F), 12);
  dps->calib.c1 = getTwosComplement((((uint32_t)coef[1] & 0x0F) << 8) | (uint32_t)coef[2], 12);
  dps->calib.c00 = getTwosComplement(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (((
                                       uint32_t)coef[5] >> 4) & 0x0F), 20);
  dps->calib.c10 = getTwosComplement((((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | (uint32_t)coef[7],
                                     20);
  dps->calib.c01 = getTwosComplement(((uint32_t)coef[8] << 8) | (uint32_t)coef[9], 16);
  dps->calib.c11 = getTwosComplement(((uint32_t)coef[10] << 8) | (uint32_t)coef[11], 16);
  dps->calib.c20 = getTwosComplement(((uint32_t)coef[12] << 8) | (uint32_t)coef[13], 16);
  dps->calib.c21 = getTwosComplement(((uint32_t)coef[14] << 8) | (uint32_t)coef[15], 16);
  dps->calib.c30 = getTwosComplement(((uint32_t)coef[16] << 8) | (uint32_t)coef[17], 16);

  // DPS310 ignores c31 and c40 (only used on SPL07_003)
  dps->calib.c31 = 0;
  dps->calib.c40 = 0;
}

/**
 * @brief Extract the raw 24-bit pressure and temperature measurements
 * @param dps The dps310 instance, expects the 6 result bytes (registers
 *            0x00-0x05, burst-read in one transaction) in the transaction buffer
 */
static void parse_sensor_data(struct Dps310_I2c *dps)
{
  uint32_t data_xlsb, data_lsb, data_msb;
  uint32_t prs_raw, tmp_raw;

  // Registers 0x00 to 0x02 for pressure
  data_msb = (uint32_t)dps->i2c_trans.buf[0] << 16;
  data_lsb = (uint32_t)dps->i2c_trans.buf[1] << 8;
  data_xlsb = (uint32_t)dps->i2c_trans.buf[2];
  prs_raw = data_msb | data_lsb | data_xlsb;
  dps->raw_pressure = getTwosComplement(prs_raw, 24);

  // Registers 0x03 to 0x05 for temperature
  data_msb = (uint32_t)dps->i2c_trans.buf[3] << 16;
  data_lsb = (uint32_t)dps->i2c_trans.buf[4] << 8;
  data_xlsb = (uint32_t)dps->i2c_trans.buf[5];
  tmp_raw = data_msb | data_lsb | data_xlsb;
  dps->raw_temperature = getTwosComplement(tmp_raw, 24);
}

/**
 * @brief Convert raw measurements into pressure [Pa] and temperature [deg C]
 * @param dps The dps310 instance
 *
 * Implements the compensation formulas of the datasheet (section 4.9) using
 * the factory calibration coefficients. The scale factors kT/kP belong to
 * the configured 16x oversampling; keep them in sync with the PRC settings
 * written in DPS310_STATUS_CONFIGURE_REGS.
 */
static void compensate_sensor(struct Dps310_I2c *dps)
{
  float Traw_sc = (float)dps->raw_temperature / DPS310_SCALE_FACTOR_KT;
  float Praw_sc = (float)dps->raw_pressure / DPS310_SCALE_FACTOR_KP;

  struct dps310_reg_calib_data *c = &dps->calib;

  dps->temperature = (c->c0 * 0.5f) + (c->c1 * Traw_sc);
  dps->pressure = c->c00 + Praw_sc * (c->c10 + Praw_sc * (c->c20 + Praw_sc * c->c30)) +
                  Traw_sc * c->c01 + Traw_sc * Praw_sc * (c->c11 + Praw_sc * c->c21);
}

/**
 * @brief Initialize the driver instance (no bus traffic yet)
 * @param dps   The dps310 instance
 * @param i2c_p I2C peripheral the sensor is connected to
 * @param addr  8-bit I2C slave address (DPS310_I2C_ADDR or DPS310_I2C_ADDR_ALT)
 *
 * Sensor detection and configuration happen asynchronously afterwards through
 * dps310_i2c_periodic() and dps310_i2c_event().
 */
void dps310_i2c_init(struct Dps310_I2c *dps, struct i2c_periph *i2c_p, uint8_t addr)
{
  dps->i2c_p = i2c_p;
  dps->i2c_trans.slave_addr = addr;
  dps->i2c_trans.status = I2CTransDone;
  dps->data_available = false;
  dps->initialized = false;
  dps->is_broken = false;
  dps->init_error_cnt = 0;
  dps->timer = 0;
  dps->status = DPS310_STATUS_UNINIT;
  dps->temp_coef_srce = 0;
  dps->temp_fix_step = 0;
}

/**
 * @brief Run the driver state machine, submitting at most one I2C transaction
 * @param dps The dps310 instance
 *
 * Should be called periodically. The call rate is decoupled from the
 * sensor-internal 16 Hz measurement rate: polling faster only re-reads the
 * latest sample. Does nothing while a transaction is still in flight.
 * A persistently failing or absent sensor is retried after a
 * DPS310_BROKEN_RETRY_US backoff instead of spamming the bus.
 */
void dps310_i2c_periodic(struct Dps310_I2c *dps)
{
  if (dps->is_broken) {
    // Back off, but periodically retry a full re-detection instead of giving up forever.
    // Unsigned arithmetic handles a wrap-around of the time counter.
    if ((uint32_t)(get_sys_time_usec() - dps->timer) < DPS310_BROKEN_RETRY_US) {
      return;
    }
    dps->is_broken = false;
    dps->init_error_cnt = 0;
    dps->status = DPS310_STATUS_UNINIT;
  }

  if (dps->i2c_trans.status != I2CTransDone) {
    return;
  }

  switch (dps->status) {
    case DPS310_STATUS_UNINIT:
      dps->data_available = false;
      dps->initialized = false;
      dps->temp_fix_step = 0;
      dps->status = DPS310_STATUS_GET_ID;
      break;

    case DPS310_STATUS_GET_ID:
      dps->i2c_trans.buf[0] = DPS310_REG_ID;
      i2c_transceive(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 1, 1);
      break;

    case DPS310_STATUS_TEMP_FIX:
      dps->i2c_trans.buf[0] = dps310_temp_fix_seq[dps->temp_fix_step][0];
      dps->i2c_trans.buf[1] = dps310_temp_fix_seq[dps->temp_fix_step][1];
      i2c_transmit(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 2);
      break;

    case DPS310_STATUS_WAIT_RDY:
      dps->i2c_trans.buf[0] = DPS310_REG_MEAS_CFG;
      i2c_transceive(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 1, 1);
      break;

    case DPS310_STATUS_GET_COEF_SRCE:
      dps->i2c_trans.buf[0] = DPS310_REG_COEF_SRCE;
      i2c_transceive(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 1, 1);
      break;

    case DPS310_STATUS_GET_CALIB:
      dps->i2c_trans.buf[0] = DPS310_REG_COEF;
      i2c_transceive(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 1, 18);
      break;

    case DPS310_STATUS_CONFIGURE_REGS:
      dps->i2c_trans.buf[0] = DPS310_REG_PRS_CFG;
      // Sensor-internal background rates, decoupled from the (e.g. 50Hz) periodic polling rate.
      // Datasheet budget: total conversion time < 1s/s. At 16x oversampling (27.6ms/conversion),
      // 16Hz P + 16Hz T = 883ms/s, so 16Hz is the maximum working rate for both channels.
      // Note: 16x oversampling requires P_SHIFT/T_SHIFT below and kT=kP=253952 in compensate_sensor().
      dps->i2c_trans.buf[1] = DPS310_PRS_CFG_PM_RATE_16HZ | DPS310_PRS_CFG_PM_PRC_16;
      dps->i2c_trans.buf[2] = DPS310_TMP_CFG_TMP_RATE_16HZ | DPS310_TMP_CFG_TMP_PRC_16 | dps->temp_coef_srce;
      dps->i2c_trans.buf[3] = 0x00; // Idle MEAS_CFG initially until CFG_REG is correctly established below!
      dps->i2c_trans.buf[4] = DPS310_CFG_REG_P_SHIFT | DPS310_CFG_REG_T_SHIFT;
      i2c_transmit(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 5);
      break;

    case DPS310_STATUS_CONFIGURE_MEAS:
      // Explicitly enable continuous measurements AFTER CFG_REG is written to avoid start-up timing bugs
      dps->i2c_trans.buf[0] = DPS310_REG_MEAS_CFG;
      dps->i2c_trans.buf[1] = DPS310_MEAS_CTRL_CONT;
      i2c_transmit(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 2);
      break;

    case DPS310_STATUS_READ_DATA:
      dps->i2c_trans.buf[0] = DPS310_REG_PSR_B2;
      i2c_transceive(dps->i2c_p, &dps->i2c_trans, dps->i2c_trans.slave_addr, 1, 6);
      break;

    default:
      break;
  }
}

/**
 * @brief Consume finished I2C transactions and advance the state machine
 * @param dps The dps310 instance
 *
 * Should be called from the event loop. On success it parses the responses
 * and publishes plausibility-checked measurements through data_available.
 * On failure it counts consecutive errors, escalating from a simple retry to
 * a full re-initialization (DPS310_MAX_ERROR_CNT) and, while uninitialized,
 * to a timed backoff.
 */
void dps310_i2c_event(struct Dps310_I2c *dps)
{
  if (dps->i2c_trans.status == I2CTransSuccess) {
    switch (dps->status) {
      case DPS310_STATUS_GET_ID:
        // Apply the temperature errata fix only on genuine DPS310 silicon.
        // Register-compatible parts (e.g. SPL07-003, ID 0x11) skip it.
        if (dps->i2c_trans.buf[0] == DPS310_CHIP_ID) {
          dps->temp_fix_step = 0;
          dps->status = DPS310_STATUS_TEMP_FIX;
        } else {
          dps->status = DPS310_STATUS_WAIT_RDY;
        }
        break;

      case DPS310_STATUS_TEMP_FIX:
        dps->temp_fix_step++;
        if (dps->temp_fix_step >= DPS310_TEMP_FIX_SEQ_LEN) {
          dps->status = DPS310_STATUS_WAIT_RDY;
        }
        break;

      case DPS310_STATUS_WAIT_RDY:
        // Only proceed once the sensor is ready and, crucially, the calibration
        // coefficients are valid (~40ms after power-up). Reading them too early
        // would silently corrupt every pressure value computed afterwards.
        if ((dps->i2c_trans.buf[0] & (DPS310_MEAS_CFG_COEF_RDY | DPS310_MEAS_CFG_SENSOR_RDY))
            == (DPS310_MEAS_CFG_COEF_RDY | DPS310_MEAS_CFG_SENSOR_RDY)) {
          dps->status = DPS310_STATUS_GET_COEF_SRCE;
        }
        break;

      case DPS310_STATUS_GET_COEF_SRCE:
        // Isolate the bit handling internal/external temperature sensor src logic
        dps->temp_coef_srce = dps->i2c_trans.buf[0] & DPS310_COEF_SRCE_BIT_TMP_COEF_SRCE;
        dps->status = DPS310_STATUS_GET_CALIB;
        break;

      case DPS310_STATUS_GET_CALIB:
        parse_calib_data(dps);
        dps->status = DPS310_STATUS_CONFIGURE_REGS;
        break;

      case DPS310_STATUS_CONFIGURE_REGS:
        dps->status = DPS310_STATUS_CONFIGURE_MEAS;
        break;

      case DPS310_STATUS_CONFIGURE_MEAS:
        dps->status = DPS310_STATUS_READ_DATA;
        dps->initialized = true;
        dps->init_error_cnt = 0;
        break;

      case DPS310_STATUS_READ_DATA:
        parse_sensor_data(dps);
        compensate_sensor(dps);
        dps->init_error_cnt = 0; // successful transaction, reset the consecutive failure counter
        // Only publish values within the specified measurement range of the sensor,
        // anything outside means the data got corrupted on its way here
        if ((dps->pressure >= DPS310_PRESSURE_MIN_PA) && (dps->pressure <= DPS310_PRESSURE_MAX_PA)) {
          dps->data_available = true;
        }
        break;

      default:
        break;
    }
    dps->i2c_trans.status = I2CTransDone;
  } else if (dps->i2c_trans.status == I2CTransFailed) {
    dps->init_error_cnt++;
    if (!dps->initialized) {
      /* Failure during initialization: count and eventually back off */
      if (dps->init_error_cnt >= DPS310_MAX_ERROR_CNT) {
        dps->is_broken = true;  // Back off, retried after DPS310_BROKEN_RETRY_US
        dps->timer = get_sys_time_usec();
      }
      dps->status = DPS310_STATUS_UNINIT;
    } else if (dps->init_error_cnt >= DPS310_MAX_ERROR_CNT) {
      /* Too many consecutive failures during measurements: full re-initialization */
      dps->initialized = false;
      dps->data_available = false;
      dps->init_error_cnt = 0;
      dps->status = DPS310_STATUS_UNINIT;
    }
    /* Otherwise a transient failure simply retries the current read on the next periodic tick */
    dps->i2c_trans.status = I2CTransDone;
  }
}
