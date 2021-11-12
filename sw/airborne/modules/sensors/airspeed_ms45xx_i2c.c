/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2014 Felix Ruess <felix.ruess@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file modules/sensors/airspeed_ms45xx_i2c.c
 * Airspeed sensor module using the MS45xxDO digital pressure sensor via I2C.
 * Needs to be one of the versions with 14bit pressure and 11bit temperature.
 */

#include "std.h"
#include "mcu_periph/i2c.h"
#include "modules/sensors/airspeed_ms45xx_i2c.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

#ifndef USE_AIRSPEED_MS45XX
#if USE_AIRSPEED
#define USE_AIRSPEED_MS45XX TRUE
PRINT_CONFIG_MSG("USE_AIRSPEED_MS45XX set to TRUE since this is set USE_AIRSPEED")
#endif
#endif

#if USE_AIRSPEED_MS45XX
#include "state.h"
#endif

/** Default I2C device
 */
#ifndef MS45XX_I2C_DEV
#define MS45XX_I2C_DEV i2c2
#endif

/** Sensor I2C slave address (existing defaults 0x50, 0x6C and 0x8C)
 */
#ifndef MS45XX_I2C_ADDR
#define MS45XX_I2C_ADDR 0x50
#endif

/** MS45xx sensors pressure output type can be in PSI or InH2O, as defined in the datasheet
 *  if not defined to be MS45XX_PRESSURE_OUTPUT_TYPE_InH2O then PSI is used
 * */
#ifndef MS45XX_PRESSURE_OUTPUT_TYPE_InH2O
#define MS45XX_PRESSURE_OUTPUT_TYPE_InH2O 0
#endif

/** MS45xx pressure range in PSI or InH2O
 * The sensor is available in many ranges, the datasheet of your pressure sensor will tell which one
 * and what this range represents
 */
#ifndef MS45XX_PRESSURE_RANGE
#define MS45XX_PRESSURE_RANGE 1
#endif

/** Pressure Type 0 = Differential, 1 = Gauge
 * note there are theoretical more types than 2, e.g. Absolute not implemented */
#ifndef MS45XX_PRESSURE_TYPE
#define MS45XX_PRESSURE_TYPE 0
#endif

/** Use low pass filter on pressure values
 */
#ifndef USE_AIRSPEED_LOWPASS_FILTER
#define USE_AIRSPEED_LOWPASS_FILTER TRUE
#endif

/** MS45xx output Type.
 * 0 = Output Type A with 10% to 90%
 * 1 = Output Type B with 5% to 95%
 */
#ifndef MS45XX_OUTPUT_TYPE
#define MS45XX_OUTPUT_TYPE 0
#endif

/** Conversion factor from InH2O to Pa */
#define InH2O_TO_PA 249.08891

/** Conversion factor from psi to Pa */
#define PSI_TO_PA 6894.75729

#if MS45XX_PRESSURE_OUTPUT_TYPE_InH2O
#define OutputPressureToPa InH2O_TO_PA
#else
#define OutputPressureToPa PSI_TO_PA
#endif

#if MS45XX_OUTPUT_TYPE == 0
/* Offset and scaling for OUTPUT TYPE A:
 * p_raw = (0.8*16383)/ (Pmax - Pmin) * (pressure - Pmin) + 0.1*16383
 * For differential sensors Pmax = MS45XX_PRESSURE_RANGE = -Pmin.
 *
 * p_diff = (p_raw - 0.1*16383) * 2*RANGE/(0.8*16383) - RANGE
 * p_diff = p_raw * 2*RANGE/(0.8*16383) - (RANGE + (0.1 * 16383) * 2*RANGE/(0.8*16383)
 * p_diff = p_raw * 2*RANGE/(0.8*16383) - (1.25 * RANGE)
 * p_diff = p_raw * scale - offset
 * then convert to Pascal
 */
#ifndef MS45XX_PRESSURE_SCALE
#define MS45XX_PRESSURE_SCALE (2 * MS45XX_PRESSURE_RANGE / (0.8 * 16383) * OutputPressureToPa)
#endif
#ifndef MS45XX_PRESSURE_OFFSET
#define MS45XX_PRESSURE_OFFSET (1.25 * MS45XX_PRESSURE_RANGE * OutputPressureToPa)
#endif
#else /* Can still be improved using another if statment with MS45XX_PRESSURE_TYPE etc. */
/* Offset and scaling for OUTPUT TYPE B:
 * p_raw = (0.9*16383)/ (Pmax - Pmin) * (pressure - Pmin) + 0.05*16383
 */
#ifndef MS45XX_PRESSURE_SCALE
#define MS45XX_PRESSURE_SCALE (MS45XX_PRESSURE_RANGE/(0.9*16383)*OutputPressureToPa)
#endif
#ifndef MS45XX_PRESSURE_OFFSET
#define MS45XX_PRESSURE_OFFSET (((MS45XX_PRESSURE_RANGE*0.05*16383)/(0.9*16383))*OutputPressureToPa)
#endif
#endif

PRINT_CONFIG_VAR(MS45XX_OUTPUT_TYPE)
PRINT_CONFIG_VAR(MS45XX_PRESSURE_TYPE)
PRINT_CONFIG_VAR(MS45XX_PRESSURE_RANGE)
PRINT_CONFIG_VAR(MS45XX_PRESSURE_SCALE)
PRINT_CONFIG_VAR(MS45XX_PRESSURE_OFFSET)

/** Send a AIRSPEED_MS45XX message with every new measurement.
 * Mainly for debugging, use with caution, sends message at ~100Hz.
 */
#ifndef MS45XX_SYNC_SEND
#define MS45XX_SYNC_SEND FALSE
#endif

/** Quadratic scale factor for indicated airspeed.
 * airspeed = sqrt(2*p_diff/density)
 * With p_diff in Pa and standard air density of 1.225 kg/m^3,
 * default airspeed scale is 2/1.225
 */
#ifndef MS45XX_AIRSPEED_SCALE
#define MS45XX_AIRSPEED_SCALE 1.6327
#endif

/** Time constant for second order Butterworth low pass filter
 * Default of 0.15 should give cut-off freq of 1/(2*pi*tau) ~= 1Hz
 */
#ifndef MS45XX_LOWPASS_TAU
#define MS45XX_LOWPASS_TAU 0.15
#endif

struct AirspeedMs45xx ms45xx;
static struct i2c_transaction ms45xx_trans;
#ifdef USE_AIRSPEED_LOWPASS_FILTER
static Butterworth2LowPass ms45xx_filter;
#endif

static void ms45xx_downlink(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AIRSPEED_MS45XX(trans,dev,AC_ID,
                                &ms45xx.pressure,
                                &ms45xx.temperature,
                                &ms45xx.airspeed);
}

void ms45xx_i2c_init(void)
{
  ms45xx.pressure = 0.;
  ms45xx.temperature = 0;
  ms45xx.airspeed = 0.;
  ms45xx.pressure_type = MS45XX_PRESSURE_TYPE;
  ms45xx.pressure_scale = MS45XX_PRESSURE_SCALE;
  ms45xx.pressure_offset = MS45XX_PRESSURE_OFFSET;
  ms45xx.airspeed_scale = MS45XX_AIRSPEED_SCALE;
  ms45xx.sync_send = MS45XX_SYNC_SEND;

  ms45xx_trans.status = I2CTransDone;
  // setup low pass filter with time constant and 100Hz sampling freq
#ifdef USE_AIRSPEED_LOWPASS_FILTER
  init_butterworth_2_low_pass(&ms45xx_filter, MS45XX_LOWPASS_TAU,
                              MS45XX_I2C_PERIODIC_PERIOD, 0);
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_MS45XX, ms45xx_downlink);
#endif
}

void ms45xx_i2c_periodic(void)
{
  // Initiate next read
  if (ms45xx_trans.status == I2CTransDone) {
    i2c_receive(&MS45XX_I2C_DEV, &ms45xx_trans, MS45XX_I2C_ADDR, 4);
  }
}

#define AUTOSET_NB_MAX 20

void ms45xx_i2c_event(void)
{
  static int autoset_nb = 0;
  static float autoset_offset = 0.f;

  /* Check if transaction is succesfull */
  if (ms45xx_trans.status == I2CTransSuccess) {

    /* 2 MSB of data are status bits, 0 = good data, 2 = already fetched, 3 = fault */
    uint8_t status = (0xC0 & ms45xx_trans.buf[0]) >> 6;

    if (status == 0) {
      /* 14bit raw pressure */
      uint16_t p_raw = 0x3FFF & (((uint16_t)(ms45xx_trans.buf[0]) << 8) | (uint16_t)(ms45xx_trans.buf[1]));

      /* For type Diff
       * Output is proportional to the difference between Port 1 and Port 2. Output
       * swings positive when Port 1> Port 2. Output is 50% of total counts
       * when Port 1=Port 2.
       * For type Gauge
       * p_out = p_raw * scale - offset
       */

      float p_out = (p_raw * ms45xx.pressure_scale) - ms45xx.pressure_offset;
#ifdef USE_AIRSPEED_LOWPASS_FILTER
      ms45xx.pressure = update_butterworth_2_low_pass(&ms45xx_filter, p_out);
#else
      ms45xx.pressure = p_out;
#endif

      if (ms45xx.autoset_offset) {
        if (autoset_nb < AUTOSET_NB_MAX) {
          autoset_offset += p_raw * ms45xx.pressure_scale;
          autoset_nb++;
        } else {
          ms45xx.pressure_offset = autoset_offset / (float)autoset_nb;
          autoset_offset = 0.f;
          autoset_nb = 0;
          ms45xx.autoset_offset = false;
        }
      }

      /* 11bit raw temperature, 5 LSB bits not used */
      uint16_t temp_raw = 0xFFE0 & (((uint16_t)(ms45xx_trans.buf[2]) << 8) |
                                    (uint16_t)(ms45xx_trans.buf[3]));
      temp_raw = temp_raw >> 5;
      /* 0 = -50degC, 20147 = 150degC
       * ms45xx_temperature in 0.1 deg Celcius
       */
      ms45xx.temperature = ((uint32_t)temp_raw * 2000) / 2047 - 500;

      // Send (differential) pressure via ABI
      AbiSendMsgBARO_DIFF(MS45XX_SENDER_ID, ms45xx.pressure);
      // Send temperature as float in deg Celcius via ABI
      float temp = ms45xx.temperature / 10.0f;
      AbiSendMsgTEMPERATURE(MS45XX_SENDER_ID, temp);
      // Compute airspeed
      ms45xx.airspeed = sqrtf(Max(ms45xx.pressure * ms45xx.airspeed_scale, 0));

#if USE_AIRSPEED_MS45XX
      stateSetAirspeed_f(ms45xx.airspeed);
#endif
      if (ms45xx.sync_send) {
        ms45xx_downlink(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
      }
    }

    // Set to done
    ms45xx_trans.status = I2CTransDone;
  } else if (ms45xx_trans.status == I2CTransFailed) {
    // Just retry if failed
    ms45xx_trans.status = I2CTransDone;
  }
}
