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
#include "modules/core/abi.h"
#include "mcu_periph/sys_time.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"


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
 * p_diff = (p_raw - offset) * scale
 * then convert to Pascal
 */
#ifndef MS45XX_PRESSURE_SCALE
#define MS45XX_PRESSURE_SCALE (2 * MS45XX_PRESSURE_RANGE / (0.8 * 16383) * OutputPressureToPa)
#endif
#ifndef MS45XX_PRESSURE_OFFSET
#define MS45XX_PRESSURE_OFFSET (16383 / 2)
#endif
#else /* Can still be improved using another if statment with MS45XX_PRESSURE_TYPE etc. */
/* Offset and scaling for OUTPUT TYPE B:
 * p_raw = (0.9*16383)/ (Pmax - Pmin) * (pressure - Pmin) + 0.05*16383
 */
#ifndef MS45XX_PRESSURE_SCALE
#define MS45XX_PRESSURE_SCALE (MS45XX_PRESSURE_RANGE/(0.9*16383)*OutputPressureToPa)
#endif
#ifndef MS45XX_PRESSURE_OFFSET
#define MS45XX_PRESSURE_OFFSET (0.05*16383)
#endif
#endif

PRINT_CONFIG_VAR(MS45XX_OUTPUT_TYPE)
PRINT_CONFIG_VAR(MS45XX_PRESSURE_TYPE)
PRINT_CONFIG_VAR(MS45XX_PRESSURE_RANGE)
PRINT_CONFIG_VAR(MS45XX_PRESSURE_SCALE)
PRINT_CONFIG_VAR(MS45XX_PRESSURE_OFFSET)


/** Quadratic scale factor for indicated airspeed.
 * airspeed = sqrt(2*p_diff/density)
 * With p_diff in Pa and standard air density of 1.225 kg/m^3,
 * default airspeed scale is 2/1.225
 */
#ifdef MS45XX_AIRSPEED_SCALE
PRINT_CONFIG_MSG("MS45XX changed air density. PS: Use MS45XX_PRESSURE_SCALE to calibrate the MS45XX.");
#endif

#define MS45XX_RHO_DIV_2 1.6327


struct AirspeedMs45xx ms45xx;
static struct i2c_transaction ms45xx_trans;

void ms45xx_i2c_init(void)
{
  ms45xx.pressure = 0.;
  ms45xx.temperature = 0;
  ms45xx.airspeed = 0.;
  ms45xx.raw_p = 0;
  ms45xx.pressure_type = MS45XX_PRESSURE_TYPE;
  ms45xx.pressure_scale = MS45XX_PRESSURE_SCALE;
  ms45xx.pressure_offset = MS45XX_PRESSURE_OFFSET;

  ms45xx_trans.status = I2CTransDone;
}


void ms45xx_i2c_periodic(void)
{
  // Initiate next read
  if (ms45xx_trans.status == I2CTransDone) {
    i2c_receive(&MS45XX_I2C_DEV, &ms45xx_trans, MS45XX_I2C_ADDR, 4);
  }
}

void ms45xx_i2c_event(void)
{
  /* Check if transaction is succesfull */
  if (ms45xx_trans.status == I2CTransSuccess) {

    /* 2 MSB of data are status bits, 0 = good data, 2 = already fetched, 3 = fault */
    uint8_t status = (0xC0 & ms45xx_trans.buf[0]) >> 6;

    if (status == 0) {
      /* 14bit raw pressure */
      uint16_t p_raw = 0x3FFF & (((uint16_t)(ms45xx_trans.buf[0]) << 8) | (uint16_t)(ms45xx_trans.buf[1]));
      ms45xx.raw_p = p_raw;

      /* 11bit raw temperature, 5 LSB bits not used */
      uint16_t temp_raw = 0xFFE0 & (((uint16_t)(ms45xx_trans.buf[2]) << 8) |
                                    (uint16_t)(ms45xx_trans.buf[3]));
      temp_raw = temp_raw >> 5;

      /* Reject any values that are the absolute minimum or maximums these
         can happen due to gnd lifts or communication errors on the bus */
      if(p_raw == 0x3FFF || p_raw == 0 || temp_raw == 0x7FF || temp_raw == 0) {
        ms45xx_trans.status = I2CTransDone;
        return;
      }

      /* For type Diff
       * Output is proportional to the difference between Port 1 and Port 2. Output
       * swings positive when Port 1> Port 2. Output is 50% of total counts
       * when Port 1=Port 2.
       * For type Gauge
       * p_out = (p_raw - offset) * scale
       */

      float p_out = (p_raw - ms45xx.pressure_offset) * ms45xx.pressure_scale;

      /* 0 = -50degC, 20147 = 150degC
       * ms45xx_temperature in 0.1 deg Celcius
       */
      ms45xx.temperature = ((uint32_t)temp_raw * 2000) / 2047 - 500;
      
      // if(electrical.vboard != 0) {
      //   float volt_diff = electrical.vboard - 5.0f;
      //   Bound(volt_diff, -0.7f, 0.5f);

      //   p_out -= 65.0f * volt_diff;
      //   ms45xx.temperature -= 8.87f * volt_diff;
      // }
      ms45xx.pressure = p_out;

      // Send (differential) pressure via ABI, air_data filters and zeroes it and computes the airspeed
      uint32_t now_ts = get_sys_time_usec();
      AbiSendMsgBARO_DIFF_RAW(MS45XX_SENDER_ID, now_ts, ms45xx.pressure);

      // Send temperature as float in deg Celcius via ABI
      float temp = ms45xx.temperature / 10.0f;
      AbiSendMsgTEMPERATURE_RAW(MS45XX_SENDER_ID, now_ts, temp, TEMPERATURE_TYPE_DIFF);

      // Compute airspeed
      float sign = 1.0f;
      if (ms45xx.pressure < 0.0f) {
        sign = -1.0f;
      }
      ms45xx.airspeed = sqrtf(ms45xx.pressure * sign * MS45XX_RHO_DIV_2) * sign;

    }

    // Set to done
    ms45xx_trans.status = I2CTransDone;
  } else if (ms45xx_trans.status == I2CTransFailed) {
    // Just retry if failed
    ms45xx_trans.status = I2CTransDone;
  }
}
