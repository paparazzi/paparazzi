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
 * Needs one of the differential versions with 14bit pressure and 11bit temperature.
 */

#include "std.h"
#include "mcu_periph/i2c.h"
#include "modules/sensors/airspeed_ms45xx_i2c.h"

/** Default I2C device
 */
#ifndef MS45XX_I2C_DEV
#define MS45XX_I2C_DEV i2c2
#endif

/** Sensor I2C slave address (defaults 0x50, 0x6C and 0x8C) */
#ifndef MS45XX_I2C_ADDR
#define MS45XX_I2C_ADDR 0x50
#endif

/** MS45xx pressure range in psi.
 * The sensor is available in 1, 2, 5, 15, 30, 50, 100, 150 psi ranges.
 */
#ifndef MS45XX_PRESSURE_RANGE
#define MS45XX_PRESSURE_RANGE 5
#endif

/** MS45xx output Type.
 * 0 = Output Type A with 10% to 90%
 * 1 = Output Type B with 5% to 95%
 */
#ifndef MS45XX_OUTPUT_TYPE
#define MS45XX_OUTPUT_TYPE 0
#endif

/** Conversion factor from psi to Pa */
#define PSI_TO_PA 6894.75729

#ifndef MS45XX_SYNC_SEND
#define MS45XX_SYNC_SEND FALSE
#endif

struct i2c_transaction ms45xx_trans;
struct AirspeedMs45xx ms45xx;


/** Quadratic scale factor for airspeed.
 * airspeed = sqrt(2*p_diff/density)
 * With p_diff in Pa and standard air density of 1.225 kg/m^3,
 * default airspeed scale is 2/1.225
 */
#ifndef MS45XX_AIRSPEED_SCALE
#define MS45XX_AIRSPEED_SCALE 1.6327
#endif

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

static void ms45xx_downlink(void)
{
  DOWNLINK_SEND_MS45XX_AIRSPEED(DefaultChannel, DefaultDevice,
                                &ms45xx.diff_pressure,
                                &ms45xx.temperature, &ms45xx.airspeed);
}

void ms45xx_i2c_init(void)
{
  ms45xx.diff_pressure = 0;
  ms45xx.temperature = 0;
  ms45xx.airspeed = 0.;
  ms45xx.sync_send = MS45XX_SYNC_SEND;

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
  ms45xx.pressure_scale = 2 * MS45XX_PRESSURE_RANGE / (0.8 * 16383) * PSI_TO_PA;
  ms45xx.pressure_offset = 1.25 * MS45XX_PRESSURE_RANGE * PSI_TO_PA;
#else
  /* Offset and scaling for OUTPUT TYPE B:
   * p_raw = (0.9*16383)/ (Pmax - Pmin) * (pressure - Pmin) + 0.05*16383
   */
  ms45xx.pressure_scale = 2 * MS45XX_PRESSURE_RANGE / (0.9 * 16383) * PSI_TO_PA;
  ms45xx.pressure_offset = (1.0 + 0.1 / 0.9) * MS45XX_PRESSURE_RANGE  * PSI_TO_PA;
#endif

  ms45xx_trans.status = I2CTransDone;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "MS45XX_AIRSPEED", ms45xx_downlink);
#endif
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
      uint16_t p_raw = 0x3FFF & (((uint16_t)(ms45xx_trans.buf[0]) << 8) |
                                 (uint16_t)(ms45xx_trans.buf[1]));
      /* Output is proportional to the difference between Port 1 and Port 2. Output
       * swings positive when Port 1> Port 2. Output is 50% of total counts
       * when Port 1=Port 2.
       * p_diff = p_raw * scale - offset
       */
      ms45xx.diff_pressure = p_raw * ms45xx.pressure_scale - ms45xx.pressure_offset;

      /* 11bit raw temperature, 5 LSB bits not used */
      uint16_t temp_raw = 0xFFE0 & (((uint16_t)(ms45xx_trans.buf[2]) << 8) |
                                    (uint16_t)(ms45xx_trans.buf[3]));
      temp_raw = temp_raw >> 5;
      /* 0 = -50degC, 20147 = 150degC
       * ms45xx_temperature in 0.1 deg Celcius
       */
      ms45xx.temperature = ((uint32_t)temp_raw * 2000) / 2047 - 500;

      // Compute airspeed
      ms45xx.airspeed = sqrtf(ms45xx.diff_pressure * ms45xx.airspeed_scale);
#if USE_AIRSPEED
      stateSetAirspeed_f(&ms45xx_airspeed);
#endif
      if (ms45xx.sync_send) {
        ms45xx_downlink();
      }
    }

    // Set to done
    ms45xx_trans.status = I2CTransDone;
  } else if (ms45xx_trans.status == I2CTransFailed) {
    // Just retry if failed
    ms45xx_trans.status = I2CTransDone;
  }
}
