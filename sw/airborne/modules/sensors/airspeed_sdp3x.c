/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file modules/sensors/airspeed_sdp3x.c
 *  Airspeed driver for the SDP3X pressure sensor via I2C.
 */

#include "std.h"
#include "mcu_periph/i2c.h"
#include "modules/sensors/airspeed_sdp3x.h"
#include "subsystems/abi.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

/** Commands and scales
 */
#define SDP3X_SCALE_TEMPERATURE   200.0f
#define SDP3X_RESET_ADDR          0x00
#define SDP3X_RESET_CMD           0x06

#define SDP3X_CONT_MEAS_AVG_MODE  0x3615
#define SDP3X_CONT_NONE_MODE      0x361E

#define SDP3X_SCALE_PRESSURE_SDP31  60
#define SDP3X_SCALE_PRESSURE_SDP32  240
#define SDP3X_SCALE_PRESSURE_SDP33  20

/** Sensor I2C slave address (existing defaults 0x42, 0x44 and 0x46)
 */
#ifndef SDP3X_I2C_ADDR
#define SDP3X_I2C_ADDR 0x42
#endif

/** Default operation mode
 */
#ifndef SDP3X_MODE
#define SDP3X_MODE SDP3X_CONT_MEAS_AVG_MODE
#endif

/** Default scale for SDP31
 */
#ifndef SDP3X_PRESSURE_SCALE
#define SDP3X_PRESSURE_SCALE SDP3X_SCALE_PRESSURE_SDP31
#endif

/* Default offset
 */
#ifndef SDP3X_PRESSURE_OFFSET
#define SDP3X_PRESSURE_OFFSET 0.f
#endif

PRINT_CONFIG_VAR(SDP3X_PRESSURE_SCALE)
PRINT_CONFIG_VAR(SDP3X_PRESSURE_OFFSET)

/** Send a AIRSPEED_MS45XX message with every new measurement. FIXME
 * Mainly for debugging, use with caution, sends message at ~100Hz.
 */
#ifndef SDP3X_SYNC_SEND
#define SDP3X_SYNC_SEND FALSE
#endif

/** Quadratic scale factor for indicated airspeed.
 * airspeed = sqrt(2*p_diff/density)
 * With p_diff in Pa and standard air density of 1.225 kg/m^3,
 * default airspeed scale is 2/1.225
 */
#ifndef SDP3X_AIRSPEED_SCALE
#define SDP3X_AIRSPEED_SCALE 1.6327
#endif

struct AirspeedSdp3x sdp3x;
static struct i2c_transaction sdp3x_trans;

static bool sdp3x_crc(const uint8_t data[], unsigned size, uint8_t checksum)
{
  uint8_t crc_value = 0xff;

  // calculate 8-bit checksum with polynomial 0x31 (x^8 + x^5 + x^4 + 1)
  for (unsigned i = 0; i < size; i++) {
    crc_value ^= (data[i]);

    for (int bit = 8; bit > 0; --bit) {
      if (crc_value & 0x80) {
        crc_value = (crc_value << 1) ^ 0x31;

      } else {
        crc_value = (crc_value << 1);
      }
    }
  }

  // verify checksum
  return (crc_value == checksum);
}

static void sdp3x_downlink(struct transport_tx *trans, struct link_device *dev)
{
  int16_t temp = (int16_t)(sdp3x.temperature * 10.f);
  pprz_msg_send_AIRSPEED_MS45XX(trans,dev,AC_ID,
                                &sdp3x.pressure,
                                &temp,
                                &sdp3x.airspeed);
}

void sdp3x_init(void)
{
  sdp3x.pressure = 0.;
  sdp3x.temperature = 0;
  sdp3x.airspeed = 0.;
  sdp3x.pressure_scale = SDP3X_PRESSURE_SCALE;
  sdp3x.pressure_offset = SDP3X_PRESSURE_OFFSET;
  sdp3x.airspeed_scale = SDP3X_AIRSPEED_SCALE;
  sdp3x.sync_send = SDP3X_SYNC_SEND;
  sdp3x.initialized = false;

  sdp3x_trans.status = I2CTransDone;
  // setup low pass filter with time constant and 100Hz sampling freq

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_MS45XX, sdp3x_downlink); // FIXME
#endif
}

void sdp3x_periodic(void)
{
  if (sdp3x_trans.status != I2CTransDone) {
    return; // not ready
  }

  if (sdp3x.initialized) {
    // Initiate next read
    i2c_receive(&SDP3X_I2C_DEV, &sdp3x_trans, SDP3X_I2C_ADDR, 6);
  }
  else {
    // Init sensor in continuous mode
    sdp3x_trans.buf[0] = SDP3X_MODE >> 8;
    sdp3x_trans.buf[1] = SDP3X_MODE & 0xFF;
    i2c_transmit(&SDP3X_I2C_DEV, &sdp3x_trans, SDP3X_I2C_ADDR, 2);
    sdp3x.initialized = true;
  }
}

void sdp3x_event(void)
{
  /* Check if transaction is succesfull */
  if (sdp3x_trans.status == I2CTransSuccess) {

    if (sdp3x.initialized) {
      // make local copy of buffer
      uint8_t buf[6];
      for (uint8_t i = 0; i < 6; i++) {
        buf[i] = sdp3x_trans.buf[i];
      }

      // Check the CRC
      if (!sdp3x_crc(&buf[0], 2, buf[2]) || !sdp3x_crc(&buf[3], 2, buf[5])) {
        // error
        sdp3x_trans.status = I2CTransDone;
        return;
      }

      int16_t p_raw = ((int16_t)(buf[0]) << 8) | (int16_t)(buf[1]);
      sdp3x.pressure = ((float)p_raw / sdp3x.pressure_scale) - sdp3x.pressure_offset;

      int16_t t_raw = ((int16_t)(buf[3]) << 8) | (int16_t)(buf[4]);
      sdp3x.temperature = (float)t_raw / SDP3X_SCALE_TEMPERATURE;

      // Send (differential) pressure via ABI
      AbiSendMsgBARO_DIFF(SDP3X_SENDER_ID, sdp3x.pressure);
      // Send temperature as float in deg Celcius via ABI
      AbiSendMsgTEMPERATURE(SDP3X_SENDER_ID, sdp3x.temperature);
      // Compute airspeed
      sdp3x.airspeed = sqrtf(Max(sdp3x.pressure * sdp3x.airspeed_scale, 0));

      if (sdp3x.sync_send) {
        sdp3x_downlink(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
      }
    }

    // Set to done
    sdp3x_trans.status = I2CTransDone;
  } else if (sdp3x_trans.status == I2CTransFailed) {
    // Just retry if failed
    sdp3x_trans.status = I2CTransDone;
  }
}
