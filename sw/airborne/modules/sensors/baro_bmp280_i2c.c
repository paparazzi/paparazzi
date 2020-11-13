/*
 * Chris Efstathiou hendrixgr@gmail.com
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
 * @file modules/sensors/baro_bmp280_i2c.c
 * Bosch BMP280 I2C sensor interface.
 *
 * This reads the values for pressure and temperature from the Bosch BMP280 sensor through I2C.
 */


#include "baro_bmp280_i2c.h"

#include "subsystems/abi.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "math/pprz_isa.h"

#if DOWNLINK && !defined(BMP280_SYNC_SEND)
#include "subsystems/datalink/telemetry.h"
#endif

/** default slave address */
#ifndef BMP280_SLAVE_ADDR
#define BMP280_SLAVE_ADDR BMP280_I2C_ADDR
#endif

float baro_alt = 0;
float baro_temp = 0;
float baro_press = 0;
bool baro_alt_valid = 0;


struct Bmp280_I2c baro_bmp280;

#if DOWNLINK && !defined(BMP280_SYNC_SEND)
static void send_baro_bmp_data(struct transport_tx *trans, struct link_device *dev)
{
  int32_t baro_altitude = (int32_t)(baro_alt * 100);
  int32_t baro_pressure = (int32_t)(baro_press);
  int32_t baro_temperature = (int32_t)(baro_temp * 100);

  pprz_msg_send_BMP_STATUS(trans, dev, AC_ID, &baro_altitude, &baro_altitude , &baro_pressure, & baro_temperature);

  return;
}
#endif


void baro_bmp280_init(void)
{

  bmp280_i2c_init(&baro_bmp280, &BMP280_I2C_DEV, BMP280_SLAVE_ADDR);
#if DOWNLINK && !defined(BMP280_SYNC_SEND)
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_BMP_STATUS, send_baro_bmp_data);
#endif
}

void baro_bmp280_periodic(void)
{
  bmp280_i2c_periodic(&baro_bmp280);
}

void baro_bmp280_event(void)
{
  bmp280_i2c_event(&baro_bmp280);

  if (baro_bmp280.data_available) {
    uint32_t now_ts = get_sys_time_usec();
    // send ABI message
    AbiSendMsgBARO_ABS(BARO_BMP_SENDER_ID, now_ts, baro_bmp280.pressure);
    AbiSendMsgTEMPERATURE(BARO_BMP_SENDER_ID, baro_bmp280.temperature);
    baro_bmp280.data_available = false;
    baro_alt = pprz_isa_altitude_of_pressure(baro_bmp280.pressure);
    baro_alt_valid = true;
    baro_press = (float)baro_bmp280.pressure;
    baro_temp = ((float)baro_bmp280.temperature) / 100;

#if defined(BMP280_SYNC_SEND)
    int32_t up = (int32_t)(baro_bmp280.raw_pressure);
    int32_t ut = (int32_t)(baro_bmp280.raw_temperature);
    int32_t p = (int32_t) baro_bmp280.pressure;
    int32_t t = (int32_t)(baro_bmp280.temperature);
    DOWNLINK_SEND_BMP_STATUS(DefaultChannel, DefaultDevice, &up, &ut, &p, &t);
#endif
  }
}
