/*
 * Florian Sansou florian.sansou@enac.fr
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
 *  SPA06 sensor interface.
 *
 * This reads the values for pressure and temperature from the SPA06 sensor.
 */


#include "baro_spa06.h"

#include "modules/core/abi.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "math/pprz_isa.h"

#if DOWNLINK && !defined(SPA06_SYNC_SEND)
#include "modules/datalink/telemetry.h"
#endif
PRINT_CONFIG_VAR(SPA06_SYNC_SEND)

#ifndef SPA06_USE_SPI
#define SPA06_USE_SPI FALSE
#endif
PRINT_CONFIG_VAR(SPA06_USE_SPI)
PRINT_CONFIG_VAR(SPA06_DEV)
/** default slave address */

#ifndef SPA06_SLAVE_ADDR
#define SPA06_SLAVE_ADDR SPA06_I2C_ADDR
#endif

#ifndef SPA06_SLAVE_IDX
#define SPA06_SLAVE_IDX SPI_SLAVE0
#endif


float baro_alt = 0;
float baro_temp = 0;
float baro_press = 0;
bool baro_alt_valid = 0;


struct spa06_t baro_spa06;

#if DOWNLINK && !defined(SPA06_SYNC_SEND)
static void send_baro_spa_data(struct transport_tx *trans, struct link_device *dev)
{
  int32_t baro_altitude = (int32_t)(baro_alt * 100);
  int32_t baro_pressure = (int32_t)(baro_press);
  int32_t baro_temperature = (int32_t)(baro_temp * 100);

  pprz_msg_send_BMP_STATUS(trans, dev, AC_ID, &baro_altitude, &baro_altitude , &baro_pressure, & baro_temperature);

  return;
}
#endif


void baro_spa06_init(void)
{
  #if SPA06_USE_SPI
    baro_spa06.bus = SPA06_SPI;
    baro_spa06.spi.p = &SPA06_DEV;
    baro_spa06.spi.slave_idx = SPA06_SLAVE_IDX;
  #else
    baro_spa06.bus = SPA06_I2C;
    baro_spa06.i2c.p = &SPA06_DEV;
    baro_spa06.i2c.slave_addr = SPA06_SLAVE_ADDR;
  #endif
  spa06_init(&baro_spa06);
#if DOWNLINK && !defined(SPA06_SYNC_SEND)
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_BMP_STATUS, send_baro_spa_data);
#endif
}

void baro_spa06_periodic(void)
{
  spa06_periodic(&baro_spa06);
}

void baro_spa06_event(void)
{
  spa06_event(&baro_spa06);

  if (baro_spa06.data_available) {
    uint32_t now_ts = get_sys_time_usec();
    // send ABI message
    AbiSendMsgBARO_ABS(BARO_SPA_SENDER_ID, now_ts, baro_spa06.pressure);
    AbiSendMsgTEMPERATURE(BARO_SPA_SENDER_ID, baro_spa06.temperature);
    baro_spa06.data_available = false;
    baro_alt = pprz_isa_altitude_of_pressure(baro_spa06.pressure);
    baro_alt_valid = true;
    baro_press = (float)baro_spa06.pressure;
    baro_temp = ((float)baro_spa06.temperature) / 100;

#if defined(SPA06_SYNC_SEND)
    int32_t up = (int32_t)(baro_spa06.raw_pressure);
    int32_t ut = (int32_t)(baro_spa06.raw_temperature);
    int32_t p = (int32_t) baro_spa06.pressure;
    int32_t t = (int32_t)(baro_spa06.temperature);
    DOWNLINK_SEND_BMP_STATUS(DefaultChannel, DefaultDevice, &up, &ut, &p, &t);
#endif
  }
}
