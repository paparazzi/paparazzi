/*
 * Chris Efstathiou hendrixgr@gmail.com
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
 * @file modules/sensors/baro_bmp280.c
 * Bosch BMP280 sensor interface.
 *
 * This reads the values for pressure and temperature from the Bosch BMP280 sensor.
 */


#include "baro_bmp280.h"

#include "modules/core/abi.h"
#include "mcu_periph/sys_time.h"
#include "math/pprz_isa.h"


#ifndef BMP280_USE_SPI
#define BMP280_USE_SPI FALSE
#endif
PRINT_CONFIG_VAR(BMP280_USE_SPI)
PRINT_CONFIG_VAR(BMP280_DEV)
/** default slave address */

#ifndef BMP280_SLAVE_ADDR
#define BMP280_SLAVE_ADDR BMP280_I2C_ADDR
#endif

#ifndef BMP280_SLAVE_IDX
#define BMP280_SLAVE_IDX SPI_SLAVE0
#endif


float baro_alt = 0;
float baro_temp = 0;
float baro_press = 0;
bool baro_alt_valid = 0;


struct bmp280_t baro_bmp280;


void baro_bmp280_init(void)
{
  #if BMP280_USE_SPI
    baro_bmp280.bus = BMP280_SPI;
    baro_bmp280.spi.p = &BMP280_DEV;
    baro_bmp280.spi.slave_idx = BMP280_SLAVE_IDX;
  #else
    baro_bmp280.bus = BMP280_I2C;
    baro_bmp280.i2c.p = &BMP280_DEV;
    baro_bmp280.i2c.slave_addr = BMP280_SLAVE_ADDR;
  #endif
  bmp280_init(&baro_bmp280);
}

void baro_bmp280_periodic(void)
{
  bmp280_periodic(&baro_bmp280);
}

void baro_bmp280_event(void)
{
  bmp280_event(&baro_bmp280);

  if (baro_bmp280.data_available) {
    uint32_t now_ts = get_sys_time_usec();
    // send ABI message
    AbiSendMsgBARO_ABS_RAW(BARO_BMP_SENDER_ID, now_ts, baro_bmp280.pressure);
    AbiSendMsgTEMPERATURE_RAW(BARO_BMP_SENDER_ID, now_ts, baro_bmp280.temperature, TEMPERATURE_TYPE_BARO);
    baro_bmp280.data_available = false;
    baro_alt = pprz_isa_altitude_of_pressure(baro_bmp280.pressure);
    baro_alt_valid = true;
    baro_press = (float)baro_bmp280.pressure;
    baro_temp = ((float)baro_bmp280.temperature) / 100;
  }
}
