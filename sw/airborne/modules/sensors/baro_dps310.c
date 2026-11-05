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
 * @file modules/sensors/baro_dps310.c
 * Infineon DPS310 I2C sensor interface.
 *
 * This code reads the values for pressure and temperature from the Infineon DPS310 sensor over I2C.
 */

#include "baro_dps310.h"
#include "modules/core/abi.h"

#ifdef DPS310_SYNC_SEND
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#endif

/** default slave address */
#ifndef DPS310_SLAVE_ADDR
  #define DPS310_SLAVE_ADDR DPS310_I2C_ADDR
#endif

struct Dps310_I2c baro_dps310;

void baro_dps310_init(void)
{
  dps310_i2c_init(&baro_dps310, &DPS310_I2C_PORT, DPS310_SLAVE_ADDR);
}

void baro_dps310_periodic(void)
{
  dps310_i2c_periodic(&baro_dps310);
}

void baro_dps310_event(void)
{
  dps310_i2c_event(&baro_dps310);

  if (baro_dps310.data_available) {
    uint32_t now_ts = get_sys_time_usec();
    // send ABI message
    AbiSendMsgBARO_ABS(BARO_DPS310_SENDER_ID, now_ts, baro_dps310.pressure);
    AbiSendMsgTEMPERATURE(BARO_DPS310_SENDER_ID, baro_dps310.temperature);
    baro_dps310.data_available = false;

#ifdef DPS310_SYNC_SEND
    float payload[4] = {(float)baro_dps310.raw_pressure, (float)baro_dps310.raw_temperature, baro_dps310.pressure, baro_dps310.temperature};
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 4, payload);
#endif
  }
}
