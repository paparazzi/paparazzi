/*
 * Copyright (C) 2010 Martin Mueller
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file dust_gp2y.c
 *  \brief Sharp GP2Y1010AU dust sensor interface
 *
 *   This reads the values for dust density from the Sharp GP2Y1010AU0F sensor
 *   through I2C (needs I2C ADC at the sensor).
 */


#include "modules/meteo/dust_gp2y.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

uint8_t  dust_gp2y_status;
uint16_t dust_gp2y_density;
float dust_gp2y_density_f;

struct i2c_transaction gp2y_trans;


#ifndef GP2Y_I2C_DEV
#define GP2Y_I2C_DEV i2c0
#endif

#define GP2Y_SLAVE_ADDR 0xED

void dust_gp2y_init(void)
{
  dust_gp2y_status = DUST_GP2Y_UNINIT;
}

void dust_gp2y_periodic(void)
{
  if (dust_gp2y_status == DUST_GP2Y_IDLE) {
    i2c_receive(&GP2Y_I2C_DEV, &gp2y_trans, GP2Y_SLAVE_ADDR, 2);
  } else if (dust_gp2y_status == DUST_GP2Y_UNINIT && sys_time.nb_sec > 1) {
    dust_gp2y_status = DUST_GP2Y_IDLE;
  }
}

void dust_gp2y_event(void)
{
  if (gp2y_trans.status == I2CTransSuccess) {
    /* read two byte particle density */
    dust_gp2y_density  = gp2y_trans.buf[0] << 8;
    dust_gp2y_density |= gp2y_trans.buf[1];

    /* "just for reference and not for guarantee" */
    dust_gp2y_density_f = ((dust_gp2y_density / 1024.) * 3.3 * (51. / 33.) - 0.6) * (0.5 / 3.);
    if (dust_gp2y_density_f < 0) {
      dust_gp2y_density_f = 0;
    }

    DOWNLINK_SEND_GP2Y_STATUS(DefaultChannel, DefaultDevice, &dust_gp2y_density, &dust_gp2y_density_f);

    gp2y_trans.status = I2CTransDone;
  }
}
