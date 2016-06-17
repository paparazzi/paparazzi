/*
 * Copyright (C) 2011 Martin Mueller <martinmm@pfump.org>
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

/** \file charge_sens.c
 *  \brief I2C interface for University of Reading charge sensor
 *
 */

#include "modules/meteo/charge_sens.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"


#ifndef CHARGE_SENS_DEV
#define CHARGE_SENS_DEV i2c0
#endif

#define CHARGE_SENS_I2C_ADDR   0x78
#define CHARGE_NB              10

struct   i2c_transaction charge_trans;
uint16_t charge[CHARGE_NB];
int32_t  charge_cnt;

void charge_sens_init(void)
{
  charge_cnt = 0;
}

void charge_sens_periodic(void)
{
  i2c_receive(&CHARGE_SENS_DEV, &charge_trans, CHARGE_SENS_I2C_ADDR, 2);
}

void charge_sens_event(void)
{
  if (charge_trans.status == I2CTransSuccess) {
    /* read two byte atmosphere charge */
    charge[charge_cnt]  = charge_trans.buf[1] << 8;
    charge[charge_cnt] |= charge_trans.buf[0];
    charge_trans.status = I2CTransDone;

    if (++charge_cnt >= CHARGE_NB) {
      DOWNLINK_SEND_ATMOSPHERE_CHARGE(DefaultChannel, DefaultDevice,
                                      &charge[0], &charge[1], &charge[2], &charge[3], &charge[4],
                                      &charge[5], &charge[6], &charge[7], &charge[8], &charge[9]);
      charge_cnt = 0;
    }
  }
}
