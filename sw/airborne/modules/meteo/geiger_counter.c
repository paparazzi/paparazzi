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

/** \file geiger_counter.c
 *  \brief I2C interface for University of Reading Geiger counter
 *
 */

#include "modules/meteo/geiger_counter.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"


#ifndef GEIGER_CNT_DEV
#define GEIGER_CNT_DEV i2c0
#endif

#define GEIGER_CNT_I2C_ADDR   0x76

struct   i2c_transaction geiger_trans;
uint32_t count_geiger_1, count_geiger_2;
uint16_t volt_geiger;

void geiger_counter_init(void)
{
}

void geiger_counter_periodic(void)
{
  i2c_receive(&GEIGER_CNT_DEV, &geiger_trans, GEIGER_CNT_I2C_ADDR, 10);
}

void geiger_counter_event(void)
{
  if (geiger_trans.status == I2CTransSuccess) {
    count_geiger_1  = (geiger_trans.buf[3] << 24) |
                      (geiger_trans.buf[2] << 16) |
                      (geiger_trans.buf[1] << 8) |
                      (geiger_trans.buf[0]);
    count_geiger_2  = (geiger_trans.buf[7] << 24) |
                      (geiger_trans.buf[6] << 16) |
                      (geiger_trans.buf[5] << 8) |
                      (geiger_trans.buf[4]);
    volt_geiger     = (geiger_trans.buf[9] << 8) |
                      (geiger_trans.buf[8]);
    geiger_trans.status = I2CTransDone;

    if (volt_geiger & 0x8000) {
      volt_geiger &= 0x7FFF;
      DOWNLINK_SEND_GEIGER_COUNTER(DefaultChannel, DefaultDevice,
                                   &count_geiger_1, &count_geiger_2, &volt_geiger);
    }
  }
}
