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

/** \file humid_htm_b71.c
 *  \brief TronSens HTM-B71 humidity/temperature sensor i2c interface
 *
 */

/* nice sensor but not very collaborative with others on the i2c bus */


#include "modules/meteo/humid_htm_b71.h"

#include "mcu_periph/sys_time.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"


#ifndef HTM_I2C_DEV
#define HTM_I2C_DEV i2c0
#endif

#define HTM_SLAVE_ADDR 0x28

struct i2c_transaction htm_trans;
uint8_t htm_status;
uint16_t humidhtm, temphtm;
float fhumidhtm, ftemphtm;


void humid_htm_init(void)
{
  htm_status = HTM_IDLE;
}

void humid_htm_start(void)
{
  if (sys_time.nb_sec > 1) {
    /* measurement request: wake up sensor, sample temperature/humidity */
    i2c_transmit(&HTM_I2C_DEV, &htm_trans, HTM_SLAVE_ADDR, 0);
    htm_status = HTM_MR;
  }
}

/* needs 18.5ms delay from measurement request */
void humid_htm_read(void)
{
  if (htm_status == HTM_MR_OK) {
    /* read humid and temp*/
    htm_status = HTM_READ_DATA;
    i2c_receive(&HTM_I2C_DEV, &htm_trans, HTM_SLAVE_ADDR, 4);
  }
}

void humid_htm_event(void)
{
  if (htm_trans.status == I2CTransSuccess) {
    switch (htm_status) {

      case HTM_MR:
        htm_status = HTM_MR_OK;
        htm_trans.status = I2CTransDone;
        break;

      case HTM_READ_DATA:
        /* check stale status */
        if (((htm_trans.buf[0] >> 6) & 0x3) == 0) {
          /* humidity */
          humidhtm = ((htm_trans.buf[0] & 0x3F) << 8) | htm_trans.buf[1];
          fhumidhtm = humidhtm / 163.83;
          /* temperature */
          temphtm = (htm_trans.buf[2] << 6) | (htm_trans.buf[3] >> 2);
          ftemphtm = -40.00 + 0.01 * temphtm;
          DOWNLINK_SEND_HTM_STATUS(DefaultChannel, DefaultDevice, &humidhtm, &temphtm, &fhumidhtm, &ftemphtm);
        }
        htm_trans.status = I2CTransDone;
        break;

      default:
        htm_trans.status = I2CTransDone;
        break;
    }
  }
}

