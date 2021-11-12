/*
 * Copyright (C) 2009  ENAC, Pascal Brisset, Michel Gorraz
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

/**
 * @file modules/energy/MPPT.c
 * @brief Solar cells MPTT monitoring
 *
 */

#include <stdbool.h>
#include "modules/energy/MPPT.h"
#include "firmwares/fixedwing/main_fbw.h"
#include "mcu_periph/i2c.h"


#define MPPT_SLAVE_ADDR 0x40
#define NB_I2C_DATA 8
#define MPPT_MODE_ADDR 0xf

struct i2c_transaction mppt_trans;

/**
 0: VBat (mV)
 1: IBat (mA)
 2: PBat (mW)
 3: VSol (mV)
 4: ISol (mA)
 5: PSol (mW)
 6: IConv (mA)
 7: PConv (mW)

 9: IBat + IConv
*/



#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

uint8_t MPPT_mode;
/** A value different from 0 is a request from this mode */

static uint8_t MPPT_status;
#define MPPT_STATUS_IDLE 0
#define MPPT_STATUS_WRITING 1
#define MPPT_STATUS_ASKING 2
#define MPPT_STATUS_READING 3


static uint8_t data_index = 0xff;
static int16_t MPPT_data[NB_DATA];

void MPPT_init(void)
{
  MPPT_mode = 0;
  MPPT_status = MPPT_STATUS_IDLE;
}


static void MPPT_ask(void)
{
  data_index++;
  if (data_index >= NB_I2C_DATA) {
    /* Setting the current value */
    fbw_current_milliamp = MPPT_data[MPPT_IBAT_INDEX];

    MPPT_data[MPPT_ITOTAL_INDEX] = MPPT_data[MPPT_IBAT_INDEX] + MPPT_data[MPPT_ICONV_INDEX];
    DOWNLINK_SEND_MPPT(DefaultChannel, DefaultDevice, NB_DATA, MPPT_data);
    data_index = 0;
  }

  mppt_trans.buf[0] = data_index;
  i2c_transmit(&i2c0, &mppt_trans, MPPT_SLAVE_ADDR, 1);
  MPPT_status = MPPT_STATUS_ASKING;
}

void MPPT_periodic(void)
{

  if (mppt_trans.status == I2CTransSuccess) {
    switch (MPPT_status) {
      case MPPT_STATUS_IDLE:
        /* If free, change mode if needed */
        if (MPPT_mode) {
          mppt_trans.buf[0] = MPPT_MODE_ADDR;
          mppt_trans.buf[1] = 0;
          mppt_trans.buf[2] = MPPT_mode;
          i2c_transmit(&i2c0, &mppt_trans, MPPT_SLAVE_ADDR, 3);
          MPPT_mode = 0;
          MPPT_status = MPPT_STATUS_WRITING;
        } else {
          MPPT_ask();
        }
        break;

      case MPPT_STATUS_WRITING:
        MPPT_status = MPPT_STATUS_IDLE;
        break;

      case MPPT_STATUS_ASKING:
        /* The slave should send 2 bytes */
        i2c_receive(&i2c0, &mppt_trans, MPPT_SLAVE_ADDR, 2);
        MPPT_status = MPPT_STATUS_READING;
        break;

      case MPPT_STATUS_READING:
        /* We got 2 bytes */
        if (data_index < NB_I2C_DATA) {
          MPPT_data[data_index] = (mppt_trans.buf[0] << 8) | mppt_trans.buf[1];
        }
        MPPT_status = MPPT_STATUS_IDLE;
        break;
    }
  }
}
