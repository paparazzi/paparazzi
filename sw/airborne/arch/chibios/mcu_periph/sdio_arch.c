/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
    */

/**
 * @file arch/chibios/mcu_periph/sdio_arch.c
 *
 * SDIO interface using ChibiOS API for Paparazzi
 *
 */

#include "std.h"
#include <string.h>
#include <ch.h>
#include <hal.h>
#include "mcu_periph/sdio.h"
#include <stdarg.h>
#include "mcu_periph/gpio.h"
#include BOARD_CONFIG


static enum {STOP, CONNECT} cnxState = STOP;


bool sdio_connect(void)
{
  if (!sdc_lld_is_card_inserted(NULL)) {
    return FALSE;
  }

  if (cnxState == CONNECT) {
    return TRUE;
  }

  // Try only 3 times to prevent hanging
  for (uint8_t i = 0; i < 3; i++) {
    sdcStart(&SDCD1, NULL);
    if (sdcConnect(&SDCD1) == HAL_SUCCESS) {
      cnxState = CONNECT;
      return TRUE;
    }
    sdcStop(&SDCD1);
    chThdSleepMilliseconds(100);
  }

  return FALSE;
}


bool sdio_disconnect(void)
{
  if (cnxState == STOP) {
    return TRUE;
  }
  if (sdcDisconnect(&SDCD1)) {
    return FALSE;
  }
  sdcStop(&SDCD1);
  cnxState = STOP;
  return TRUE;
}

bool is_card_inserted(void)
{
  return sdc_lld_is_card_inserted(NULL);
}

