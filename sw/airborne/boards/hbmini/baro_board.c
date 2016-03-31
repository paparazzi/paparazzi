/*
* Copyright (C) 2010 Christoph Niemann
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

/** @file boards/hbmini/baro_board.c
 *  Baro board interface for Bosch BMP085 on HBmini I2C1 with EOC check.
 */

#include "std.h"
#include "baro_board.h"
#include "subsystems/sensors/baro.h"
#include "peripherals/bmp085.h"
#include "peripherals/bmp085_regs.h"
#include "subsystems/abi.h"
#include "led.h"

struct Bmp085 baro_bmp085;

void baro_init(void)
{
  bmp085_init(&baro_bmp085, &i2c1, BMP085_SLAVE_ADDR);
#ifdef BARO_LED
  LED_OFF(BARO_LED);
#endif
}

void baro_periodic(void)
{
  if (baro_bmp085.initialized) {
    bmp085_periodic(&baro_bmp085);
  } else {
    bmp085_read_eeprom_calib(&baro_bmp085);
  }
}

void bmp_baro_event(void)
{
  bmp085_event(&baro_bmp085);

  if (baro_bmp085.data_available) {
    float pressure = (float)baro_bmp085.pressure;
    AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, pressure);
    baro_bmp085.data_available = false;
#ifdef BARO_LED
    RunOnceEvery(10, LED_TOGGLE(BARO_LED));
#endif
  }
}
