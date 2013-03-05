/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 */

/** @file actuators_mkk.c
 *  Actuators driver for Mikrokopter motor controllers.
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/actuators_mkk.h"

#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"


struct ActuatorsMkk actuators_mkk;

void actuators_mkk_init(void) {
}


void actuators_mkk_set(void) {
  const uint8_t actuators_addr[ACTUATORS_MKK_NB] = ACTUATORS_MKK_ADDR;

#if defined ACTUATORS_START_DELAY && ! defined SITL
  if (!actuators_delay_done) {
    if (SysTimeTimer(actuators_delay_time) < USEC_OF_SEC(ACTUATORS_START_DELAY)) return;
    else actuators_delay_done = TRUE;
  }
#endif

  for (uint8_t i=0; i<ACTUATORS_MKK_NB; i++) {

#ifdef KILL_MOTORS
    actuators_mkk.trans[i].buf[0] = 0;
#endif

    i2c_transmit(&ACTUATORS_MKK_DEVICE, &actuators_mkk.trans[i], actuators_addr[i], 1);
  }
}
