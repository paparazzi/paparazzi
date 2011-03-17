/*
 * $Id$
 *
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

#include "firmwares/rotorcraft/actuators.h"
#include "firmwares/rotorcraft/actuators/actuators_mkk.h"

#include "booz/booz2_commands.h"
#include "mcu_periph/i2c.h"
#include "sys_time.h"


struct ActuatorsMkk actuators_mkk;


uint32_t actuators_delay_time;
bool_t   actuators_delay_done;

void actuators_init(void) {

  supervision_init();
  const uint8_t actuators_addr[ACTUATORS_MKK_NB] = ACTUATORS_MKK_ADDR;
  for (uint8_t i=0; i<ACTUATORS_MKK_NB; i++) {
    actuators_mkk.trans[i].type = I2CTransTx;
    actuators_mkk.trans[i].len_w = 1;
    actuators_mkk.trans[i].slave_addr = actuators_addr[i];
    actuators_mkk.trans[i].status = I2CTransSuccess;
  }

#if defined BOOZ_START_DELAY && ! defined SITL
  actuators_delay_done = FALSE;
  SysTimeTimerStart(actuators_delay_time);
#else
  actuators_delay_done = TRUE;
  actuators_delay_time = 0;
#endif

}


void actuators_set(bool_t motors_on) {
#if defined BOOZ_START_DELAY && ! defined SITL
  if (!actuators_delay_done) {
    if (SysTimeTimer(actuators_delay_time) < SYS_TICS_OF_SEC(BOOZ_START_DELAY)) return;
    else actuators_delay_done = TRUE;
  }
#endif

  supervision_run(motors_on, FALSE, booz2_commands);
  for (uint8_t i=0; i<ACTUATORS_MKK_NB; i++) {
#ifdef KILL_MOTORS
    actuators_mkk.trans[i].buf[0] = 0;
#else
    actuators_mkk.trans[i].buf[0] = supervision.commands[i];
#endif
    i2c_submit(&ACTUATORS_MKK_DEVICE, &actuators_mkk.trans[i]);
  }
}
