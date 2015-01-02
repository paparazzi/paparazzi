/*
 * Copyright (C) 2011 Gautier Hattenberger
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

/** @file actuators_skiron.c
 *  Skiron motor speed controller by Michel.
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/actuators_skiron.h"

#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"

PRINT_CONFIG_VAR(ACTUATORS_SKIRON_I2C_DEV)

struct ActuatorsSkiron actuators_skiron;


void actuators_skiron_init(void)
{

  actuators_skiron.trans.type = I2CTransTx;
  actuators_skiron.trans.len_w = SERVOS_SKIRON_NB;
  actuators_skiron.trans.slave_addr = ACTUATORS_SKIRON_I2C_ADDR;
  actuators_skiron.trans.status = I2CTransDone;

}

void actuators_skiron_set(void)
{
#if defined ACTUATORS_START_DELAY && ! defined SITL
  if (!actuators_delay_done) {
    if (SysTimeTimer(actuators_delay_time) < USEC_OF_SEC(ACTUATORS_START_DELAY)) { return; }
    else { actuators_delay_done = TRUE; }
  }
#endif

#ifdef KILL_MOTORS
  for (uint8_t i = 0; i < ACTUATORS_SKIRON_NB; i++) {
    actuators_skiron.trans.buf[i] = 0;
  }
#endif

  i2c_submit(&ACTUATORS_SKIRON_I2C_DEV, &actuators_skiron.trans);
}
