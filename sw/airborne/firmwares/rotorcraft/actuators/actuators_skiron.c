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

#include "firmwares/rotorcraft/actuators.h"
#include "firmwares/rotorcraft/actuators/actuators_skiron.h"

#include "firmwares/rotorcraft/commands.h"
#include "mcu_periph/i2c.h"
#include "sys_time.h"


struct ActuatorsSkiron actuators_skiron;


uint32_t actuators_delay_time;
bool_t   actuators_delay_done;

void actuators_init(void) {

  supervision_init();
  actuators_skiron.trans.type = I2CTransTx;
  actuators_skiron.trans.len_w = ACTUATORS_SKIRON_NB;
  actuators_skiron.trans.slave_addr = ACTUATORS_SKIRON_I2C_ADDR;
  actuators_skiron.trans.status = I2CTransDone;
  const uint8_t actuators_idx[ACTUATORS_SKIRON_NB] = ACTUATORS_SKIRON_IDX;
  for (uint8_t i=0; i<ACTUATORS_SKIRON_NB; i++) {
    actuators_skiron.actuators_idx[i] = actuators_idx[i];
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

  supervision_run(motors_on, FALSE, commands);
  for (uint8_t i=0; i<ACTUATORS_SKIRON_NB; i++) {
    uint8_t idx = actuators_skiron.actuators_idx[i];
#ifdef KILL_MOTORS
    actuators_skiron.trans.buf[idx] = 0;
#else
    actuators_skiron.trans.buf[idx] = supervision.commands[i];
#endif
  }
  i2c_submit(&ACTUATORS_SKIRON_DEVICE, &actuators_skiron.trans);
}
