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

/** @file actuators_asctec.h
 *  Actuators driver for Asctec motor controllers.
 */

#ifndef ACTUATORS_ASCTEC_H
#define ACTUATORS_ASCTEC_H

#include "mcu_periph/i2c.h"

#include "generated/airframe.h"

enum actuators_asctec_cmd { NONE,
                            TEST,
                            REVERSE,
                            SET_ADDR
                          };

enum actuators_asctec_addr { FRONT,
                             BACK,
                             LEFT,
                             RIGHT
                           };

/* this is for the v1 protocol which does its own mixing */
enum actuators_asctec_cmds { PITCH,
                             ROLL,
                             YAW,
                             THRUST,
                             CMD_NB
                           };

struct ActuatorsAsctec {
  enum actuators_asctec_cmd cmd;
  enum actuators_asctec_addr cur_addr;
  enum actuators_asctec_addr new_addr;
  int32_t cmds[CMD_NB];
  struct i2c_transaction i2c_trans;
  volatile uint32_t nb_err;
};


extern struct ActuatorsAsctec actuators_asctec;

#define actuators_asctec_SetCommand(_v) {  \
    actuators_asctec.cmd = _v;             \
  }

#define actuators_asctec_SetNewAddr(_v) {  \
    actuators_asctec.new_addr = _v;        \
  }

#define actuators_asctec_SetCurAddr(_v) {  \
    actuators_asctec.cur_addr = _v;        \
  }

extern void actuators_asctec_init(void);
extern void actuators_asctec_set(bool_t motors_on);

#define ActuatorAsctecSet(_i, _v) { actuators_asctec.cmds[_i] = _v; }
#define ActuatorsAsctecInit() actuators_asctec_init()
#define ActuatorsAsctecCommit() actuators_asctec_set(autopilot_motors_on)


#endif /* ACTUATORS_ASCTEC_H */
