/*
 * Copyright (C) 2009-2013 The Paparazzi Team
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

/** @file actuators_asctec_v2_new.h
 *  Actuators driver for Asctec v2 motor controllers with the new I2C protocol.
 */

#ifndef ACTUATORS_ASCTEC_V2_NEW_H
#define ACTUATORS_ASCTEC_V2_NEW_H

#include "mcu_periph/i2c.h"

#include "generated/airframe.h"

enum actuators_asctec_v2_cmd { NONE,
                               TEST,
                               REVERSE,
                               SET_ADDR
                             };

enum actuators_asctec_v2_addr { FRONT,
                                BACK,
                                LEFT,
                                RIGHT
                              };

struct ActuatorsAsctecV2 {
  enum actuators_asctec_v2_cmd cmd;
  enum actuators_asctec_v2_addr cur_addr;
  enum actuators_asctec_v2_addr new_addr;
  int32_t cmds[4];
  struct i2c_transaction i2c_trans;
  volatile uint32_t nb_err;
};


extern struct ActuatorsAsctecV2 actuators_asctec_v2;

#define actuators_asctec_v2_new_SetCommand(_v) {       \
    actuators_asctec_v2.cmd = _v;                  \
  }

#define actuators_asctec_v2_new_SetNewAddr(_v) {       \
    actuators_asctec_v2.new_addr = _v;             \
  }

#define actuators_asctec_v2_new_SetCurAddr(_v) {       \
    actuators_asctec_v2.cur_addr = _v;             \
  }

extern void actuators_asctec_v2_init(void);
extern void actuators_asctec_v2_set(void);

#define ActuatorAsctec_v2_newSet(_i, _v) { actuators_asctec_v2.cmds[_i] = _v; }
#define ActuatorsAsctec_v2_newInit() actuators_asctec_v2_init()
#define ActuatorsAsctec_v2_newCommit() actuators_asctec_v2_set()


#endif /* ACTUATORS_ASCTEC_V2_NEW_H */
