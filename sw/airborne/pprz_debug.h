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

#ifndef PPRZ_DEBUG_H
#define PPRZ_DEBUG_H

#define MY_ASSERT(cond) {     \
    if (!(cond)) while(1);      \
  }

#ifdef PPRZ_DEBUG

#include "std.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

extern uint8_t pprz_debug_mod;
extern uint8_t pprz_debug_err;

#define DEBUG_IMU          0
#define DEBUG_MAX_1117     1
#define DEBUG_SCP1000      2
#define DEBUG_LINK_MCU_IMU 3


#define ASSERT(cond, mod, err) {                                    \
    if (!(cond)) {                                                  \
      pprz_debug_mod = mod;                                         \
      pprz_debug_err = err;                                         \
      DOWNLINK_SEND_PPRZ_DEBUG(DefaultChannel, DefaultDevice, &pprz_debug_mod, &pprz_debug_err);  \
    }                                                               \
  }
#else
#define ASSERT(cond, mod, err) {}
#endif


#endif /* PPRZ_DEBUG_H */
