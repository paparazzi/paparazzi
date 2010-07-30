/*
 * $Id: actuators_buss_twi_blmc_hw.h 3847 2009-08-02 21:47:31Z poine $
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

#ifndef BOOZ_ACTUATORS_MKK_H
#define BOOZ_ACTUATORS_MKK_H

#include "std.h"

#ifndef ACTUATORS_MKK_DEVICE
#define ACTUATORS_MKK_DEVICE i2c0
#endif

#define __Device(dev, _z) dev##_##_z 
#define  _Device(dev, _z) __Device(dev, _z)
#define  Device( _z) _Device(ACTUATORS_MKK_DEVICE, _z)

#define   DeviceBuf                    Device(buf)
#define   DeviceTransmit(_x, _y, _z)   Device(transmit(_x, _y, _z))

enum actuators_mkk_status {IDLE, BUSY};

struct ActuatorsMkk {
  volatile enum actuators_mkk_status status;
  volatile bool_t  i2c_done;
  volatile uint8_t idx;
};

extern struct ActuatorsMkk actuators_mkk; 

#include "actuators/booz_actuators_mkk_arch.h"
/* must be defined by underlying achitecture */
extern void booz_actuators_mkk_arch_init(void);


#include "airframe.h"
#include "actuators/booz_supervision.h"
extern const uint8_t actuators_addr[];


#ifdef KILL_MOTORS
#define ActuatorsMkkI2cHandler() {					\
    actuators_mkk.idx++;						\
    if (actuators_mkk.idx<ACTUATORS_MKK_NB) {				\
      DeviceBuf[0] = 0;							\
      DeviceTransmit(actuators_addr[actuators_mkk.idx], 1, &actuators_mkk.i2c_done); \
    }									\
    else								\
      actuators_mkk.status = IDLE;					\
  }
#else /* KILL_MOTORS */
#define ActuatorsMkkI2cHandler() {					\
    actuators_mkk.idx++;						\
    if (actuators_mkk.idx<ACTUATORS_MKK_NB) {				\
      DeviceBuf[0] = supervision.commands[actuators_mkk.idx];		\
      DeviceTransmit(actuators_addr[actuators_mkk.idx], 1, &actuators_mkk.i2c_done); \
    }									\
    else								\
      actuators_mkk.status = IDLE;					\
  }
#endif /* KILL_MOTORS */

#endif /* BOOZ_ACTUATORS_MKK_H */
