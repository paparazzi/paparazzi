/*
 *
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file boards/bebop/actuators.h
 * Actuator driver for the bebop
 */

#ifndef ACTUATORS_BEBOP_RAW_H_
#define ACTUATORS_BEBOP_RAW_H_

#include <stdint.h>
#include "mcu_periph/i2c.h"

#define ACTUATORS_BEBOP_ADDR          0x8

// I2C Commands
#define ACTUATORS_BEBOP_SET_REF_SPEED 0x02    ///< Set reference speed
#define ACTUATORS_BEBOP_GET_OBS_DATA  0x20    ///< Get observation data
#define ACTUATORS_BEBOP_START_PROP    0x40    ///< Start the propellers
#define ACTUATORS_BEBOP_TOGGLE_GPIO   0x4D    ///< Toggle GPIO (reset, red led, green led)
#define ACTUATORS_BEBOP_STOP_PROP     0x60    ///< Stop the propellers
#define ACTUATORS_BEBOP_CLEAR_ERROR   0x80    ///< Clear all current erros
#define ACTUATORS_BEBOP_PLAY_SOUND    0x82    ///< Play a sound (0=stop, 1=short beep, 2=boot beep, 3=Be-Bop-Ah-Lula, negative=repeat)
#define ACTUATORS_BEBOP_GET_INFO      0xA0    ///< Get version information


struct ActuatorsBebop {
  struct i2c_transaction i2c_trans;   ///< I2C transaction for communicating with the bebop BLDC driver
  uint16_t rpm_ref[4];                ///< Reference RPM
  uint16_t rpm_obs[4];                ///< Observed RPM
  uint8_t led;                        ///< Current led status
};

#define ActuatorsBebopSet(_i, _v) { actuators_bebop.rpm_ref[_i] = _v; }
#define ActuatorsBebopCommit() actuators_bebop_commit();
#define ActuatorsBebopInit() actuators_bebop_init();

extern struct ActuatorsBebop actuators_bebop;
extern void actuators_bebop_commit(void);
extern void actuators_bebop_init(void);

#endif /* ACTUATORS_BEBOP_RAW_H_ */
