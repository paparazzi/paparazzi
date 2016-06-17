/*
 * Original Code from:
 * Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com
 *
 * Adapated for Paparazzi by:
 * Copyright (C) 2012 Dino Hensen <dino.hensen@gmail.com>
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
 * @file boards/ardrone/actuators.h
 * Actuator driver for ardrone2-raw version
 */

#ifndef ACTUATORS_ARDRONE2_H_
#define ACTUATORS_ARDRONE2_H_

#include <stdint.h>

#ifndef ACTUATORS_ARDRONE_NB
#define ACTUATORS_ARDRONE_NB 4
#endif

#define SERVOS_TICS_OF_USEC(_v) (_v)

#define ActuatorArdroneSet(_i, _v) { actuators_pwm_values[_i] = _v; }
#define ActuatorsArdroneCommit() actuators_ardrone_commit();
#define ActuatorsArdroneInit() actuators_ardrone_init();

#define MOT_LEDOFF 0
#define MOT_LEDRED 1
#define MOT_LEDGREEN 2
#define MOT_LEDORANGE 3

uint16_t actuators_pwm_values[ACTUATORS_ARDRONE_NB];

extern void actuators_ardrone_commit(void);
extern void actuators_ardrone_init(void);


int actuators_ardrone_cmd(uint8_t cmd, uint8_t *reply, int replylen);
void actuators_ardrone_set_pwm(uint16_t pwm0, uint16_t pwm1, uint16_t pwm2, uint16_t pwm3);
void actuators_ardrone_set_leds(uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3);
void actuators_ardrone_close(void);

#endif /* ACTUATORS_ARDRONE2_H_ */
