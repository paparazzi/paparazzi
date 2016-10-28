/*
 * Copyright (C) 2010 Flixr
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

#ifndef SWITCH_SERVO_H
#define SWITCH_SERVO_H

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

extern bool switch_servo_on;
extern int16_t switch_servo_value;

#ifndef SWITCH_SERVO_ON_VALUE
#define SWITCH_SERVO_ON_VALUE 2000
#endif
#ifndef SWITCH_SERVO_OFF_VALUE
#define SWITCH_SERVO_OFF_VALUE 1000
#endif
#ifndef SWITCH_SERVO_SERVO
#define SWITCH_SERVO_SERVO SWITCH
#endif


extern void switch_servo_init(void);
extern void switch_servo_periodic(void);

#define SwitchServoOn()  ({ switch_servo_on = true; false; })
#define SwitchServoOff() ({ switch_servo_on = false; false; })

#endif //SWITCH_SERVO_H

