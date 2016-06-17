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

#ifndef SERVO_SWITCH_H
#define SERVO_SWITCH_H

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

extern bool servo_switch_on;
extern int16_t servo_switch_value;

#ifndef SERVO_SWITCH_ON_VALUE
#define SERVO_SWITCH_ON_VALUE 2000
#endif
#ifndef SERVO_SWITCH_OFF_VALUE
#define SERVO_SWITCH_OFF_VALUE 1000
#endif
#ifndef SERVO_SWITCH_SERVO
#define SERVO_SWITCH_SERVO SWITCH
#endif


extern void servo_switch_init(void);
extern void servo_switch_periodic(void);

#define ServoSwitchOn()  ({ servo_switch_on = true; false; })
#define ServoSwitchOff() ({ servo_switch_on = false; false; })

#endif //SERVO_SWITCH_H

