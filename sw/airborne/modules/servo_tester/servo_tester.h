/*
 * Copyright (C) 2012 Ewoud Smeur <ewoud_smeur@msn.com>
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

/** @file modules/servo_tester/servo_tester.h
 *
 * Module useful for testing a servo and finding the response
 */

#ifndef SERVO_TESTER_H
#define SERVO_TESTER_H

#include "generated/airframe.h"

extern int32_t servo_test_val;
extern bool is_servo;
extern bool do_servo_run;

void servo_tester_init(void);
void servo_tester_periodic(void);

#endif /* SERVO_TESTER_H */
