/*
 * Copyright (C) 2023 Flo&Fab <surname.name@enac.fr>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/actuators/servo_sts3032.h"
 * @author Flo&Fab <surname.name@enac.fr>
 * feetech sts3032 servo 
 */

#ifndef SERVO_STS3032_H
#define SERVO_STS3032_H


extern void servo_sts3032_init(void);
extern void servo_sts3032_test(void);
extern void servo_sts3032_event(void);

#endif  // SERVO_STS3032_H
