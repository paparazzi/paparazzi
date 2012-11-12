/*
 * Copyright (C) 2009  Gautier Hattenberger
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
 *
 */

/** \file demo_module.h
 *
 * demo module with blinking LEDs
 */

#ifndef DEMO_MODULE_H
#define DEMO_MODULE_H

#ifndef DEMO_MODULE_LED
#define DEMO_MODULE_LED 2
#endif

void init_demo(void);
void periodic_1Hz_demo(void);
void periodic_10Hz_demo(void);
void start_demo(void);
void stop_demo(void);

#endif
