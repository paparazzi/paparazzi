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
 *
 */

/**
 * @file boards/bebop/electrical.c
 * Dummy electrical status readings for the bebop.
 * Because the voltage measurements is done trough the motor controllers.
 */

#include "subsystems/electrical.h"
#include "video.h"
#include <stdlib.h>

struct Electrical electrical;

void electrical_init(void)
{
  // First we try to kill the dragon-prog and its respawner if it is running (done here because initializes first)
  int ret __attribute__((unused)) = system("killall -q -9 watchdog.sh; killall -q -9 dragon-prog");

  // We also try to initialize the video CMOS chips here (Bottom and front)
  mt9v117_init();
  //mt9f002_init();
}

void electrical_periodic(void) { }
