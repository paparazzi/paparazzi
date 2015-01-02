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

#include "demo_module.h"
#include "led.h"

void init_demo(void)
{
  // this part is already done by led_init in fact
  LED_INIT(DEMO_MODULE_LED);
  LED_OFF(DEMO_MODULE_LED);
}

void periodic_1Hz_demo(void)
{
  LED_TOGGLE(DEMO_MODULE_LED);
}

void periodic_10Hz_demo(void)
{
  LED_TOGGLE(DEMO_MODULE_LED);
}

void start_demo(void)
{
  LED_ON(DEMO_MODULE_LED);
}

void stop_demo(void)
{
  LED_OFF(DEMO_MODULE_LED);
}

