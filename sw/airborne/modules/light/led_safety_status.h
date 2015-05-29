/*
 * Copyright (C) 2012 Pranay Sinha <psinha@transition-robotics.com>
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
 * @file modules/light/led_safety_status.h
 *
 * Simple module to blink LEDs when battery voltage drops below a certain
 * level, radio control is lost or when takeoff safety conditions are not met.
 */

#ifndef LED_SAFETY_STATUS_H
#define LED_SAFETY_STATUS_H

#include "std.h"

/**
 * Initialises periodic loop; place more init functions here if expanding driver
 */
extern void led_safety_status_init(void);

/**
 * Periodic function that makes the leds blink in the right pattern for
 * each situation.
 */
extern void led_safety_status_periodic(void);

#endif  /* LED_SAFETY_STATUS_H */

