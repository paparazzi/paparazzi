/*
 * Copyright (C) 2019 Xavier Paris <xavier.paris@enac.fr>
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

/**
 * @file    arch/chibios/modules/light/light_ws2812_arch.h
 * @brief   ws2812 driver based on ChibiOS
 *
 * @author Xavier Paris
 * @maintainer Gautier Hattenberger <gautier.hattenberger@enac.fr>
 */

#pragma once

#include "std.h"

/** Number of LEDs
 */
#ifndef WS2812_NB_LEDS
#define WS2812_NB_LEDS 8
#endif

extern void light_ws2812_arch_init(void);

/** set color RGB color of one led
 */
extern void light_ws2812_arch_set(uint32_t led_number, uint8_t r, uint8_t g, uint8_t b);

