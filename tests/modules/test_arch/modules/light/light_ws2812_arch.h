/*
 * Copyright (C) 2026 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file modules/light/light_ws2812_arch.h
 *  dummy WS2812 led handling for test
 */

#ifndef LIGHT_WS2812_ARCH_H
#define LIGHT_WS2812_ARCH_H

#include "std.h"

#ifndef WS2812_NB_LEDS
#define WS2812_NB_LEDS 8
#endif

static inline void light_ws2812_arch_init(void) {}
static inline void light_ws2812_arch_set(uint32_t led_number __attribute__((unused)), uint8_t r __attribute__((unused)),
    uint8_t g __attribute__((unused)), uint8_t b __attribute__((unused))) {}

#endif /* LIGHT_WS2812_ARCH_H */
