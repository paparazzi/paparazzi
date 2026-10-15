/*
 * Copyright (C) 2026 Tomaso De Ponti <tomaso@aerogriduav.com>
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
 * @file    arch/chibios/modules/sound/buzzer_arch.h
 * @brief   Piezo buzzer tone output with a ChibiOS PWM driver
 *
 * The board defines BUZZER_GPIO, BUZZER_PIN, BUZZER_AF,
 * BUZZER_PWMD and BUZZER_PWM_CHANNEL.
 */

#pragma once

#include "std.h"

extern void buzzer_arch_init(void);

/** Play a square wave at freq Hz, 0 for silence */
extern void buzzer_arch_tone(uint16_t freq);
