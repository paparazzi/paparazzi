/*
 * Copyright (C) 2019 Xavier Paris
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/light/light_scheduler.h
 * @brief basic LED scheduler based on WS2812 RGB led driver
 *
 * @author Xavier Paris
 * @maintainer Gautier Hattenberger <gautier.hattenberger@enac.fr>
 */

#ifndef LIGHT_SCHEDULER_H
#define LIGHT_SCHEDULER_H

#include "std.h"

/** Light modes */
#define LIGHT_SCHEDULER_MODE_RAINBOW 0
#define LIGHT_SCHEDULER_MODE_COLOR   1
#define LIGHT_SCHEDULER_MODE_OFF     2
#define LIGHT_SCHEDULER_MODE_SPARKLE 3

extern uint8_t light_scheduler_mode;       ///< one of LIGHT_SCHEDULER_MODE_*
extern uint8_t light_scheduler_red;        ///< red value in color mode
extern uint8_t light_scheduler_green;      ///< green value in color mode
extern uint8_t light_scheduler_blue;       ///< blue value in color mode
extern uint8_t light_scheduler_brightness; ///< brightness scaling (255 = full)

/** Color presets and flags, order must match the values of the color setting
 *  Flags are shown as three equal bands along each strip
 */
enum light_scheduler_color_t {
  LIGHT_SCHEDULER_COLOR_CUSTOM,
  LIGHT_SCHEDULER_COLOR_RED,
  LIGHT_SCHEDULER_COLOR_GREEN,
  LIGHT_SCHEDULER_COLOR_BLUE,
  LIGHT_SCHEDULER_COLOR_WHITE,
  LIGHT_SCHEDULER_COLOR_YELLOW,
  LIGHT_SCHEDULER_COLOR_ORANGE,
  LIGHT_SCHEDULER_COLOR_CYAN,
  LIGHT_SCHEDULER_COLOR_MAGENTA,
  LIGHT_SCHEDULER_COLOR_PURPLE,
  LIGHT_SCHEDULER_COLOR_FLAG_ITALY,       ///< first flag, red | white | green
  LIGHT_SCHEDULER_COLOR_FLAG_NETHERLANDS, ///< red | white | blue
  LIGHT_SCHEDULER_COLOR_FLAG_FRANCE,      ///< blue | white | red
  LIGHT_SCHEDULER_COLOR_NB
};

#define LIGHT_SCHEDULER_COLOR_FLAG_FIRST LIGHT_SCHEDULER_COLOR_FLAG_ITALY

extern uint8_t light_scheduler_color;      ///< selected preset (light_scheduler_color_t)

extern void light_scheduler_init(void);
extern void light_scheduler_periodic(void);

/** Settings handler: apply a color preset to red/green/blue (or select a flag) and switch to color mode */
extern void light_scheduler_set_color(uint8_t color);

#endif /* LIGHT_SCHEDULER_H */
