/*
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com>
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

/** @file "modules/decawave/dw_arduino_periph.h"
 * @author Fabien-B <fabien-b@github.com>
 * Driver to get ranging data from Decawave DW1000 modules connected to Arduino

      Decawave DW1000 modules (http://www.decawave.com/products/dwm1000-module) are Ultra-Wide-Band devices that can be used for communication and ranging.
      Especially, using 3 modules as anchors can provide data for a localization system based on trilateration.
      The DW1000 is using a SPI connection, but an arduino-compatible board can be used with the library https://github.com/thotro/arduino-dw1000 to hyde the low level drivers and provide direct ranging informations.

 */

#ifndef DW_ARDUINO_PERIPH_H
#define DW_ARDUINO_PERIPH_H

extern void dw1000_arduino_periph_init(void);
extern void dw1000_arduino_periph_event(void);

#endif  // DW_ARDUINO_PERIPH_H
