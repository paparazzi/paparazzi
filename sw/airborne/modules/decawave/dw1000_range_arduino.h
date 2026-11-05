/** 
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com>
 * This file is part of paparazzi. See LICENCE file.
 * 
 * @file "modules/decawave/dw1000_range_arduino.h"
 * @author Fabien-B <fabien-b@github.com>
 * 
 * Driver to get ranging data from Decawave DW1000 modules connected to Arduino
 * Decawave DW1000 modules (http://www.decawave.com/products/dwm1000-module) are Ultra-Wide-Band devices that can be used for communication and ranging.
 * Especially, using 3 modules as anchors can provide data for a localization system based on trilateration.
 * The DW1000 is using a SPI connection, but an arduino-compatible board can be used with the library https://github.com/thotro/arduino-dw1000 to hyde the low level drivers and provide direct ranging informations.
 * 
 */

#ifndef DW1000_RANGE_ARDUINO_H
#define DW1000_RANGE_ARDUINO_H

extern void dw1000_range_arduino_init(void);
extern void dw1000_range_arduino_event(void);

#endif  // DW1000_RANGE_ARDUINO_H
