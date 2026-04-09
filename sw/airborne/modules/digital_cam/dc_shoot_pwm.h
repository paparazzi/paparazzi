/*
 * Copyright (C) 2025 Jean-Baptiste Forestier <jean-baptiste.forestier@enac.fr>
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
 * @file "modules/digital_cam/dc_shoot_pwm.h
 * @author Jean-Baptiste Forestier
 * Trigger using pwm cameras like MAPIR (https://www.mapir.camera/en-gb)
 * or ThermalCapture 2.0 from TeAx (https://thermalcapture.com/thermalcapture-2-0/)
 *
 *                       **** MAPIR ****
 * Duty cycle == 2000us => TRIGGER_ON
 * Duty cycle == 1500us => SD Unmount
 * Duty cycle == 1000us => TRIGGER_OFF
 * Image capture frequency = 1.5s for JPG and 2.5-3.0s RAW+JPG
 *
 * The quickest the 2000us command should be sent is about once every 1.5s as the camera cannot
 * capture JPG images more quickly than 1.5s. For RAW+JPG mode we recommend a 2.5-3.0s wait time.
 *
 * RAW images are used for capturing data for reflectance measurements, otherwise the pixels in the JPG are not usable.
 * im_freq_timer should therefore be > 1.5s (=>TRIGGER_CAMERA_CAPTURE_IMAGE_PERIOD = 1.5 seconds)
 *
 * NMEA GPS data are stored in metadata
 *
 *                   **** ThermalCapture ****
 * Duty cycle < 1500us => TRIGGER_OFF
 * Duty cycle > 1500us => TRIGGER_ON
 * Image capture frequency up to 30 Hz
 * Possibility to capture one frame per trigger in configurator : Trigger frame mode
 * Convert 14-bit data into temperature values :
 * High gain mode: temp [°C] = raw * 0.04 - 273.15
 * Low gain mode : temp [°C] = raw * 0.4  - 273.15
 *
 * NMEA GPS data are stored in metadata
 *
 *                   **** Trigger  system ****
 *
 * TIME            ────────────────────────────────────────────────────────────>
 *
 *     PWM             1000us         2000us         1000us         2000us
 *     Vcc             ┌────┐        ┌───────┐       ┌────┐        ┌───────┐
 *                     │ OFF│        │  ON   │       │ OFF│        │  ON   │
 *     0V          ────┘    └────────┘       └───────┘    └────────┘       └────
 * im_freq_timer:  ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 0 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 0 ↓↓↓↓↓↓↓↓↓↓
 * shutter_timer:                    ↓↓↓↓ 0                        ↓↓↓↓ 0
 *
 *                                     ↑                             ↑
 *                                  Picture 1                     Picture 2
 *
 * im_freq_timer should therefore be > 1.5s (=>TRIGGER_CAMERA_CAPTURE_IMAGE_PERIOD = 1.5 seconds)
 *
 */


#ifndef DC_SHOOT_PWM_H
#define DC_SHOOT_PWM_H

#include "std.h"

void dc_shoot_pwm_init(void);
void dc_shoot_pwm_periodic(void);

#endif

