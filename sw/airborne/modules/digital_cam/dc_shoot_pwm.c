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
 * @file "modules/digital_cam/dc_shoot_pwm.c"
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
 */

#include "modules/digital_cam/dc_shoot_pwm.h"

// Include Standard Camera Control Interface
#include "dc.h"

#include "generated/airframe.h"
#include "generated/modules.h"
#include "modules/actuators/actuators.h"


/** how long to push shutter in seconds */
#ifndef DC_CAM_PWM_SHUTTER_DELAY
#define DC_CAM_PWM_SHUTTER_DELAY 0.1
#endif

/** Max PWM Value */
#ifndef DC_CAM_PWM_ON_VALUE
#define DC_CAM_PWM_ON_VALUE MAX_PPRZ
#endif

/** Min PWM Value */
#ifndef DC_CAM_PWM_OFF_VALUE
#define DC_CAM_PWM_OFF_VALUE MIN_PPRZ
#endif

/** Servo destination */
#ifndef DC_CAM_PWM_SERVO
#define DC_CAM_PWM_SERVO DC_CAM_TRIGGER
#endif

#define CamActuatorSet(_a, _v) ActuatorSet(_a, _v)

/**
 * Timer used for Shutter delay control
 */
static uint8_t shutter_timer;

/**
 * Initialization function
 */
void dc_shoot_pwm_init(void)
{
  shutter_timer = 0;
  CamActuatorSet(DC_CAM_PWM_SERVO, DC_CAM_PWM_OFF_VALUE);
}

/**
 * Periodic function to send data
 */
void dc_shoot_pwm_periodic(void)
{
  // Manage the shutter opening time each DC_CAM_PWM_SHUTTER_DELAY seconds
  if (shutter_timer == 0) {
    CamActuatorSet(DC_CAM_PWM_SERVO, DC_CAM_PWM_OFF_VALUE);
  } else {
    shutter_timer--;
  }

  // Common DC Periodic task
  dc_periodic();
}

/* Command The Camera */
void dc_send_command(uint8_t cmd)
{
  if (cmd == DC_SHOOT) {
    CamActuatorSet(DC_CAM_PWM_SERVO, DC_CAM_PWM_ON_VALUE);
    shutter_timer = DC_CAM_PWM_SHUTTER_DELAY * DC_SHOOT_PWM_PERIODIC_FREQ;
    dc_send_shot_position();
  }

  // call command send_command function
  dc_send_command_common(cmd);
}

