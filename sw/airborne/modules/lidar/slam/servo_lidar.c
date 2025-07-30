/*
 * Copyright (C) 2025 Alejandro Rochas <alrochas@ucm.es>
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

/** @file modules/lidar/slam/servo_lidar.c
 *  @brief driver for the servo to move the lidar
 *
 */

 // Depends on the lidar (it doesnt make any sense without it)

#include "mcu_periph/uart.h"
#include "modules/core/abi.h"
#include "modules/datalink/downlink.h"
#include "servo_lidar.h"
#include "modules/lidar/tfmini.h"



#define MOTOR_SPEED 5
static uint32_t last_time = 0;

bool enable_servo = false;
float motor_speed = MOTOR_SPEED;
struct TFMiniServo tf_servo;

struct ServoLidar servoLidar;

void servoLidar_init(void) {
  servoLidar.enabled = false;
  servoLidar.speed = MOTOR_SPEED; // default speed
  servoLidar.position = 0;
  servoLidar.angle = 0;
  servoLidar.direction = 0;
  servoLidar.last_update = 0;
}


#ifdef COMMAND_SERVO
void servoLidar_periodic(void) {
  if (get_sys_time_msec() > servoLidar.last_update + servoLidar.speed) {
    servoLidar.last_update = get_sys_time_msec();
    
    if(servoLidar.enabled) {
      // Update servo position
      servoLidar.position += (servoLidar.direction == 0) ? 100 : -100;
      if (servoLidar.position >= MAX_PPRZ*0.8 || servoLidar.position <= -MAX_PPRZ*0.8) {
          servoLidar.direction ^= 1;
      }
      
      // Set servo command
      commands[COMMAND_SERVO] = servoLidar.position;
      servoLidar.angle = PWM2ANGLE(servoLidar.position);
      
      // Send ABI message
      AbiSendMsgOBSTACLE_DETECTION(AGL_LIDAR_TFMINI_ID, tfmini.distance, servoLidar.angle, 0);
    }
    else {
      commands[COMMAND_SERVO] = 0; // Center servo when disabled
    }
  }
}
#else
void servoLidar_periodic(void) {
  // No servo functionality; do nothing
  #error "You need to define COMMAND_SERVO to use servoLidar_periodic()"
}
#endif