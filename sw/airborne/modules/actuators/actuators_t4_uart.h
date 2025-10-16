/*
 * Copyright (C) 2024 The Paparazzi Team
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
 * @file "modules/actuators/actuators_t4_uart.h"
 * @author Alessandro Mancinelli, Sunyou Hwang, OpenUAS
 * @brief Uses a T4 Actuators Board as fly by wire system. This Board can control serial bus servos, ESC's and PWM servos, with as big benefir providing real time telemetry in return into the autopilot state.
 * Read more on how to create your own T4 Board here: https://github.com/tudelft/t4_actuators_board/
 * 
 */

#ifndef ACTUATORS_T4_UART_H
#define ACTUATORS_T4_UART_H

#define START_BYTE_ACTUATORS_T4 0x9A  //1st start block identifier byte

#include "std.h"
#include <stdbool.h>
#include <stdlib.h>
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"

struct __attribute__((__packed__)) ActuatorsT4In {
  /* ESCs telemetry & error code */

  /* RPM output from ESC's to flightcontroller */
  int16_t esc_1_rpm;
  int16_t esc_2_rpm;
  int16_t esc_3_rpm;
  int16_t esc_4_rpm;

  /* Error code from ESC's to flightcontroller */
  int16_t esc_1_error_code;
  int16_t esc_2_error_code;
  int16_t esc_3_error_code;
  int16_t esc_4_error_code;

  /* Current (mA) output from ESC's to flightcontroller */
  int16_t esc_1_current;
  int16_t esc_2_current;
  int16_t esc_3_current;
  int16_t esc_4_current;

  /* Voltage (mV) output from ESC's to flightcontroller */
  int16_t esc_1_voltage;
  int16_t esc_2_voltage;
  int16_t esc_3_voltage;
  int16_t esc_4_voltage;

  /* Temperature (Celcius) not implemented, since it will take away bus bandwidth, feel free to add if really nned it, preferably with low message rate */
  //int16_t esc_1_temperature; // FIXME Kelvin as SI it is might be better?
  //int16_t esc_2_temperature;
  //int16_t esc_3_temperature;
  //int16_t esc_4_temperature;

  /* SERVOS telemetry where angle is in Degrees times 100 */
  int16_t servo_1_angle;
  int16_t servo_2_angle;
  int16_t servo_3_angle;
  int16_t servo_4_angle;
  int16_t servo_5_angle;
  int16_t servo_6_angle;
  int16_t servo_7_angle;
  int16_t servo_8_angle;
  int16_t servo_9_angle;
  int16_t servo_10_angle;
  /* Note that PWM servos nor PWM ESC's give feedback in real life. Servos have estimated angles */
  int16_t servo_11_angle;
  int16_t servo_12_angle;

  /* Servo load in UNDEFINED Units */
  int16_t servo_1_load;
  int16_t servo_2_load;
  int16_t servo_3_load;
  int16_t servo_4_load;
  int16_t servo_5_load;
  int16_t servo_6_load;
  int16_t servo_7_load;
  int16_t servo_8_load;
  int16_t servo_9_load;
  int16_t servo_10_load;
  /* Note that PWM servos or ESC (11 and 12) give no real feedback on load */
  //int16_t servo_11_load;
  //int16_t servo_12_load;

  uint16_t bitmask_servo_health; //Bitmask of servo health status

  /* Rolling message in  */
  float rolling_msg_in;
  uint8_t rolling_msg_in_id;

  /* CHECKSUM for the data packages */
  uint8_t checksum_in;
};

struct __attribute__((__packed__)) ActuatorsT4Out {
  /* Arming Command */
  uint8_t esc_arm; //Arm ESC boolean in bitfield
  uint16_t servo_arm; //Arm servo boolean in bitfield

  /* ESC cmd values 0 - 1999 */
  int16_t esc_1_dshot_cmd;
  int16_t esc_2_dshot_cmd;
  int16_t esc_3_dshot_cmd;
  int16_t esc_4_dshot_cmd;

  /* Servo cmd in Degrees * 100 */
  int16_t servo_1_cmd;
  int16_t servo_2_cmd;
  int16_t servo_3_cmd;
  int16_t servo_4_cmd;
  int16_t servo_5_cmd;
  int16_t servo_6_cmd;
  int16_t servo_7_cmd;
  int16_t servo_8_cmd;
  int16_t servo_9_cmd;
  int16_t servo_10_cmd;
  int16_t servo_11_cmd;
  int16_t servo_12_cmd;

  /* Rolling message out */
  float rolling_msg_out;
  uint8_t rolling_msg_out_id;
  /* CHECKSUM */
  uint8_t checksum_out;
};

extern void actuators_t4_uart_init(void);
extern void actuators_t4_uart_parse_msg_in(void);
extern void actuators_t4_uart_event(void);

#endif /* ACTUATORS_T4_UART_H */

