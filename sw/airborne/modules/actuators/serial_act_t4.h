/*
 * Copyright (C) Paparazzi Team
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
 * @file "modules/sensors/ca_am7.h"
 * @author Alessandro Mancinelli
 * Uses a teensy 4.0 as fly by wire system. The teensy can control serial bus servos and KISS ESC providing real time telemetry.
 */

#ifndef SERIAL_ACT_T4
#define SERIAL_ACT_T4

#define START_BYTE_SERIAL_ACT_T4 0x9A  //1st start block identifier byte


#include "std.h"
#include <stdbool.h>
#include <stdlib.h>
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"


struct __attribute__((__packed__)) serial_act_t4_in {
    //ESC telemetry & error code
	int16_t motor_1_rpm_int; //RPM motor 1
	int16_t motor_2_rpm_int; //RPM motor 2
	int16_t motor_3_rpm_int; //RPM motor 3
	int16_t motor_4_rpm_int; //RPM motor 4
    int16_t motor_1_error_code_int; //ESC 1 error code 
    int16_t motor_2_error_code_int; //ESC 2 error code 
    int16_t motor_3_error_code_int; //ESC 3 error code 
    int16_t motor_4_error_code_int; //ESC 4 error code 
    int16_t motor_1_current_int; //ESC 1 current mA
    int16_t motor_2_current_int; //ESC 2 current mA
    int16_t motor_3_current_int; //ESC 3 current mA
    int16_t motor_4_current_int; //ESC 4 current mA   
    int16_t motor_1_voltage_int; //ESC 1 voltage mV
    int16_t motor_2_voltage_int; //ESC 2 voltage mV
    int16_t motor_3_voltage_int; //ESC 3 voltage mV
    int16_t motor_4_voltage_int; //ESC 4 voltage mV       
    //SERVOS telemetry & update rate 
	int16_t servo_1_angle_int; //Degrees * 100 
	int16_t servo_2_angle_int; //Degrees * 100 
	int16_t servo_3_angle_int; //Degrees * 100 
    int16_t servo_4_angle_int; //Degrees * 100 
    int16_t servo_5_angle_int; //Degrees * 100 
    int16_t servo_6_angle_int; //Degrees * 100 
    int16_t servo_7_angle_int; //Degrees * 100 
    int16_t servo_8_angle_int; //Degrees * 100 
    int16_t servo_9_angle_int; //Degrees * 100 
    int16_t servo_10_angle_int; //Degrees * 100 
	int16_t servo_1_update_time_us; //MicroSeconds
	int16_t servo_2_update_time_us; //MicroSeconds
	int16_t servo_3_update_time_us; //MicroSeconds
    int16_t servo_4_update_time_us; //MicroSeconds
    int16_t servo_5_update_time_us; //MicroSeconds
    int16_t servo_6_update_time_us; //MicroSeconds 
    int16_t servo_7_update_time_us; //MicroSeconds 
    int16_t servo_8_update_time_us; //MicroSeconds 
    int16_t servo_9_update_time_us; //MicroSeconds 
    int16_t servo_10_update_time_us; //MicroSeconds    
    //Rolling message in 
    float rolling_msg_in;
    uint8_t rolling_msg_in_id;   
    //CHECKSUM
    uint8_t checksum_in;
};

struct __attribute__((__packed__)) serial_act_t4_out {
    //ARM cmd
    int8_t motor_arm_int; //Arm motor boolean
    int8_t servo_arm_int; //Arm servo boolean
    //ESC cmd
    int16_t motor_1_dshot_cmd_int; //Motor cmd 0 - 1999
    int16_t motor_2_dshot_cmd_int; //Motor cmd 0 - 1999
    int16_t motor_3_dshot_cmd_int; //Motor cmd 0 - 1999
    int16_t motor_4_dshot_cmd_int; //Motor cmd 0 - 1999
    //Servo cmd
    int16_t servo_1_cmd_int; //Degrees * 100 
    int16_t servo_2_cmd_int; //Degrees * 100 
    int16_t servo_3_cmd_int; //Degrees * 100 
    int16_t servo_4_cmd_int; //Degrees * 100 
    int16_t servo_5_cmd_int; //Degrees * 100 
    int16_t servo_6_cmd_int; //Degrees * 100 
    int16_t servo_7_cmd_int; //Degrees * 100 
    int16_t servo_8_cmd_int; //Degrees * 100 
    int16_t servo_9_cmd_int; //Degrees * 100 
    int16_t servo_10_cmd_int; //Degrees * 100   
    //Rolling message out
    float rolling_msg_out;
    uint8_t rolling_msg_out_id;
    //CHECKSUM
    uint8_t checksum_out;
};

extern void serial_act_t4_init(void);
extern void serial_act_t4_event(void);

#endif

