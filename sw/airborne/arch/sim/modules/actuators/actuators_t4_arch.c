/*
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
 */
/**
 * @file arch/sim/modules/actuators/actuators_t4_arch.c
 * Actuator interface for T4 driver
 * @author Sunyou Hwang
 */
#include <stdio.h>
#include "modules/actuators/actuators_t4_arch.h"
#include "modules/actuators/actuators_myt4.h"

#include "modules/core/abi.h"

/**
 * Print the configuration variables from the header
 */
PRINT_CONFIG_VAR(ACTUATORS_T4_NB)

int32_t actuators_t4_values[ACTUATORS_T4_NB];

struct serial_act_t4_out myserial_act_t4_out_local;
float serial_act_t4_extra_data_out_local[255] __attribute__((aligned));


void actuators_t4_arch_init(void) {
    // ARM motors and servos by default
    myserial_act_t4_out_local.motor_arm_int = 1;
    myserial_act_t4_out_local.servo_arm_int = 1;
}

void actuators_t4_commit(void) {
    //Servos 1~8
    myserial_act_t4_out_local.servo_1_cmd_int = (int16_t)(actuators_t4_values[0]);
    myserial_act_t4_out_local.servo_2_cmd_int = (int16_t)(actuators_t4_values[1]);
    myserial_act_t4_out_local.servo_3_cmd_int = (int16_t)(actuators_t4_values[2]);
    myserial_act_t4_out_local.servo_4_cmd_int = (int16_t)(actuators_t4_values[3]);
    myserial_act_t4_out_local.servo_5_cmd_int = (int16_t)(actuators_t4_values[4]);
    myserial_act_t4_out_local.servo_6_cmd_int = (int16_t)(actuators_t4_values[5]);
    myserial_act_t4_out_local.servo_7_cmd_int = (int16_t)(actuators_t4_values[6]);
    myserial_act_t4_out_local.servo_8_cmd_int = (int16_t)(actuators_t4_values[7]);
    //Servos 9~10
    myserial_act_t4_out_local.servo_9_cmd_int = (int16_t)(actuators_t4_values[8]);
    myserial_act_t4_out_local.servo_10_cmd_int = (int16_t)(actuators_t4_values[9]);

    //Motors
    myserial_act_t4_out_local.motor_1_dshot_cmd_int =  (int16_t)(actuators_t4_values[10]);
    myserial_act_t4_out_local.motor_2_dshot_cmd_int =  (int16_t)(actuators_t4_values[11]);
    myserial_act_t4_out_local.motor_3_dshot_cmd_int =  (int16_t)(actuators_t4_values[12]);
    myserial_act_t4_out_local.motor_4_dshot_cmd_int =  (int16_t)(actuators_t4_values[13]);

    // send abi msg
    AbiSendMsgSERIAL_ACT_T4_OUT(ABI_SERIAL_ACT_T4_OUT_ID, &myserial_act_t4_out_local, &serial_act_t4_extra_data_out_local[0]);
}