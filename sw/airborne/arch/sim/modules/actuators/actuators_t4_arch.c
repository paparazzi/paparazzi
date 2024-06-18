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
}

void actuators_t4_commit(void) {
    printf("%d, %d \n", actuators_t4_values[0], actuators_t4_values[1]);
    // set values to the act_t4_out struct
    // from actuators_t4_values

    // *** actuator id configuration???
    // actuators_t4_values[driver_no] = actuators[servo_idx].val;

    // also motor scale (0-1999)
    // also servo scale (deg*100)
    // 20 - 1000

    // get_servo_min_MYT4(_idx);
    // get_servo_max_MTT4(_idx);
    // get_servo_idx_MYT4(_idx_driver); returns servo idx
    // DRIVER_NO: <servo no="DRIVER_NO"/> and SERVO_IDX: defined order in AF config

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
    // AbiSendMsgSERIAL_ACT_T4_OUT();
    AbiSendMsgSERIAL_ACT_T4_OUT(ABI_SERIAL_ACT_T4_OUT_ID, &myserial_act_t4_out_local, &serial_act_t4_extra_data_out_local[0]);
}

//void send_t4_abi_msg(void){
//
//        //Motor cmds
//            //Arm motor:
//            myserial_act_t4_out_local.motor_arm_int = 1;
//            //Arm servos:
//            myserial_act_t4_out_local.servo_arm_int = 1;
//
//            //Bound motor values before submitting:
//            Bound(overactuated_mixing.commands[0], 0, MAX_PPRZ);
//            Bound(overactuated_mixing.commands[1], 0, MAX_PPRZ);
//            Bound(overactuated_mixing.commands[2], 0, MAX_PPRZ);
//            Bound(overactuated_mixing.commands[3], 0, MAX_PPRZ);
//            // Bound actuator values for the FBW system before submitting:
//            Bound(overactuated_mixing.commands[4],(OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_EL_1_ZERO_VALUE)*K_ppz_angle_el,(OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_EL_1_ZERO_VALUE)*K_ppz_angle_el);
//            Bound(overactuated_mixing.commands[5],(OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_EL_2_ZERO_VALUE)*K_ppz_angle_el,(OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_EL_2_ZERO_VALUE)*K_ppz_angle_el);
//            Bound(overactuated_mixing.commands[6],(OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_EL_3_ZERO_VALUE)*K_ppz_angle_el,(OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_EL_3_ZERO_VALUE)*K_ppz_angle_el);
//            Bound(overactuated_mixing.commands[7],(OVERACTUATED_MIXING_SERVO_EL_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_EL_4_ZERO_VALUE)*K_ppz_angle_el,(OVERACTUATED_MIXING_SERVO_EL_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_EL_4_ZERO_VALUE)*K_ppz_angle_el);
//
//            Bound(overactuated_mixing.commands[8],(OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_1_ZERO_VALUE)*K_ppz_angle_az,(OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_1_ZERO_VALUE)*K_ppz_angle_az);
//            Bound(overactuated_mixing.commands[9],(OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_2_ZERO_VALUE)*K_ppz_angle_az,(OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_2_ZERO_VALUE)*K_ppz_angle_az);
//            Bound(overactuated_mixing.commands[10],(OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_3_ZERO_VALUE)*K_ppz_angle_az,(OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_3_ZERO_VALUE)*K_ppz_angle_az);
//            Bound(overactuated_mixing.commands[11],(OVERACTUATED_MIXING_SERVO_AZ_MIN_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_4_ZERO_VALUE)*K_ppz_angle_az,(OVERACTUATED_MIXING_SERVO_AZ_MAX_ANGLE - OVERACTUATED_MIXING_SERVO_AZ_4_ZERO_VALUE)*K_ppz_angle_az);
//
//            //Motors cmds:
//                myserial_act_t4_out_local.motor_1_dshot_cmd_int =  (int16_t) (overactuated_mixing.commands[0] * MAX_DSHOT_VALUE / MAX_PPRZ);
//                myserial_act_t4_out_local.motor_2_dshot_cmd_int =  (int16_t) (overactuated_mixing.commands[1] * MAX_DSHOT_VALUE / MAX_PPRZ);
//                myserial_act_t4_out_local.motor_3_dshot_cmd_int =  (int16_t) (overactuated_mixing.commands[2] * MAX_DSHOT_VALUE / MAX_PPRZ);
//                myserial_act_t4_out_local.motor_4_dshot_cmd_int =  (int16_t) (overactuated_mixing.commands[3] * MAX_DSHOT_VALUE / MAX_PPRZ);
//
//            //Elevator servos cmd:
//            myserial_act_t4_out_local.servo_2_cmd_int = (int16_t) ( ( overactuated_mixing.commands[4] / K_ppz_angle_el ) * 100 * FBW_T4_K_RATIO_GEAR_EL * 180/M_PI );
//            myserial_act_t4_out_local.servo_6_cmd_int = (int16_t) ( ( overactuated_mixing.commands[5] / K_ppz_angle_el ) * 100 * FBW_T4_K_RATIO_GEAR_EL * 180/M_PI );
//            myserial_act_t4_out_local.servo_8_cmd_int = (int16_t) ( ( overactuated_mixing.commands[6] / K_ppz_angle_el ) * 100 * FBW_T4_K_RATIO_GEAR_EL * 180/M_PI );
//            myserial_act_t4_out_local.servo_4_cmd_int = (int16_t) ( ( overactuated_mixing.commands[7] / K_ppz_angle_el ) * 100 * FBW_T4_K_RATIO_GEAR_EL * 180/M_PI );
//            //Azimuth servos cmd:
//            myserial_act_t4_out_local.servo_1_cmd_int = (int16_t) ( ( overactuated_mixing.commands[8] / K_ppz_angle_az ) * 100 * FBW_T4_K_RATIO_GEAR_AZ * 180/M_PI );
//            myserial_act_t4_out_local.servo_5_cmd_int = (int16_t) ( ( overactuated_mixing.commands[9] / K_ppz_angle_az ) * 100 * FBW_T4_K_RATIO_GEAR_AZ * 180/M_PI );
//            myserial_act_t4_out_local.servo_7_cmd_int = (int16_t) ( ( -overactuated_mixing.commands[10] / K_ppz_angle_az ) * 100 * FBW_T4_K_RATIO_GEAR_AZ * 180/M_PI );
//            myserial_act_t4_out_local.servo_3_cmd_int = (int16_t) ( ( -overactuated_mixing.commands[11] / K_ppz_angle_az ) * 100 * FBW_T4_K_RATIO_GEAR_AZ * 180/M_PI );
//            //Aileron servos PWM cmd:
//            myserial_act_t4_out_local.servo_9_cmd_int = (int16_t) (indi_u[14] * 100.0 * 180.0/M_PI );
//            myserial_act_t4_out_local.servo_10_cmd_int = (int16_t) (indi_u[14] * 100.0 * 180.0/M_PI );
//
//
//        //Assign extra data:
//        serial_act_t4_extra_data_out_local[0] = FBW_T4_AILERONS_FIRST_ORD_DEN;
//        serial_act_t4_extra_data_out_local[1] = FBW_T4_AILERONS_FIRST_ORD_NUM;
//
//        serial_act_t4_extra_data_out_local[2] = max_pwm_servo_9;
//        serial_act_t4_extra_data_out_local[3] = min_pwm_servo_9;
//        serial_act_t4_extra_data_out_local[4] = neutral_pwm_servo_9;
//        serial_act_t4_extra_data_out_local[5] = FBW_T4_SERVO_9_MIN_ANGLE_DEG;
//        serial_act_t4_extra_data_out_local[6] = FBW_T4_SERVO_9_MAX_ANGLE_DEG;
//        serial_act_t4_extra_data_out_local[7] = FBW_T4_SERVO_9_DELAY_TS;
//
//        serial_act_t4_extra_data_out_local[8] = FBW_T4_AILERONS_FIRST_ORD_DEN;
//        serial_act_t4_extra_data_out_local[9] = FBW_T4_AILERONS_FIRST_ORD_NUM;
//
//        serial_act_t4_extra_data_out_local[10] = max_pwm_servo_10;
//        serial_act_t4_extra_data_out_local[11] = min_pwm_servo_10;
//        serial_act_t4_extra_data_out_local[12] = neutral_pwm_servo_10;
//        serial_act_t4_extra_data_out_local[13] = FBW_T4_SERVO_10_MIN_ANGLE_DEG;
//        serial_act_t4_extra_data_out_local[14] = FBW_T4_SERVO_10_MAX_ANGLE_DEG;
//        serial_act_t4_extra_data_out_local[15] = FBW_T4_SERVO_10_DELAY_TS;
//
//        //SEND MESSAGE:
//        AbiSendMsgSERIAL_ACT_T4_OUT(ABI_SERIAL_ACT_T4_OUT_ID, &myserial_act_t4_out_local, &serial_act_t4_extra_data_out_local[0]);
//
//}