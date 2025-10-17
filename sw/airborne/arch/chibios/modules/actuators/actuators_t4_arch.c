/*
 * Copyright (C) 2024 The Paparazzi Team
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
 */
/**
 * @file arch/chibios/modules/actuators/actuators_t4_arch.c
 * @brief Actuator interface for T4 driver
 * @author Sunyou Hwang
 */

#include "modules/actuators/actuators_t4_arch.h"
#include "modules/actuators/actuators_t4.h"

#include "modules/core/abi.h"

#ifndef ACTUATORS_T4_REFRESH_FREQUENCY
#define ACTUATORS_T4_REFRESH_FREQUENCY 200;
#endif

#ifndef ACTUATORS_T4_NUM_MAX_SERVOS
#define ACTUATORS_T4_NUM_MAX_SERVOS 12
#endif

#ifndef ACTUATORS_T4_NUM_MAX_ESCS
#define ACTUATORS_T4_NUM_MAX_ESCS 4
#endif

/**
 * Print the configuration variables from the header
 */
PRINT_CONFIG_VAR(ACTUATORS_T4_NB)

int32_t actuators_t4_values[ACTUATORS_T4_NB];

struct ActuatorsT4Out actuators_t4_out_local;
float actuators_t4_extra_data_out_local[255] __attribute__((aligned));

void actuators_t4_arch_init(void) {
    /* Arm ESCs and servos by default, Note that only armed servo directly provide force needed for e.g. landing gear */ 
    //TODO: Add option set or check is disarmed / is armed as a default value per actuator
    actuators_t4_out_local.esc_arm = (1 << ACTUATORS_T4_NUM_MAX_ESCS) -1;
    actuators_t4_out_local.servo_arm = (1 << ACTUATORS_T4_NUM_MAX_SERVOS) - 1;

    /* comm_refresh_frequency [Hz] */
    actuators_t4_extra_data_out_local[0] = ACTUATORS_T4_REFRESH_FREQUENCY;
}

void actuators_t4_commit(void) {
    /*
    For developers wanting to improve here note that:
    DRIVER_NO: <servo no="DRIVER_NO"/> and SERVO_IDX: defined order in AF config
    actuators_t4_values[driver_no] = actuators[servo_idx].val;
    get_servo_min_T4(_idx);
    get_servo_max_T4(_idx);
    get_servo_idx_T4(_idx_driver); returns servo idx
    */

    /* Yes, indeed it could well be that we waste two bytes here, feel free to improve
       Servos connected to pin outputs 1~10 (Serial Bus), 
       Note that connector 0 does not exist on a T4 actuators board, 
       indeed we waste two bytes here, feel free to improve.
    */
    actuators_t4_out_local.servo_1_cmd = (int16_t)(actuators_t4_values[1]);
    actuators_t4_out_local.servo_2_cmd = (int16_t)(actuators_t4_values[2]);
    actuators_t4_out_local.servo_3_cmd = (int16_t)(actuators_t4_values[3]);
    actuators_t4_out_local.servo_4_cmd = (int16_t)(actuators_t4_values[4]);
    actuators_t4_out_local.servo_5_cmd = (int16_t)(actuators_t4_values[5]);
    actuators_t4_out_local.servo_6_cmd = (int16_t)(actuators_t4_values[6]);
    actuators_t4_out_local.servo_7_cmd = (int16_t)(actuators_t4_values[7]);
    actuators_t4_out_local.servo_8_cmd = (int16_t)(actuators_t4_values[8]);
    actuators_t4_out_local.servo_9_cmd = (int16_t)(actuators_t4_values[9]);
    actuators_t4_out_local.servo_10_cmd = (int16_t)(actuators_t4_values[10]);

    /* PWM Servos (or ESCs) connected to pin outputs 11~12 (PWM) */
    actuators_t4_out_local.servo_11_cmd = (int16_t)(actuators_t4_values[11]);
    actuators_t4_out_local.servo_12_cmd = (int16_t)(actuators_t4_values[12]);

    /* DShot capable ESC's connected to pin outputs 13~16 (DShot) */
    actuators_t4_out_local.esc_1_dshot_cmd = (int16_t)(actuators_t4_values[13]);
    actuators_t4_out_local.esc_2_dshot_cmd = (int16_t)(actuators_t4_values[14]);
    actuators_t4_out_local.esc_3_dshot_cmd = (int16_t)(actuators_t4_values[15]);
    actuators_t4_out_local.esc_4_dshot_cmd = (int16_t)(actuators_t4_values[16]);

    // Send ABI message
    AbiSendMsgACTUATORS_T4_OUT(ABI_ACTUATORS_T4_OUT_ID, &actuators_t4_out_local, &actuators_t4_extra_data_out_local[0]);
}
