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
 * @file "modules/actuators/actuators_t4_uart.c"
 * @author Alessandro Mancinelli, Sunyou Hwang, OpenUAS
 * @brief Uses a T4 Actuators Board as fly by wire system. This Board can control serial bus servos, ESC's and PWM servos, with as big benefir providing real time telemetry in return into the autopilot state.
 * Read more on how to create your own T4 Board here: https://github.com/tudelft/t4_actuators_board/
 */

#include "pprzlink/pprz_transport.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include <time.h>
#include <sys/time.h>
#include "modules/core/abi.h"
#include "modules/actuators/actuators_t4_uart.h"

/* Variables for outbound packet */
static abi_event ACTUATORS_T4_OUT;
uint8_t actuators_t4_out_msg_id;
struct ActuatorsT4Out actuators_t4_out;
float actuators_t4_extra_data_out[255]__attribute__((aligned));

/* Variables for outbound packet */
struct ActuatorsT4In actuators_t4_in;
float actuators_t4_extra_data_in[255]__attribute__((aligned));

uint16_t actuators_t4_buf_in_cnt = 0;
uint32_t actuators_t4_missed_packets_in = 0;
uint32_t actuators_t4_received_packets = 0;
uint16_t actuators_t4_message_frequency_in = 0;
float actuators_t4_last_ts = 0;

static uint8_t actuators_t4_msg_buf_in[sizeof(struct ActuatorsT4In)*2]__attribute__((aligned));   ///< The message buffer for the device chosen to be 2* message_size total


#if PERIODIC_TELEMETRY

#include "modules/datalink/telemetry.h"

static void actuators_t4_downlink(struct transport_tx *trans, struct link_device *dev) {
    int16_t esc_1_rpm_telemetry = actuators_t4_in.esc_1_rpm;
    int16_t esc_2_rpm_telemetry = actuators_t4_in.esc_2_rpm;
    int16_t esc_3_rpm_telemetry = actuators_t4_in.esc_3_rpm;
    int16_t esc_4_rpm_telemetry = actuators_t4_in.esc_4_rpm;

    int16_t esc_1_error_code_telemetry = actuators_t4_in.esc_1_error_code;
    int16_t esc_2_error_code_telemetry = actuators_t4_in.esc_2_error_code;
    int16_t esc_3_error_code_telemetry = actuators_t4_in.esc_3_error_code;
    int16_t esc_4_error_code_telemetry = actuators_t4_in.esc_4_error_code;

    int16_t esc_1_current_telemetry = actuators_t4_in.esc_1_current;
    int16_t esc_2_current_telemetry = actuators_t4_in.esc_2_current;
    int16_t esc_3_current_telemetry = actuators_t4_in.esc_3_current;
    int16_t esc_4_current_telemetry = actuators_t4_in.esc_4_current;

    int16_t esc_1_voltage_telemetry = actuators_t4_in.esc_1_voltage;
    int16_t esc_2_voltage_telemetry = actuators_t4_in.esc_2_voltage;
    int16_t esc_3_voltage_telemetry = actuators_t4_in.esc_3_voltage;
    int16_t esc_4_voltage_telemetry = actuators_t4_in.esc_4_voltage;

    // Not used, and if so maybe to uint16_t in Kelvin not Celcius
    //int16_t esc_1_temperature_telemetry = actuators_t4_in.esc_1_temperature;
    //int16_t esc_2_temperature_telemetry = actuators_t4_in.esc_2_temperature;
    //int16_t esc_3_temperature_telemetry = actuators_t4_in.esc_3_temperature;
    //int16_t esc_4_temperature_telemetry = actuators_t4_in.esc_4_temperature;

    int16_t servo_1_angle_telemetry = actuators_t4_in.servo_1_angle;
    int16_t servo_2_angle_telemetry = actuators_t4_in.servo_2_angle;
    int16_t servo_3_angle_telemetry = actuators_t4_in.servo_3_angle;
    int16_t servo_4_angle_telemetry = actuators_t4_in.servo_4_angle;
    int16_t servo_5_angle_telemetry = actuators_t4_in.servo_5_angle;
    int16_t servo_6_angle_telemetry = actuators_t4_in.servo_6_angle;
    int16_t servo_7_angle_telemetry = actuators_t4_in.servo_7_angle;
    int16_t servo_8_angle_telemetry = actuators_t4_in.servo_8_angle;
    int16_t servo_9_angle_telemetry = actuators_t4_in.servo_9_angle;
    int16_t servo_10_angle_telemetry = actuators_t4_in.servo_10_angle;
    int16_t servo_11_angle_telemetry = actuators_t4_in.servo_11_angle;
    int16_t servo_12_angle_telemetry = actuators_t4_in.servo_12_angle;

    int16_t servo_1_load_telemetry = actuators_t4_in.servo_1_load;
    int16_t servo_2_load_telemetry = actuators_t4_in.servo_2_load;
    int16_t servo_3_load_telemetry = actuators_t4_in.servo_3_load;
    int16_t servo_4_load_telemetry = actuators_t4_in.servo_4_load;
    int16_t servo_5_load_telemetry = actuators_t4_in.servo_5_load;
    int16_t servo_6_load_telemetry = actuators_t4_in.servo_6_load;
    int16_t servo_7_load_telemetry = actuators_t4_in.servo_7_load;
    int16_t servo_8_load_telemetry = actuators_t4_in.servo_8_load;
    int16_t servo_9_load_telemetry = actuators_t4_in.servo_9_load;
    int16_t servo_10_load_telemetry = actuators_t4_in.servo_10_load;
    uint16_t bitmask_servo_health_telemetry = actuators_t4_in.bitmask_servo_health;

    float rolling_msg_in_telemetry = actuators_t4_in.rolling_msg_in;
    uint8_t rolling_msg_in_id_telemetry = actuators_t4_in.rolling_msg_in_id; 

    pprz_msg_send_ACTUATORS_T4_IN(trans, dev, AC_ID, 
                &esc_1_rpm_telemetry, &esc_2_rpm_telemetry, &esc_3_rpm_telemetry, &esc_4_rpm_telemetry,
                &servo_1_angle_telemetry, &servo_2_angle_telemetry, &servo_3_angle_telemetry, &servo_4_angle_telemetry,
                &servo_5_angle_telemetry, &servo_6_angle_telemetry, &servo_7_angle_telemetry, &servo_8_angle_telemetry,
                &servo_9_angle_telemetry, &servo_10_angle_telemetry, &servo_11_angle_telemetry, &servo_12_angle_telemetry,
                &actuators_t4_missed_packets_in, &actuators_t4_message_frequency_in,
                &rolling_msg_in_telemetry, &rolling_msg_in_id_telemetry,
                &esc_1_error_code_telemetry, &esc_2_error_code_telemetry, &esc_3_error_code_telemetry, &esc_4_error_code_telemetry,
                &servo_1_load_telemetry, &servo_2_load_telemetry, &servo_3_load_telemetry, &servo_4_load_telemetry,
                &servo_5_load_telemetry, &servo_6_load_telemetry, &servo_7_load_telemetry, &servo_8_load_telemetry,
                &servo_9_load_telemetry, &servo_10_load_telemetry,
                &bitmask_servo_health_telemetry,
                &esc_1_current_telemetry, &esc_2_current_telemetry, &esc_3_current_telemetry, &esc_4_current_telemetry,
                &esc_1_voltage_telemetry, &esc_2_voltage_telemetry, &esc_3_voltage_telemetry, &esc_4_voltage_telemetry);
}

    static void actuators_t4_uplink(struct transport_tx *trans, struct link_device *dev) {
    
    uint8_t esc_arm_telemetry = actuators_t4_out.esc_arm;
    uint16_t servo_arm_telemetry = actuators_t4_out.servo_arm;
    int16_t esc_1_dshot_cmd_telemetry = actuators_t4_out.esc_1_dshot_cmd;
    int16_t esc_2_dshot_cmd_telemetry = actuators_t4_out.esc_2_dshot_cmd;
    int16_t esc_3_dshot_cmd_telemetry = actuators_t4_out.esc_3_dshot_cmd;
    int16_t esc_4_dshot_cmd_telemetry = actuators_t4_out.esc_4_dshot_cmd;

    int16_t servo_1_angle_cmd_telemetry = actuators_t4_out.servo_1_cmd;
    int16_t servo_2_angle_cmd_telemetry = actuators_t4_out.servo_2_cmd;
    int16_t servo_3_angle_cmd_telemetry = actuators_t4_out.servo_3_cmd;
    int16_t servo_4_angle_cmd_telemetry = actuators_t4_out.servo_4_cmd;
    int16_t servo_5_angle_cmd_telemetry = actuators_t4_out.servo_5_cmd;
    int16_t servo_6_angle_cmd_telemetry = actuators_t4_out.servo_6_cmd;
    int16_t servo_7_angle_cmd_telemetry = actuators_t4_out.servo_7_cmd;
    int16_t servo_8_angle_cmd_telemetry = actuators_t4_out.servo_8_cmd;
    int16_t servo_9_angle_cmd_telemetry = actuators_t4_out.servo_9_cmd;
    int16_t servo_10_angle_cmd_telemetry = actuators_t4_out.servo_10_cmd;
    int16_t servo_11_angle_cmd_telemetry = actuators_t4_out.servo_11_cmd;
    int16_t servo_12_angle_cmd_telemetry = actuators_t4_out.servo_12_cmd;

    float rolling_msg_out_telemetry = actuators_t4_out.rolling_msg_out;
    uint8_t rolling_msg_out_id_telemetry = actuators_t4_out.rolling_msg_out_id;

    pprz_msg_send_ACTUATORS_T4_OUT(trans, dev, AC_ID, 
                &esc_arm_telemetry, &servo_arm_telemetry,
                &esc_1_dshot_cmd_telemetry, &esc_2_dshot_cmd_telemetry, &esc_3_dshot_cmd_telemetry, &esc_4_dshot_cmd_telemetry,
                &servo_1_angle_cmd_telemetry, &servo_2_angle_cmd_telemetry, &servo_3_angle_cmd_telemetry, &servo_4_angle_cmd_telemetry,
                &servo_5_angle_cmd_telemetry, &servo_6_angle_cmd_telemetry, &servo_7_angle_cmd_telemetry, &servo_8_angle_cmd_telemetry,
                &servo_9_angle_cmd_telemetry, &servo_10_angle_cmd_telemetry, &servo_11_angle_cmd_telemetry, &servo_12_angle_cmd_telemetry,
                &rolling_msg_out_telemetry, &rolling_msg_out_id_telemetry);

    }

#endif // PERIODIC_TELEMETRY

static void data_actuators_t4_out(uint8_t sender_id __attribute__((unused)), struct ActuatorsT4Out * actuators_t4_out_ptr, float * actuators_t4_extra_data_out_ptr){

  /* Copying the struct to be transmitted and the extra data in a local variable: */
    memcpy(&actuators_t4_out,actuators_t4_out_ptr,sizeof(struct ActuatorsT4Out));
    memcpy(&actuators_t4_extra_data_out,actuators_t4_extra_data_out_ptr, sizeof(actuators_t4_extra_data_out) );

    /* Increase the counter to track the sending messages: */
    actuators_t4_out.rolling_msg_out = actuators_t4_extra_data_out[actuators_t4_out_msg_id];
    actuators_t4_out.rolling_msg_out_id = actuators_t4_out_msg_id;
    actuators_t4_out_msg_id++;
    if(actuators_t4_out_msg_id == 255){
      actuators_t4_out_msg_id = 0;
    }

  /* Send the message over UART to the T4 Actuators Board: */
    uint8_t *buf_send = (uint8_t *)&actuators_t4_out;
    //Calculating the checksum
    uint8_t checksum_out_local = 0;
    for(uint16_t i = 0; i < sizeof(struct ActuatorsT4Out) - 1; i++){
        checksum_out_local += buf_send[i];
    }
    actuators_t4_out.checksum_out = checksum_out_local;

#ifdef ACTUATORS_T4_SIM
  /* don't send the data if it is SIM */ 
  //TODO: why not, could be useful for HITL
#else
  /* Do send the bytes */
  uart_put_byte(&(ACTUATORS_T4_PORT), 0, START_BYTE_ACTUATORS_T4);
  for(uint8_t i = 0; i < sizeof(struct ActuatorsT4Out) ; i++){
      uart_put_byte(&(ACTUATORS_T4_PORT), 0, buf_send[i]);
  }
#endif
}

void actuators_t4_uart_init() 
{
  actuators_t4_buf_in_cnt = 0;
  actuators_t4_out_msg_id = 0;

  /* Init ABI bind msg: */
  AbiBindMsgACTUATORS_T4_OUT(ABI_BROADCAST, &ACTUATORS_T4_OUT, data_actuators_t4_out);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATORS_T4_IN, actuators_t4_downlink);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATORS_T4_OUT, actuators_t4_uplink);
#endif
}

/* Send the received message over ABI so then another module can use this actuator state info in e.g. control modules */
void actuators_t4_uart_parse_msg_in(void)
{
  memcpy(&actuators_t4_in, &actuators_t4_msg_buf_in[1], sizeof(struct ActuatorsT4In)); //Starting from 1 to avoid reading the starting byte
  /* Assign the rolling message: */
  actuators_t4_extra_data_in[actuators_t4_in.rolling_msg_in_id] = actuators_t4_in.rolling_msg_in;
  /* Send msg through ABI: */
  AbiSendMsgACTUATORS_T4_IN(ABI_ACTUATORS_T4_IN_ID, &actuators_t4_in, &actuators_t4_extra_data_in[0]);
}

/* Event checking if serial packet are available on the bus */
void actuators_t4_uart_event()
{
  if(fabs(get_sys_time_float() - actuators_t4_last_ts) > 5){ //Reset received packets to zero every 5 second to update the statistics
    actuators_t4_received_packets = 0;
    actuators_t4_last_ts = get_sys_time_float();
  }
#ifdef ACTUATORS_T4_SIM //TODO: use the SIM, NPS and HITL flags ,but if HIL is used it should be able to send the data to the T4 board
    /* Don't do anything if it is SIM */
#else
    while(uart_char_available(&(ACTUATORS_T4_PORT)) > 0) {
        uint8_t actuators_t4_byte_in;
        actuators_t4_byte_in = uart_getch(&(ACTUATORS_T4_PORT));
        if ((actuators_t4_byte_in == START_BYTE_ACTUATORS_T4) || (actuators_t4_buf_in_cnt > 0)) {
            actuators_t4_msg_buf_in[actuators_t4_buf_in_cnt] = actuators_t4_byte_in;
            actuators_t4_buf_in_cnt++;
        }
        if (actuators_t4_buf_in_cnt > sizeof(struct ActuatorsT4In) ) {
            actuators_t4_buf_in_cnt = 0;
            uint8_t checksum_in_local = 0;
            for(uint16_t i = 1; i < sizeof(struct ActuatorsT4In) ; i++){
                checksum_in_local += actuators_t4_msg_buf_in[i];
            }
            if(checksum_in_local == actuators_t4_msg_buf_in[sizeof(struct ActuatorsT4In)]){
                actuators_t4_uart_parse_msg_in();
                actuators_t4_received_packets++;
            }
            else {
                actuators_t4_missed_packets_in++;
            }
        }
    }
#endif
  actuators_t4_message_frequency_in = (uint16_t) actuators_t4_received_packets/(get_sys_time_float() - actuators_t4_last_ts);
}

