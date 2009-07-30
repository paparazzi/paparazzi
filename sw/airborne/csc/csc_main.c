/*
 * $Id: booz2_main.c 3049 2009-02-24 16:51:25Z poine $
 *  
 * Copyright (C) 2008  Antoine Drouin
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

#include <inttypes.h>

#include "csc_main.h"

#include "std.h"

#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart.h"
#include "csc_telemetry.h"
#include "periodic.h"
#include "downlink.h"
#include "i2c.h"

#include "csc_servos.h"
#include "csc_throttle.h"
#include "csc_adc.h"
#include "csc_rc_spektrum.h"
#include "csc_msg_def.h"

#include "csc_can.h"
#include "csc_ap_link.h"
static inline void on_servo_cmd(struct CscServoCmd *cmd);
static inline void on_motor_cmd(struct CscMotorMsg *msg);

#define SERVO_TIMEOUT (SYS_TICS_OF_SEC(0.1) / PERIODIC_TASK_PERIOD)
#define CSC_STATUS_TIMEOUT (SYS_TICS_OF_SEC(0.25) / PERIODIC_TASK_PERIOD)

static uint32_t servo_cmd_timeout = 0;
static uint32_t can_msg_count = 0;

// gps stuff stolen from antoine's code
#include "booz/booz2_gps.h"
#include "math/pprz_geodetic_int.h"

struct LtpDef_i  booz_ins_ltp_def;
         bool_t  booz_ins_ltp_initialised;
struct NedCoor_i booz_ins_gps_pos_cm_ned;
struct NedCoor_i booz_ins_gps_speed_cm_s_ned;

static void csc_main_init( void ) {

  hw_init();
  sys_time_init();
  led_init();

#ifdef USE_UART0
  Uart0Init();
#endif
#ifdef USE_UART1
  Uart1Init();
#endif

#ifdef SPEKTRUM_LINK
  spektrum_init();
#endif


#ifdef USE_GPS
  booz2_gps_init();
#endif

  csc_ap_link_init();
  csc_ap_link_set_servo_cmd_cb(on_servo_cmd);
  csc_ap_link_set_motor_cmd_cb(on_motor_cmd);

  csc_adc_init();

  #ifdef USE_I2C0
  i2c_init();
  #endif
  // be sure to call servos_init after uart1 init since they are sharing pins
  csc_servos_init();
#ifdef USE_CSC_THROTTLE
  csc_throttle_init();
#endif
  int_enable();
  

}


static void csc_main_periodic( void ) {
  static uint32_t zeros[4] = {0, 0, 0, 0};
  static uint32_t csc_loops = 0;

  #ifdef DOWNLINK
  PeriodicSendAp();
  #endif

  if (servo_cmd_timeout > SERVO_TIMEOUT) {
    LED_OFF(CAN_LED);
    csc_servos_set(zeros);
  } else {
    servo_cmd_timeout++;
  }
  
  if ((++csc_loops % CSC_STATUS_TIMEOUT) == 0) {
    csc_ap_link_send_status(csc_loops, can_msg_count);
  }
  if ((++csc_loops % CSC_STATUS_TIMEOUT) == 0) {
    csc_adc_periodic();
  }

}

static inline void on_gps_event(void) {
  if (booz_gps_state.fix == BOOZ2_GPS_FIX_3D) {
    if (!booz_ins_ltp_initialised) {
      ltp_def_from_ecef_i(&booz_ins_ltp_def, &booz_gps_state.ecef_pos);
      booz_ins_ltp_initialised = TRUE;
    }
    ned_of_ecef_point_i(&booz_ins_gps_pos_cm_ned, &booz_ins_ltp_def, &booz_gps_state.ecef_pos);
    ned_of_ecef_vect_i(&booz_ins_gps_speed_cm_s_ned, &booz_ins_ltp_def, &booz_gps_state.ecef_vel);
  }

  struct CscGPSFixMsg fix_msg;
  struct CscGPSAccMsg acc_msg;
  struct CscGPSPosMsg pos_msg;

  fix_msg.gps_fix = (booz_gps_state.fix == BOOZ2_GPS_FIX_3D);
  fix_msg.vx = booz_ins_gps_speed_cm_s_ned.x;
  fix_msg.vy = booz_ins_gps_speed_cm_s_ned.y;
  fix_msg.vz = booz_ins_gps_speed_cm_s_ned.z;
  csc_ap_send_msg(CSC_GPS_FIX_ID, (const uint8_t *) &fix_msg, sizeof(fix_msg));

  acc_msg.pacc = booz_gps_state.pacc;
  acc_msg.sacc = booz_gps_state.sacc;
  csc_ap_send_msg(CSC_GPS_ACC_ID, (const uint8_t *) &acc_msg, sizeof(acc_msg));

  pos_msg.val = booz_ins_gps_pos_cm_ned.x;
  pos_msg.axis = CSC_GPS_AXIS_IDX_X;
  csc_ap_send_msg(CSC_GPS_POS_ID, (const uint8_t *) &pos_msg, sizeof(pos_msg));

  pos_msg.val = booz_ins_gps_pos_cm_ned.y;
  pos_msg.axis = CSC_GPS_AXIS_IDX_Y;
  csc_ap_send_msg(CSC_GPS_POS_ID, (const uint8_t *) &pos_msg, sizeof(pos_msg));

  pos_msg.val = booz_ins_gps_pos_cm_ned.z;
  pos_msg.axis = CSC_GPS_AXIS_IDX_Z;
  csc_ap_send_msg(CSC_GPS_POS_ID, (const uint8_t *) &pos_msg, sizeof(pos_msg));

}

static void csc_main_event( void ) {

  csc_can_event();
#ifdef USE_CSC_THROTTLE
  csc_throttle_event_task();
#endif
#ifdef SPEKTRUM_LINK
  spektrum_event_task();
#endif
#ifdef USE_GPS
  Booz2GpsEvent(on_gps_event);
#endif
}


#define MIN_SERVO SYS_TICS_OF_USEC(1000)
#define MAX_SERVO SYS_TICS_OF_USEC(2000)

static inline void on_servo_cmd(struct CscServoCmd *cmd)
{

  uint32_t servos_checked[4];
  uint32_t i;
  for (i=0; i<4; i++)
    servos_checked[i] = cmd->servos[i];
    //    servos_checked[i] = Chop(servos[i],MIN_SERVO, MAX_SERVO);
  csc_servos_set(servos_checked);

  servo_cmd_timeout = 0;
  ++can_msg_count;

  //  DOWNLINK_SEND_CSC_CAN_MSG(&can1_rx_msg.frame, &can1_rx_msg.id,
  //  &can1_rx_msg.dat_a, &can1_rx_msg.dat_b);
  //  DOWNLINK_SEND_ADC_GENERIC(&servos[0], &servos[1]);

}


static inline void on_motor_cmd(struct CscMotorMsg *msg)
{
  // always send to throttle_id zero, only one motorcontrol per csc board
  const static uint8_t throttle_id = 0;

  #ifdef USE_CSC_THROTTLE
  csc_throttle_send_msg(throttle_id, msg->cmd_id, msg->arg1, msg->arg2);
  #endif
}

int main( void ) {
  csc_main_init();
  while(1) {
  if (sys_time_periodic())
    csc_main_periodic();
    csc_main_event();
  }
  return 0;
}

