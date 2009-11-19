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
#include "csc_vane.h"

#ifdef USE_BUSS_TWI_BLMC_MOTOR
#include "buss_twi_blmc_hw.h"
#endif
#include "i2c.h"

#include "csc_servos.h"

#include "airspeed.h"
#include "baro_ets.h"
#include "airspeed_ets.h"

#include "interrupt_hw.h"
#include "uart.h"
#include "csc_telemetry.h"
#include "periodic.h"
#include "downlink.h"
#include "pwm_input.h"

#include "csc_adc.h"
#include "csc_rc_spektrum.h"

#include "csc_can.h"
#include "csc_ap_link.h"
static inline void on_servo_cmd(struct CscServoCmd *cmd);
static inline void on_motor_cmd(struct CscMotorMsg *msg);
static inline void on_prop_cmd(struct CscPropCmd *msg, int idx);

#define SERVO_TIMEOUT (SYS_TICS_OF_SEC(0.1) / PERIODIC_TASK_PERIOD)
#define CSC_STATUS_TIMEOUT (SYS_TICS_OF_SEC(0.25) / PERIODIC_TASK_PERIOD)
#define AIRSPEED_TIMEOUT (SYS_TICS_OF_SEC(0.05) / PERIODIC_TASK_PERIOD)

static uint32_t servo_cmd_timeout = 0;
static uint32_t can_msg_count = 0;


static void csc_main_init( void ) {

  hw_init();
  sys_time_init();
  led_init();

  
  actuators_init();
  csc_servos_init();


#ifdef USE_UART0
  Uart0Init();
#endif

#ifdef USE_UART1
  Uart1Init();
#endif

#ifdef SPEKTRUM_LINK
  spektrum_init();
#endif

#ifdef USE_PWM_INPUT
  pwm_input_init();
#endif

  csc_ap_link_init();
  csc_ap_link_set_servo_cmd_cb(on_servo_cmd);
  csc_ap_link_set_motor_cmd_cb(on_motor_cmd);
  csc_ap_link_set_prop_cmd_cb(on_prop_cmd);

#ifdef ADC
  csc_adc_init();
#endif

  // be sure to call servos_init after uart1 init since they are sharing pins
  #ifdef USE_I2C0
  i2c0_init();
  #endif

  #ifdef USE_BUSS_TWI_BLMC_MOTOR
  motors_init();
  #endif

#ifdef USE_AIRSPEED
  airspeed_init();
#endif

#ifdef USE_AIRSPEED_ETS
  airspeed_ets_init();
#endif
#ifdef USE_BARO_ETS
  baro_ets_init();
#endif

  int_enable();
}


static void csc_main_periodic( void ) {
  static uint32_t zeros[4] = {0, 0, 0, 0};
  static uint32_t csc_loops = 0;

  #ifdef DOWNLINK
  PeriodicSendAp_DefaultChannel();
  #endif

  #ifdef USE_VANE_SENSOR
  csc_vane_periodic();
  #endif

  if (servo_cmd_timeout > SERVO_TIMEOUT) {
    csc_servos_set(zeros);
  } else {
    servo_cmd_timeout++;
  }
  
  if ((++csc_loops % CSC_STATUS_TIMEOUT) == 0) {
    csc_ap_link_send_status(csc_loops, can_msg_count);
  }
#ifdef ADC
  if ((csc_loops % CSC_STATUS_TIMEOUT) == 0) {
    csc_adc_periodic();
  }
#endif

  if ((csc_loops % AIRSPEED_TIMEOUT) == 0) {
#ifdef USE_AIRSPEED_ETS
    airspeed_ets_periodic();
#endif
#ifdef USE_BARO_ETS
    baro_ets_read();
#endif
  } else if ((csc_loops % AIRSPEED_TIMEOUT) == 1) {
#ifdef USE_BARO_ETS
    baro_ets_periodic();
#endif
#ifdef USE_AIRSPEED_ETS
    airspeed_ets_read();
#endif
  }

#ifdef USE_AIRSPEED
  airspeed_update();
#endif
}

static void csc_main_event( void ) {

  csc_can_event();
#ifdef USE_BUSS_TWI_BLMC_MOTOR
  motors_event();
#endif
#ifdef SPEKTRUM_LINK
  spektrum_event_task();
#endif
}


#define MIN_SERVO SYS_TICS_OF_USEC(1000)
#define MAX_SERVO SYS_TICS_OF_USEC(2000)

#ifdef USE_BUSS_TWI_BLMC_MOTOR
static void on_prop_cmd(struct CscPropCmd *cmd, int idx)
{
  for(uint8_t i = 0; i < 4; i++)
    motors_set_motor(i + idx * 4, cmd->speeds[i]);
  
  motors_commit(idx == 1);

  ++can_msg_count;
}
#else
static void on_prop_cmd(struct CscPropCmd *cmd, int idx) {}
#endif


static void on_servo_cmd(struct CscServoCmd *cmd)
{

  uint16_t* servos = (uint16_t*)(cmd);
  uint8_t i;
  
  //  uint32_t servos_checked[4];
  for(i = 0; i < 4; i++)
    csc_servo_normalized_set(i,servos[i]);
  csc_servos_commit();
  
  servo_cmd_timeout = 0;
  
  ++can_msg_count;
}


static void on_motor_cmd(struct CscMotorMsg *msg)
{

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

