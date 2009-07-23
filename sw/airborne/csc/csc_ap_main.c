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
#include "downlink.h"
#include "periodic.h"
#include "airframe.h"
#include "commands.h"
#include "radio_control.h"

#include "csc_servos.h"
#include "csc_telemetry.h"
#include "csc_adc.h"
#include "csc_xsens.h"
#include "csc_autopilot.h"
#include "csc_can.h"
#include "pwm_input.h"

#define CSC_STATUS_TIMEOUT (SYS_TICS_OF_SEC(0.25) / PERIODIC_TASK_PERIOD)

#define PPRZ_MODE_MANUAL 0
#define PPRZ_MODE_AUTO1 1

#define TRESHOLD_MANUAL_PPRZ (MIN_PPRZ / 2)
#define PPRZ_MODE_OF_RC(mode) ((mode) < TRESHOLD_MANUAL_PPRZ ? PPRZ_MODE_MANUAL : PPRZ_MODE_AUTO1)

extern uint8_t vsupply;

uint8_t pprz_mode = PPRZ_MODE_AUTO1;
static uint16_t cpu_time = 0;
uint8_t csc_trims_set = 0;

static void nop(struct CscCanMsg *msg)
{

}

#define RADIO_SCALE 20
#define ROLL_OFFSET 1544
#define PITCH_OFFSET 2551
#define YAW_OFFSET 3585
#define MODE_OFFSET 5632

static void on_rc_cmd(struct CscRCMsg *msg)
{
  int aux2_flag;
  static int last_aux2_flag = -1;

  rc_values[RADIO_ROLL]  = CSC_RC_SCALE*(msg->right_stick_horizontal - CSC_RC_OFFSET);
  rc_values[RADIO_PITCH] = -CSC_RC_SCALE*(msg->right_stick_vertical - CSC_RC_OFFSET);
  rc_values[RADIO_YAW]   =  CSC_RC_SCALE*((msg->left_stick_horizontal_and_aux2 & ~(3 << 13)) - CSC_RC_OFFSET);
  uint8_t mode = (msg->left_stick_vertical_and_flap_mix & (3 << 13)) >> 13;
  rc_values[RADIO_MODE]  =  mode ? -7000 : ( (mode == 1) ? 0 : 7000); 
  aux2_flag = (msg->left_stick_horizontal_and_aux2 >> 13) & 0x1;
  rc_values[RADIO_MODE2] = (aux2_flag == 0) ? -7000 : ( (aux2_flag == 1) ? 0 : 7000); 
  rc_values[RADIO_THROTTLE] = -CSC_RC_SCALE*((msg->left_stick_vertical_and_flap_mix & ~(3 << 13)) - CSC_RC_OFFSET);

  time_since_last_ppm = 0;
  rc_status = RC_OK;
  pprz_mode = PPRZ_MODE_OF_RC(rc_values[RADIO_MODE]);
  if (pprz_mode == PPRZ_MODE_MANUAL) {
    csc_ap_clear_ierrors();
    SetCommandsFromRC(commands);
  }
}

static void csc_main_init( void ) {

  hw_init();
  sys_time_init();
  led_init();

  Uart0Init();
  Uart1Init();

  xsens_init();

  csc_adc_init();
  ppm_init();

#ifdef USE_PWM_INPUT
  pwm_input_init();
#endif

  csc_ap_link_init();
  csc_ap_link_set_servo_cmd_cb(&nop);
  csc_ap_link_set_motor_cmd_cb(&nop);
  csc_ap_link_set_rc_cmd_cb(on_rc_cmd);
  actuators_init();

  csc_ap_init();
  int_enable();

}


static void csc_main_periodic( void )
{
  static uint32_t csc_loops = 0;
  
  PeriodicSendAp();
  radio_control_periodic_task();

  if (rc_status == RC_REALLY_LOST) {
      pprz_mode = PPRZ_MODE_AUTO1;
  }
  cpu_time++;

  if ((++csc_loops % CSC_STATUS_TIMEOUT) == 0) {
    csc_adc_periodic();
  }
  xsens_periodic_task();
  if (pprz_mode == PPRZ_MODE_AUTO1)
    csc_ap_periodic(cpu_time);

  if (csc_trims_set) {
    csc_trims_set = 0;
    csc_ap_set_trims();
  }

#ifdef ACTUATORS
  SetActuatorsFromCommands(commands);
#endif

}

static void csc_main_event( void )
{
  csc_can_event();
  xsens_event_task();
  DatalinkEvent();
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
