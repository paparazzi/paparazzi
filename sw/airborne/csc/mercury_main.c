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

#include "csc_servos.h"
#include "booz2_imu.h"
#include "booz_ahrs_aligner.h"
#include "booz_ahrs.h"
#include "mercury_xsens.h"
#include "csc_telemetry.h"
#include "csc_adc.h"
#include "mercury_ap.h"
#include "booz2_autopilot.h"
#include "csc_can.h"

#include "booz2_stabilization_attitude.h"


#define CSC_STATUS_TIMEOUT (SYS_TICS_OF_SEC(0.25) / PERIODIC_TASK_PERIOD)

#define PPRZ_MODE_MANUAL 0
#define PPRZ_MODE_AUTO1 1

#define TRESHOLD_MANUAL_PPRZ (MIN_PPRZ / 2)
#define PPRZ_MODE_OF_RC(mode) ((mode) < TRESHOLD_MANUAL_PPRZ ? PPRZ_MODE_MANUAL : PPRZ_MODE_AUTO1)


uint8_t pprz_mode = PPRZ_MODE_AUTO1;
static uint16_t cpu_time = 0;

int main( void ) {
  csc_main_init();
  while(1) {
    if (sys_time_periodic())
      csc_main_periodic();
    csc_main_event();
  }
  return 0;
}

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

  rc_values[RADIO_ROLL] = -RADIO_SCALE * (msg->right_stick_horizontal - ROLL_OFFSET);
  rc_values[RADIO_PITCH] = -RADIO_SCALE * (msg->right_stick_vertical - PITCH_OFFSET);
  rc_values[RADIO_YAW] = RADIO_SCALE * (msg->left_stick_horizontal - YAW_OFFSET);
  rc_values[RADIO_MODE] = RADIO_SCALE * (msg->flap_mix - MODE_OFFSET);
  time_since_last_ppm = 0;
  rc_status = RC_OK;
  pprz_mode = PPRZ_MODE_OF_RC(rc_values[RADIO_MODE]);
  if (pprz_mode == PPRZ_MODE_MANUAL)
    SetCommandsFromRC(commands);
}

STATIC_INLINE void csc_main_init( void ) {

  hw_init();
  sys_time_init();
  led_init();

  Uart0Init();
  Uart1Init();
  
  booz2_imu_init();

  booz_ahrs_aligner_init();
  booz_ahrs_init();
  
  xsens_init();

  booz2_stabilization_attitude_init();


  csc_adc_init();
  ppm_init();

  csc_ap_link_init();
  csc_ap_link_set_servo_cmd_cb(&nop);
  csc_ap_link_set_motor_cmd_cb(&nop);
  csc_ap_link_set_rc_cmd_cb(on_rc_cmd);
  actuators_init();

  csc_ap_init();
  int_enable();

  booz2_stabilization_attitude_enter();
}


STATIC_INLINE void csc_main_periodic( void )
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
  if (pprz_mode == PPRZ_MODE_AUTO1){
    csc_ap_periodic();
  }

}

STATIC_INLINE void csc_main_event( void )
{
  csc_can_event();
  xsens_event_task();
  DatalinkEvent();
}
