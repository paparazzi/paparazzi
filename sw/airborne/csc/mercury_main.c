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

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#include "generated/periodic.h"
#include "generated/airframe.h"
#include "commands.h"

#include "csc_msg_def.h"
#include ACTUATORS
#include "subsystems/imu.h"
#include "booz/ahrs/ahrs_aligner.h"
#include "booz/ahrs.h"
#include "mercury_xsens.h"
#include "csc_telemetry.h"
#include "csc_adc.h"
#include "mercury_ap.h"
#include "booz/booz2_autopilot.h"
#include "booz/guidance/booz2_guidance_v.h"
#include "csc_can.h"
#include "csc_baro.h"
#include "subsystems/radio_control.h"

#include "booz/stabilization/stabilization_attitude.h"

extern uint8_t vsupply;


#define CSC_STATUS_TIMEOUT (CPU_TICKS_OF_SEC(0.25) / PERIODIC_TASK_PERIOD)

#define PPRZ_MODE_MOTORS_OFF 0
#define PPRZ_MODE_MOTORS_ON  1
#define PPRZ_MODE_IN_FLIGHT  2

uint8_t pprz_mode = PPRZ_MODE_MOTORS_OFF;
static uint16_t cpu_time = 0;

static inline void csc_main_init( void );
static inline void csc_main_periodic( void );
static inline void csc_main_event( void );

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

static void on_rc_cmd(struct CscRCMsg *msg)
{
  uint16_t aux2_flag;

  radio_control.values[RADIO_ROLL]  = CSC_RC_SCALE*(msg->right_stick_horizontal - CSC_RC_OFFSET);
  radio_control.values[RADIO_PITCH] = -CSC_RC_SCALE*(msg->right_stick_vertical - CSC_RC_OFFSET);
  radio_control.values[RADIO_YAW]   =  CSC_RC_SCALE*((msg->left_stick_horizontal_and_aux2 & ~(3 << 13)) - CSC_RC_OFFSET);
  pprz_mode = (msg->left_stick_vertical_and_flap_mix & (3 << 13)) >> 13;
  aux2_flag = (msg->left_stick_horizontal_and_aux2 >> 13) & 0x1;
  radio_control.values[RADIO_MODE2] = (aux2_flag == 0) ? -7000 : ( (aux2_flag == 1) ? 0 : 7000);
  radio_control.values[RADIO_MODE] = (pprz_mode == 0) ? -7000 : ( (pprz_mode == 1) ? 0 : 7000);
  radio_control.values[RADIO_THROTTLE] = -CSC_RC_SCALE*((msg->left_stick_vertical_and_flap_mix & ~(3 << 13)) - CSC_RC_OFFSET);

  radio_control.time_since_last_frame = 0;
  radio_control.status = RC_OK;
}

static inline void csc_main_init( void ) {

  mcu_init();
  sys_time_init();
  led_init();

  Uart0Init();
  Uart1Init();

  imu_init();

  ahrs_aligner_init();
  ahrs_init();

  xsens_init();

  stabilization_attitude_init();
  booz2_guidance_v_init();
  booz_ins_init();

  csc_adc_init();

  baro_scp_init();

  csc_ap_link_init();
  csc_ap_link_set_servo_cmd_cb(&nop);
  csc_ap_link_set_motor_cmd_cb(&nop);
  csc_ap_link_set_rc_cmd_cb(on_rc_cmd);
  actuators_init();

  props_init();

  csc_ap_init();
  mcu_int_enable();

  stabilization_attitude_enter();
}


static inline void csc_main_periodic( void )
{
  static uint32_t csc_loops = 0;

  PeriodicSendAp();
  radio_control_periodic();

  cpu_time++;

  if ((++csc_loops % CSC_STATUS_TIMEOUT) == 0) {
    csc_adc_periodic();
  }
  xsens_periodic_task();

  baro_scp_periodic();

  csc_ap_periodic(pprz_mode == PPRZ_MODE_IN_FLIGHT,
		  !(pprz_mode > PPRZ_MODE_MOTORS_OFF && ahrs_aligner.status == AHRS_ALIGNER_LOCKED));

}

static inline void csc_main_event( void )
{
  csc_can_event();
  xsens_event_task();
  DatalinkEvent();
}
