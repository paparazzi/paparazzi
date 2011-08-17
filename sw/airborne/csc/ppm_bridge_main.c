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
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "mcu_periph/uart.h"
#include "downlink.h"
#include "generated/periodic.h"
#include "generated/airframe.h"
#include "commands.h"
#include "subsystems/radio_control.h"

#include "csc_telemetry.h"
#include "led.h"

#include "pprz_transport.h"

#define RC_PROTOCOL_SYNC 13999

extern uint8_t vsupply;

static uint16_t cpu_time = 0;

static void csc_main_init( void ) {

  mcu_init();
  sys_time_init();
  led_init();

  Uart0Init();
  Uart1Init();

  ppm_init();

  // Configure P0.21 as GPIO, output and then pull high as we use it to drive ppm input transistor
  PINSEL1 = PINSEL1 & ~(0x3 << 10);
  IO0DIR = IO0DIR | (0x1 << 21);
  IO0PIN = IO0DIR | (0x1 << 21);

  mcu_int_enable();

}


static void csc_main_periodic( void )
{
  PeriodicSendAp(DefaultChannel);
  radio_control_periodic_task();

  cpu_time++;
}

static void send_short( int16_t s )
{
  Uart1Transmit(s >> 8);
  Uart1Transmit(s & 0xFF);
}

static void send_channel ( int16_t c )
{
  send_short(c);
  send_short(~c);
}

static void on_rc_event( void )
{

#ifdef SWAP_STICKS
  int temp;
  temp = rc_values[RADIO_YAW];
  rc_values[RADIO_YAW] = rc_values[RADIO_ROLL];
  rc_values[RADIO_ROLL] = temp;
#endif

  // 7 channels
  // integers -9600 to +9600

  // sync bytes
  send_channel(RC_PROTOCOL_SYNC);

  for (int i = 0; i < RADIO_CTL_NB; i++) {
    send_channel(rc_values[i]);
  }

  LED_TOGGLE(3);
}

static void csc_main_event( void )
{
  RadioControlEventCheckAndHandle(on_rc_event);
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
