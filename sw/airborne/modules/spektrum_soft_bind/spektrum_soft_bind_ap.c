/*
 * Copyright (C) Kevin van Hecke
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
 * @file "modules/spektrum_soft_bind/spektrum_soft_bind_ap.c"
 * @author Kevin van Hecke
 * Puts Spektrum in binding mode through software
 */

#include "modules/spektrum_soft_bind/spektrum_soft_bind_ap.h"
#include "subsystems/intermcu/intermcu_ap.h"
#include "led.h"

#include "mcu.h"
#include "subsystems/radio_control.h"
#include "mcu_periph/sys_time_arch.h"

#include "mcu_periph/uart.h"
#include "mcu_periph/gpio.h"

void spektrum_soft_bind_init(void)
{

}

#define _UART_INIT(i) i ## _init()
#define UART_INIT(i) _UART_INIT(i)

bool bind_soft_value;
void spektrum_soft_bind_click(bool val)
{
#ifndef INTER_MCU_AP
  send_spektrum_bind();
#else
  intermcu_send_spektrum_bind();
#endif

  bind_soft_value = val;
}

#ifndef INTER_MCU_AP
void send_spektrum_bind(void)
{
  //power cycle the spektrum
#if defined(RADIO_CONTROL_LED)
  LED_OFF(RADIO_CONTROL_LED);
#endif

  RADIO_CONTROL_POWER_OFF(RADIO_CONTROL_POWER_PORT, RADIO_CONTROL_POWER_PIN);
  sys_time_usleep(100000);
  RADIO_CONTROL_POWER_ON(RADIO_CONTROL_POWER_PORT, RADIO_CONTROL_POWER_PIN);
#if defined(RADIO_CONTROL_LED)
  LED_ON(RADIO_CONTROL_LED);
#endif

  //put to bind mode
  RADIO_CONTROL_BIND_IMPL_FUNC();    //basically  = radio_control_spektrum_try_bind()

  // init uart peripherals
  UART_INIT(SPEKTRUM_PRIMARY_UART);
#ifdef SPEKTRUM_SECONDARY_UART
  UART_INIT(SPEKTRUM_SECONDARY_UART);
#endif

}
#endif
