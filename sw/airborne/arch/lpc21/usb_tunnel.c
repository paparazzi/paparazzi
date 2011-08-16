/*
 * $Id$
 *
 * Copyright (C) 2009  Martin Mueller
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

/** \file usb_tunnel.c
 *  \brief USB tunnel application
 *
 *   This creates a USB serial port that connects to UART0 or UART1
 * port of the LPC processor. This enables you to configure the gps
 * receiver or the modem without removing it.
 */

#include "std.h"
#include "mcu.h"
#include "sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/usb_serial.h"

/* minimum LED blink on time 10Hz = 100ms */
#define BLINK_MIN 10

int main( void ) {
  unsigned char inc;
  unsigned int rx_time=0, tx_time=0;

  mcu_init();
  sys_time_init();
  led_init();
  VCOM_allow_linecoding(1);

#ifdef USE_USB_SERIAL
  VCOM_init();
#endif

  mcu_int_enable();

  LED_ON(3);

#ifdef USE_UART0
  while(1) {
    if (T0TC > (rx_time+((PCLK / T0_PCLK_DIV) / BLINK_MIN))) LED_OFF(1);
    if (T0TC > (tx_time+((PCLK / T0_PCLK_DIV) / BLINK_MIN))) LED_OFF(2);
    if (Uart0ChAvailable() && VCOM_check_free_space(1)) {
      LED_ON(1);
      rx_time = T0TC;
      inc = Uart0Getch();
      VCOM_putchar(inc);
    }
    if (VCOM_check_available() && uart0_check_free_space(1)) {
      LED_ON(2);
      tx_time = T0TC;
      inc = VCOM_getchar();
      Uart0Transmit(inc);
    }
  }
#else
  while(1) {
    if (T0TC > (rx_time+((PCLK / T0_PCLK_DIV) / BLINK_MIN))) LED_OFF(1);
    if (T0TC > (tx_time+((PCLK / T0_PCLK_DIV) / BLINK_MIN))) LED_OFF(2);
    if (Uart1ChAvailable() && VCOM_check_free_space(1)) {
      LED_ON(1);
      rx_time = T0TC;
      inc = Uart1Getch();
      VCOM_putchar(inc);
    }
    if (VCOM_check_available() && uart1_check_free_space(1)) {
      LED_ON(2);
      tx_time = T0TC;
      inc = VCOM_getchar();
      Uart1Transmit(inc);
    }
  }
#endif

  return 0;
}

