/*
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
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/usb_serial.h"

/* minimum LED blink on time 10Hz = 100ms */
#define BLINK_MIN 10

#ifndef USB_TUNNEL_UART
#if USE_UART0
#define USB_TUNNEL_UART uart0
#else
#define USB_TUNNEL_UART uart1
#endif
#endif

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

#if USE_LED_3
  LED_ON(3);
#endif

  while(1) {

#if USE_LED_1
    if (T0TC > (rx_time+((PCLK / T0_PCLK_DIV) / BLINK_MIN))) LED_OFF(1);
#endif
#if USE_LED_2
    if (T0TC > (tx_time+((PCLK / T0_PCLK_DIV) / BLINK_MIN))) LED_OFF(2);
#endif

    if (uart_char_available(&USB_TUNNEL_UART) && VCOM_check_free_space(1)) {
#if USE_LED_1
      LED_ON(1);
#endif
      rx_time = T0TC;
      inc = uart_getch(&USB_TUNNEL_UART);
      VCOM_putchar(inc);
    }
    if (VCOM_check_available() && uart_check_free_space(&USB_TUNNEL_UART, 1)) {
#if USE_LED_2
      LED_ON(2);
#endif
      tx_time = T0TC;
      inc = VCOM_getchar();
      uart_transmit(&USB_TUNNEL_UART, inc);
    }
  }

  return 0;
}

