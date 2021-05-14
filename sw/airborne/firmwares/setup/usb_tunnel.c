/*
 * Copyright (C) 2009  Martin Mueller
 *               2014  Felix Ruess <felix.ruess@gmail.com>
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

/**
 * @file usb_tunnel.c
 *
 * USB tunnel application.
 *
 * This creates a USB serial port that connects to one of the UARTs.
 * This enables you to e.g. configure the gps receiver or the modem without removing it.
 */

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/usb_serial.h"

#ifndef USB_TUNNEL_UART
#error USB_TUNNEL_UART not defined. Add <configure name="TUNNEL_PORT" value="UARTx"/>
#endif

PRINT_CONFIG_VAR(USB_TUNNEL_UART)

PRINT_CONFIG_VAR(TUNNEL_RX_LED)
PRINT_CONFIG_VAR(TUNNEL_TX_LED)

/** minimum LED blink on time in ms */
#define BLINK_MIN 100

static inline void tunnel_event(void)
{
  unsigned char inc;

#if LED_AVAILABLE(TUNNEL_RX_LED)
  static uint32_t rx_time = 0;
  if (get_sys_time_msec() > rx_time + BLINK_MIN) {
    LED_OFF(TUNNEL_RX_LED);
  }
#endif
#if LED_AVAILABLE(TUNNEL_TX_LED)
  static uint32_t tx_time = 0;
  if (get_sys_time_msec() > tx_time + BLINK_MIN) {
    LED_OFF(TUNNEL_TX_LED);
  }
#endif

  if (uart_char_available(&USB_TUNNEL_UART) && VCOM_check_free_space(1)) {
#if LED_AVAILABLE(TUNNEL_RX_LED)
    LED_ON(TUNNEL_RX_LED);
    rx_time = get_sys_time_msec();
#endif
    inc = uart_getch(&USB_TUNNEL_UART);
    VCOM_putchar(inc);
  }
  long fd = 0;
  if (VCOM_check_available() && uart_check_free_space(&USB_TUNNEL_UART, &fd, 1)) {
#if LED_AVAILABLE(TUNNEL_TX_LED)
    LED_ON(TUNNEL_TX_LED);
    tx_time = get_sys_time_msec();
#endif
    inc = VCOM_getchar();
    uart_put_byte(&USB_TUNNEL_UART, fd, inc);
  }
}

int main(void)
{
  mcu_init();
  sys_time_init();
  led_init();

  VCOM_allow_linecoding(1);

  VCOM_init();


  while (1) {
    VCOM_event();
    tunnel_event();
  }

  return 0;
}
