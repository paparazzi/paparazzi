/*
 * Copyright (C) 2014 Michal Podhradsky
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
 * @file modules/com/usb_serial_stm32_example2.c
 *
 * USB_SERIAL_STM32 example 2 - sends lot of data through serial port. User can control
 * the flow by pressing "S" for stop and "R" for run.
 */

#include "modules/com/usb_serial_stm32.h"

void send_command(void);

uint8_t run;
uint8_t prompt = '$';

/* A lot of text */
uint8_t big_buffer[] = " ASCII stands for American Standard Code for Information Interchange. Computers can only understand numbers, so an ASCII code is the numerical representation of a character such as 'a' or '@' or an action of some sort. ASCII was developed a long time ago and now the non-printing characters are rarely used for their original purpose. Below is the ASCII character table and this includes descriptions of the first 32 non-printing characters. ASCII was actually designed for use with teletypes and so the descriptions are somewhat obscure. If someone says they want your CV however in ASCII format, all this means is they want 'plain' text with no formatting such as tabs, bold or underscoring - the raw format that any computer can understand. This is usually so they can easily import the file into their own applications without issues. Notepad.exe creates ASCII text, or in MS Word you can save a file as 'text only' ";

/**
 * Init module, call VCOM_init() from here
 */
void init_usb_serial(void)
{
  VCOM_init();
  run = FALSE;
}

/**
 * Periodic function in case you needed to send data periodically
 * like telemetry
 * Note that the data are sent once the buffer is full, not immediately
 */
void periodic_usb_serial(void)
{
  if (run) {
    for (uint16_t i = 0; i < sizeof(big_buffer); i++) {
      VCOM_putchar(big_buffer[i]);
    }
  }
}

/**
 * Parse data from buffer
 * Note that the function receives int, not char
 * Because we want to be able to catch -1 in case no
 * more data were available
 */
void usb_serial_parse_packet(int data)
{
  if (data == -1) { return; }
  uint8_t c = (uint8_t)data;
  VCOM_putchar(prompt);
  VCOM_putchar(data);
  VCOM_putchar('\r');
  VCOM_putchar('\n');

  if (c == 'S') {
    run = FALSE;
  }
  if (c == 'R') {
    run = TRUE;
  }
  VCOM_send_message();
}

/**
 * Call VCOM_poll() from module event function
 */
void event_usb_serial(void)
{
  VCOM_event();
  if (UsbSChAvailable()) {
    usb_serial_parse_packet(UsbSGetch());
  }
}
