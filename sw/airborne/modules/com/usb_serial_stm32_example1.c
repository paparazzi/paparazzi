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
 * @file modules/com/usb_serial_stm32_example1.c
 *
 * USB_SERIAL_STM32 example 1 -  a template for a console to autopilot
 *
 */

#include "modules/com/usb_serial_stm32.h"
#include <string.h>

void send_command(void);
void cmd_execute(void);

char cmd_buf[64];
uint8_t cmd_idx;
bool_t cmd_avail;
uint8_t prompt = '$';

/**
 * Init module, call VCOM_init() from here
 */
void init_usb_serial(void)
{
  VCOM_init();
  cmd_idx = 0;
  cmd_avail = FALSE;
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
  char c = (char)data;

  if (c == '\r' || c == '\n') {
    // command complete
    cmd_avail = TRUE;
    // add termination characters and the prompt into buffer
    VCOM_putchar('\r');
    VCOM_putchar('\n');
    VCOM_putchar(prompt);
  } else {
    // buffer command
    cmd_buf[cmd_idx++] = c;
    // echo char back and transmit immediately
    VCOM_putchar((uint8_t)c);
    VCOM_send_message();
  }
}

/**
 * Helper function
 */
static inline void ReadUsbBuffer(void)
{
  while (UsbSChAvailable() && !cmd_avail) {
    usb_serial_parse_packet(UsbSGetch());
  }
}

/**
 * Execute command from user
 * use strncmp
 */
void cmd_execute(void)
{
  // copy command into tx buffer for user feedback
  for (int i = 0; i < cmd_idx; i++) {
    VCOM_putchar(cmd_buf[i]);
  }

  if (strncmp("help", cmd_buf, cmd_idx) == 0) {
    uint8_t response[] = " - user asked for help";
    for (uint16_t i = 0; i < sizeof(response); i++) {
      VCOM_putchar(response[i]);
    }
  } else {
    uint8_t response[] = " Command not recognized";
    for (uint16_t i = 0; i < sizeof(response); i++) {
      VCOM_putchar(response[i]);
    }
  }

  // add termination characters and the prompt into buffer
  VCOM_putchar('\r');
  VCOM_putchar('\n');
  VCOM_putchar(prompt);

  // reset counter
  cmd_idx = 0;
  // send complete message
  VCOM_send_message();
}

/**
 * Call VCOM_poll() from module event function
 */
void event_usb_serial(void)
{
  VCOM_event();
  if (UsbSChAvailable()) {
    ReadUsbBuffer();
  }
  if (cmd_avail) {
    cmd_execute();
    cmd_avail = FALSE;
  }
}
