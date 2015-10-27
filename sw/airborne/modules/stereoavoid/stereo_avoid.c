/*
 * Copyright (C) 2013
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/stereoavoid/stereoavoid.c
 *
 * Parse avoidance messages from stereocamera to use obstacle results in navigation
 */

// Own header
#include "stereo_avoid.h"


// Navigate Based On Computer Vision Results
#include "avoid_navigation.h"

// Paparazzi State (Attitude)
#include "state.h" // for attitude

// Serial Port
#include "mcu_periph/uart.h"
PRINT_CONFIG_VAR(STEREO_UART)

// define coms link for stereocam
#define STEREO_PORT   (&((STEREO_UART).device))
struct link_device *dev = STEREO_PORT;

#define StereoGetch() STEREO_PORT ->get_byte(STEREO_PORT->periph)
#define StereoSend1(c) STEREO_PORT->put_byte(STEREO_PORT->periph, c)
#define StereoUartSend1(c) StereoSend1(c)
#define StereoSend(_dat,_len) { for (uint8_t i = 0; i< (_len); i++) StereoSend1(_dat[i]); };
#define StereoUartSetBaudrate(_b) uart_periph_set_baudrate(STEREO_PORT, _b);
#define StereoChAvailable()(dev->char_available(dev->periph))



void stereo_avoid_init(void)
{
  // Navigation Code
  init_avoid_navigation();
}

static void stereo_parse(uint8_t c);
static void stereo_parse(uint8_t c)
{
  static int cnt = 0;
  if (c == 0xff) {
    cnt = 1;
  } else if (cnt == 1) {
    avoid_navigation_data.stereo_bin[0] = c;
    cnt = 2;
  } else if (cnt == 2) {
    avoid_navigation_data.stereo_bin[1] = c;
    run_avoid_navigation_onvision();
    cnt = 0;
  } else {
    cnt = 0;
  }
}

void stereo_avoid_run(void)
{
  while (StereoChAvailable()) {
    stereo_parse(StereoGetch());
  }
}
