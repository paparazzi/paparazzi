/*
 * Copyright (C) 2019 Xavier Paris
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/light/light_scheduler.c
 * @brief basic LED scheduler based on WS2812 RGB led driver
 *
 * @author Xavier Paris
 * @maintainer Gautier Hattenberger <gautier.hattenberger@enac.fr>
 */

#include "modules/light/light_scheduler.h"
#include "modules/light/light_ws2812_arch.h"

//#include "state.h"
#include "generated/modules.h"
#include "subsystems/gps.h"

#ifndef WS2812_SEQUENCE
#define WS2812_SEQUENCE 0
#endif

static uint32_t s = 0;
static uint32_t sequence = WS2812_SEQUENCE;

void light_scheduler_init(void)
{
  light_ws2812_arch_init();
}

void light_scheduler_periodic(void)
{
  uint32_t n, s0;

  //uint32_t freq = LIGHT_SCHEDULER_PERIODIC_FREQ;

  if (autopilot_get_mode() == AP_MODE_KILL) sequence=2;
  else if(GpsFixValid()) sequence=1;

  switch (sequence) 
  {
    case 0:
      for (n = 0; n < WS2812_NB_LEDS; n++) {
        s0 = s + 10 * n;
        light_ws2812_arch_set(n, s0 % 255, (s0 + 85) % 255, (s0 + 170) % 255);
      }
      s += 10;
      break;

    case 1: // steady
      for (n = 0; n < WS2812_NB_LEDS; n++) {
        if(n<4)  light_ws2812_arch_set(n, 50, 50, 50); // 4 rear
        if(n>=4 && n<8)  light_ws2812_arch_set(n, 0, 50, 0);   // 4 right
        if(n>=8 && n<12) light_ws2812_arch_set(n, 50, 0, 0);   // 4 left
      }
      break;

    case 2: // rotating
      if(s==0) {
        light_ws2812_arch_set(3, 0, 0, 0);light_ws2812_arch_set(0, 50, 50, 50);
        light_ws2812_arch_set(7, 0, 0, 0);light_ws2812_arch_set(4, 0, 50, 0);
        light_ws2812_arch_set(11, 0, 0, 0);light_ws2812_arch_set(8, 50, 0, 0);
      }
      if(s==1) {
        light_ws2812_arch_set(0, 0, 0, 0);light_ws2812_arch_set(1, 50, 50, 50);
        light_ws2812_arch_set(4, 0, 0, 0);light_ws2812_arch_set(5, 0, 50, 0);
        light_ws2812_arch_set(8, 0, 0, 0);light_ws2812_arch_set(9, 50, 0, 0);
      }
      if(s==2) {
        light_ws2812_arch_set(1, 0, 0, 0);light_ws2812_arch_set(2, 50, 50, 50);
        light_ws2812_arch_set(5, 0, 0, 0);light_ws2812_arch_set(6, 0, 50, 0);
        light_ws2812_arch_set(9, 0, 0, 0);light_ws2812_arch_set(10, 50, 0, 0);
      }
      if(s==3) {
        light_ws2812_arch_set(2, 0, 0, 0);light_ws2812_arch_set(3, 50, 50, 50);
        light_ws2812_arch_set(6, 0, 0, 0);light_ws2812_arch_set(7, 0, 50, 0);
        light_ws2812_arch_set(10, 0, 0, 0);light_ws2812_arch_set(11, 50, 0, 0);
	s=0;
      } else s++;
      break;
  }
}

