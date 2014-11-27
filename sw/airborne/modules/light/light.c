/*
 * Copyright (C) 2009  Gautier Hattenberger
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

#include "modules/light/light.h"
#include "led.h"
#include "generated/airframe.h"

uint8_t strobe_light_mode;
uint8_t nav_light_mode;

#ifndef LIGHT_LED_STROBE
#define LIGHT_LED_STROBE 2
#endif

#ifndef STROBE_LIGHT_MODE_DEFAULT
#define STROBE_LIGHT_MODE_DEFAULT 6
#endif

#ifndef NAV_LIGHT_MODE_DEFAULT
#define NAV_LIGHT_MODE_DEFAULT 4
#endif


void init_light(void) {
  // this part is already done by led_init in fact
  LED_INIT(LIGHT_LED_STROBE);
  LED_OFF(LIGHT_LED_STROBE);
  strobe_light_mode = STROBE_LIGHT_MODE_DEFAULT;
#ifdef LIGHT_LED_NAV
  LED_INIT(LIGHT_LED_NAV);
  LED_OFF(LIGHT_LED_NAV);
  nav_light_mode = NAV_LIGHT_MODE_DEFAULT;
#endif
}

void periodic_light(void)
{
  static uint8_t counter = 0;
#ifdef LIGHT_LED_NAV
  static uint8_t counter_nav = 0;
#endif

  switch (strobe_light_mode)
  {
    default:	// Always off
      LED_OFF(LIGHT_LED_STROBE);
      break;
    case 1:	// Always on
      LED_ON(LIGHT_LED_STROBE);
      break;
    case 2:	// Blink
    case 3:
    case 4:
      if (counter == (strobe_light_mode*5 - 4))
      {
        LED_OFF(LIGHT_LED_STROBE);
      }
      else if (counter >= 20)
      {
        LED_ON(LIGHT_LED_STROBE);
        counter = 0;
      }
      break;
    case 5:	// Complex Blinking
      if (counter == 3)
      {
        LED_OFF(LIGHT_LED_STROBE);
      }
      else if (counter == 4)
      {
        LED_ON(LIGHT_LED_STROBE);
      }
      else if (counter == 6)
      {
        LED_OFF(LIGHT_LED_STROBE);
      }
      else if (counter == 7)
      {
        LED_ON(LIGHT_LED_STROBE);
      }
      else if (counter == 8)
      {
        LED_OFF(LIGHT_LED_STROBE);
      }
      else if (counter >= 25)
      {
        LED_ON(LIGHT_LED_STROBE);
        counter = 0;
      }
      break;
    case 6:
      if (counter <= 18)
      {
        if ((counter % 2) == 0)
        {
          LED_ON(LIGHT_LED_STROBE);
        }
        else
        {
          LED_OFF(LIGHT_LED_STROBE);
        }
      }
      else if (counter == 35)
      {
        counter = 0;
      }
      break;
  }

#ifdef LIGHT_LED_NAV
  switch (nav_light_mode)
  {
    default:	// Always off
      LED_OFF(LIGHT_LED_NAV);
      break;
    case 1:	// Always on
      LED_ON(LIGHT_LED_NAV);
      break;
    case 2:	// Blink
    case 3:
    case 4:
      if (counter_nav == (nav_light_mode*5 - 4))
      {
        LED_OFF(LIGHT_LED_NAV);
      }
      else if (counter_nav >= 20)
      {
        LED_ON(LIGHT_LED_NAV);
        counter_nav = 0;
      }
      break;
  }
  counter_nav++;
#endif

  counter++;
}

