/*
 * Copyright (C) 2014 Christophe De Wagter
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
 */

/**
 * @file fbw_datalink.c
 *
 * Datalink through FBW (FlyByWire) process/mcu.
 * So it can decode flight termination modes even when AP is down.
 *
 */

#include "firmwares/fixedwing/fbw_datalink.h"
#include "mcu_periph/uart.h"
#include "led.h"


#define ModemLinkDevice (&(MODEM_LINK).device)
#define AutopilotLinkDevice (&(AUTOPILOT_LINK).device)

static inline void autopilot_parse(char c)
{
  ModemLinkDevice->put_byte(ModemLinkDevice->periph, 0, c);
}

static inline void modem_parse(char c)
{
  AutopilotLinkDevice->put_byte(AutopilotLinkDevice->periph, 0, c);
}

void fbw_datalink_periodic(void)
{
#ifdef MODEM_LINK_LED
  LED_OFF(MODEM_LINK_LED);
#endif
#ifdef AUTOPILOT_LINK_LED
  LED_OFF(AUTOPILOT_LINK_LED);
#endif
}

void fbw_datalink_event(void)
{
#ifdef MODEM_LINK_LED
  if (ModemLinkDevice->char_available(ModemLinkDevice->periph)) {
    LED_ON(MODEM_LINK_LED);
  }
#endif
#ifdef AUTOPILOT_LINK_LED
  if (AutopilotLinkDevice->char_available(AutopilotLinkDevice->periph)) {
    LED_ON(AUTOPILOT_LINK_LED);
  }
#endif

  while (ModemLinkDevice->char_available(ModemLinkDevice->periph))
    modem_parse(ModemLinkDevice->get_byte(ModemLinkDevice->periph));

  while (AutopilotLinkDevice->char_available(AutopilotLinkDevice->periph))
    autopilot_parse(AutopilotLinkDevice->get_byte(AutopilotLinkDevice->periph));
}
