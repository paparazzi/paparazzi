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


#define __ModemLink(dev, _x) dev##_x
#define _ModemLink(dev, _x)  __ModemLink(dev, _x)
#define ModemLink(_x) _ModemLink(MODEM_LINK, _x)
#define ModemBuffer() ModemLink(ChAvailable())


#define __AutopilotLink(dev, _x) dev##_x
#define _AutopilotLink(dev, _x)  __AutopilotLink(dev, _x)
#define AutopilotLink(_x) _AutopilotLink(AUTOPILOT_LINK, _x)

#define AutopilotBuffer() AutopilotLink(ChAvailable())

static inline void autopilot_parse(char c)
{
  ModemLink(Transmit(c));
}

static inline void modem_parse(char c)
{
  AutopilotLink(Transmit(c));
}

#define ReadAutopilotBuffer() {                 \
    while (AutopilotLink(ChAvailable()))        \
      autopilot_parse(AutopilotLink(Getch()));  \
  }

#define ReadModemBuffer() {                     \
    while (ModemLink(ChAvailable()))            \
      modem_parse(ModemLink(Getch()));          \
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
  if (ModemLink(ChAvailable())) {
    LED_ON(MODEM_LINK_LED);
  }
#endif
#ifdef AUTOPILOT_LINK_LED
  if (AutopilotLink(ChAvailable())) {
    LED_ON(AUTOPILOT_LINK_LED);
  }
#endif

  ReadModemBuffer();
  ReadAutopilotBuffer();
}
