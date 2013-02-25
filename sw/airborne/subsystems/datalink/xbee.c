/*
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
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

#include "mcu_periph/sys_time.h"
#include "print.h"
#include "subsystems/datalink/xbee.h"

#ifdef SIM_UART
#include "sim_uart.h"
#endif

uint8_t xbee_cs;
uint8_t xbee_rssi;

struct xbee_transport xbee_tp;

#define AT_COMMAND_SEQUENCE "+++"
#define AT_SET_MY "ATMY"
#define AT_AP_MODE "ATAP1\r"
#define AT_EXIT "ATCN\r"

static uint8_t xbee_text_reply_is_ok(void)
{
  char c[2];
  int count = 0;

  while (TransportLink(XBEE_UART,ChAvailable()))
  {
    char cc = TransportLink(XBEE_UART,Getch());
    if (count < 2)
      c[count] = cc;
    count++;
  }

  if ((count > 2) && (c[0] == 'O') && (c[1] == 'K'))
    return TRUE;

  return FALSE;
}

static uint8_t xbee_try_to_enter_api(void) {

  /** Switching to AT mode (FIXME: busy waiting) */
  XBeePrintString(XBEE_UART,AT_COMMAND_SEQUENCE);

  /** - busy wait 1.25s */
  sys_time_usleep(1250000);

  return xbee_text_reply_is_ok();
}

#define XBeeUartSetBaudrate(_a) TransportLink(XBEE_UART,SetBaudrate(_a))


#if XBEE_BAUD == B9600
    #define XBEE_BAUD_ALTERNATE B57600
    #define XBEE_ATBD_CODE "ATBD3\rATWR\r"
    #pragma message "Experimental: XBEE-API@9k6 auto-baudrate 57k6 -> 9k6 (stop ground link for correct operation)"
#elif XBEE_BAUD == B57600
    #define XBEE_BAUD_ALTERNATE B9600
    #define XBEE_ATBD_CODE "ATBD6\rATWR\r"
    #pragma message "Experimental: XBEE-API@57k6 auto-baudrate 9k6 -> 57k6 (stop ground link for correct operation)"
#else
    #warning XBEE-API Non default baudrate: auto-baud disabled
#endif


void xbee_init( void ) {
  xbee_tp.status = XBEE_UNINIT;
  xbee_tp.trans.msg_received = FALSE;

  // Empty buffer before init process
  while (TransportLink(XBEE_UART,ChAvailable()))
    TransportLink(XBEE_UART,Getch());

#ifndef NO_XBEE_API_INIT
  /** - busy wait 1.25s */
  sys_time_usleep(1250000);

  if (! xbee_try_to_enter_api() )
  {
    #ifdef XBEE_BAUD_ALTERNATE

      // Badly configured... try the alternate baudrate:
      XBeeUartSetBaudrate(XBEE_BAUD_ALTERNATE);
      if ( xbee_try_to_enter_api() )
      {
        // The alternate baudrate worked,
        XBeePrintString(XBEE_UART,XBEE_ATBD_CODE);
      }
      else
      {
        // Complete failure, none of the 2 baudrates result in any reply
        // TODO: set LED?

        // Set the default baudrate, just in case everything is right
        XBeeUartSetBaudrate(XBEE_BAUD);
        XBeePrintString(XBEE_UART,"\r");
      }

    #endif
    // Continue changing settings until the EXIT is issued.
  }

  /** Setting my address */
  XBeePrintString(XBEE_UART,AT_SET_MY);
  uint16_t addr = XBEE_MY_ADDR;
  XBeePrintHex16(XBEE_UART,addr);
  XBeePrintString(XBEE_UART,"\r");

  XBeePrintString(XBEE_UART,AT_AP_MODE);

#ifdef XBEE_INIT
  XBeePrintString(XBEE_UART,XBEE_INIT);
#endif

  /** Switching back to normal mode */
  XBeePrintString(XBEE_UART,AT_EXIT);

  XBeeUartSetBaudrate(XBEE_BAUD);

#endif
}
