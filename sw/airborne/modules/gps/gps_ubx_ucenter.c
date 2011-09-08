/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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
 */

#include "gps_ubx_ucenter.h"

#define GPS_UBX_UCENTER_STATUS_RUNNING    1
#define GPS_UBX_UCENTER_STATUS_STOPPED    0

#define GPS_UBX_UCENTER_REPLY_NONE        0
#define GPS_UBX_UCENTER_REPLY_ACK         1
#define GPS_UBX_UCENTER_REPLY_NACK        2

/** Space Vehicle Information */
struct gps_ubx_ucenter_t {
  uint8_t status;
  uint8_t reply;
} gps_ubx_ucenter;


/////////////////////////////
// Periodic Function

void gps_ubx_ucenter_init(void)
{
  // Start UCenter
  gps_ubx_ucenter.status = GPS_UBX_UCENTER_STATUS_RUNNING;
  gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
}



/////////////////////////////
// Periodic Function

void gps_ubx_ucenter_periodic(void)
{
  // Save processing time inflight
  if (gps_ubx_ucenter.status == GPS_UBX_UCENTER_STATUS_STOPPED)
    return;

  // Autobaud
  
  //   
}

/////////////////////////////
// Event Function

void gps_ubx_ucenter_event(void)
{
  // Save processing time inflight
  if (gps_ubx_ucenter_status == GPS_UBX_UCENTER_STATUS_STOPPED)
    return;

  // Read Configuration Reply's
  if (gps_ubx.msg_class == UBX_ACK_ID) {
    if (gps_ubx.msg_id == UBX_ACK_ACK_ID) {
      gps_ubx_ucenter.reply = GPX_UBX_UCENTER_REPLY_ACK;
    }
    else
    {
      gps_ubx_ucenter.reply = GPX_UBX_UCENTER_REPLY_NACK;
    }
  }
  // Version info
  else if (gps_ubx.msg_class == UBX_ACK_ID) {
  }
}



/*
 * dynamic GPS configuration
#define NAV_DYN_STATIONARY  1
#define NAV_DYN_PEDESTRIAN  2
#define NAV_DYN_AUTOMOTIVE  3
#define NAV_DYN_SEA         4
#define NAV_DYN_AIRBORNE_1G 5
#define NAV_DYN_AIRBORNE_2G 6
#define NAV_DYN_AIRBORNE_4G 7

#define NAV5_DYN_PORTABLE    0
#define NAV5_DYN_FIXED       1
#define NAV5_DYN_STATIONARY  2
#define NAV5_DYN_PEDESTRIAN  3
#define NAV5_DYN_AUTOMOTIVE  4
#define NAV5_DYN_SEA         5
#define NAV5_DYN_AIRBORNE_1G 6
#define NAV5_DYN_AIRBORNE_2G 7
#define NAV5_DYN_AIRBORNE_4G 8

#define NAV5_2D_ONLY 1
#define NAV5_3D_ONLY 2
#define NAV5_AUTO    3


bool_t gps_configuring;
static uint8_t gps_status_config;

gps_status_config = 0;
gps_configuring = TRUE;

 */

/*
 *
 *
 * GPS dynamic configuration
 *
 *
#ifdef GPS_CONFIGURE

#define UBX_PROTO_MASK  0x0001
#define NMEA_PROTO_MASK 0x0002
#define RTCM_PROTO_MASK 0x0004

#define GPS_PORT_DDC   0x00
#define GPS_PORT_UART1 0x01
#define GPS_PORT_UART2 0x02
#define GPS_PORT_USB   0x03
#define GPS_PORT_SPI   0x04

#ifndef GPS_PORT_ID
#define GPS_PORT_ID GPS_PORT_UART1
#endif

#define __UBX_GPS_BAUD(_u) _u##_BAUD
#define _UBX_GPS_BAUD(_u) __UBX_GPS_BAUD(_u)
#define UBX_GPS_BAUD _UBX_GPS_BAUD(GPS_LINK)

// Configure the GPS baud rate using the current uart baud rate. Busy
// wait for the end of the transmit. Then, BEFORE waiting for the ACK,
// change the uart rate. 
#if GPS_PORT_ID == GPS_PORT_UART1 || GPS_PORT_ID == GPS_PORT_UART2
void gps_configure_uart(void) {
#ifdef FMS_PERIODIC_FREQ
  UbxSend_CFG_PRT(GPS_PORT_ID, 0x0, 0x0, 0x000008D0, 38400, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
  uint8_t loop=0;
  while (GpsUartRunning) {
    //doesn't work unless some printfs are used, so :
    if (loop<9) {
      printf("."); loop++;
    } else {
      printf("\b"); loop--;
    }
  }
#else
  UbxSend_CFG_PRT(GPS_PORT_ID, 0x0, 0x0, 0x000008D0, UBX_GPS_BAUD, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
  while (GpsUartRunning); //
#endif

  GpsUartInitParam(UBX_GPS_BAUD,  UART_8N1, UART_FIFO_8);
}
#endif

#if GPS_PORT_ID == GPS_PORT_DDC
void gps_configure_uart(void) {
  UbxSend_CFG_PRT(GPS_PORT_ID, 0x0, 0x0, GPS_I2C_SLAVE_ADDR, 0x0, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
}
#endif

#define IGNORED 0
#define RESERVED 0

#ifdef USER_GPS_CONFIGURE
#include USER_GPS_CONFIGURE
#else
static bool_t user_gps_configure(bool_t cpt) {
  switch (cpt) {
  case 0:
    //New ublox firmware v5 or higher uses CFG_NAV5 message, CFG_NAV is no longer available
    //UbxSend_CFG_NAV(NAV_DYN_AIRBORNE_2G, 3, 16, 24, 20, 5, 0, 0x3C, 0x3C, 0x14, 0x03E8 ,0x0000, 0x0, 0x17, 0x00FA, 0x00FA, 0x0064, 0x012C, 0x000F, 0x00, 0x00);
    UbxSend_CFG_NAV5(0x05, NAV5_DYN_AIRBORNE_2G, NAV5_3D_ONLY, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, RESERVED, RESERVED, RESERVED, RESERVED);
    break;
  case 1:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_POSUTM_ID, 0, 1, 0, 0);
    break;
  case 2:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_VELNED_ID, 0, 1, 0, 0);
    break;
  case 3:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_STATUS_ID, 0, 1, 0, 0);
    break;
  case 4:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_SVINFO_ID, 0, 4, 0, 0);
    break;
  case 5:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_SOL_ID, 0, 8, 0, 0);
    break;
  case 6:
    UbxSend_CFG_SBAS(0x00, 0x00, 0x00, 0x00, 0x00);
    break;
  case 7:
    UbxSend_CFG_RATE(0x00FA, 0x0001, 0x0000);
    return FALSE;
  }
  return TRUE; // Continue, except for the last case
}
#endif // ! USER_GPS_CONFIGURE

void gps_configure( void ) {
  if (gps_ubx.msg_class == UBX_ACK_ID) {
    if (gps_ubx.msg_id == UBX_ACK_ACK_ID) {
      gps_status_config++;
    }
  }
  gps_configuring = user_gps_configure(gps_status_config);
}
#endif // GPS_CONFIGURE

 */



