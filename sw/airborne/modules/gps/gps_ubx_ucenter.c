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
 *
 * Initial author: C. De Wagter
 */

#include "gps_ubx_ucenter.h"

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//
// UCENTER: init, periodic and event

static bool_t gps_ubx_ucenter_autobaud(uint8_t nr);
static bool_t gps_ubx_ucenter_configure(uint8_t nr);

#define GPS_UBX_UCENTER_STATUS_STOPPED    0
#define GPS_UBX_UCENTER_STATUS_AUTOBAUD   1
#define GPS_UBX_UCENTER_STATUS_CONFIG     2

#define GPS_UBX_UCENTER_REPLY_NONE        0
#define GPS_UBX_UCENTER_REPLY_ACK         1
#define GPS_UBX_UCENTER_REPLY_NACK        2
#define GPS_UBX_UCENTER_REPLY_VERSION     3

// All U-Center data
struct gps_ubx_ucenter_struct gps_ubx_ucenter;


/////////////////////////////
// Init Function

void gps_ubx_ucenter_init(void)
{
  // Start UCenter
  gps_ubx_ucenter.status = GPS_UBX_UCENTER_STATUS_AUTOBAUD;
  gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
  gps_ubx_ucenter.cnt = 0;

  gps_ubx_ucenter.baud_init = 0;
  gps_ubx_ucenter.baud_run = 0;

  gps_ubx_ucenter.sw_ver_h = 0;
  gps_ubx_ucenter.sw_ver_l = 0;
  gps_ubx_ucenter.hw_ver_h = 0;
  gps_ubx_ucenter.hw_ver_l = 0;

  for (int i=0; i<GPS_UBX_UCENTER_CONFIG_STEPS; i++)
  {
    gps_ubx_ucenter.replies[i] = 0;
  }
}


/////////////////////////////
// Periodic Function
// -time-based configuration

void gps_ubx_ucenter_periodic(void)
{
  switch (gps_ubx_ucenter.status)
  {
    // Save processing time inflight
    case GPS_UBX_UCENTER_STATUS_STOPPED:
      return;
    // Automatically Determine Current Baudrate
    case GPS_UBX_UCENTER_STATUS_AUTOBAUD:
      if (gps_ubx_ucenter_autobaud(gps_ubx_ucenter.cnt) == FALSE)
      {
        gps_ubx_ucenter.status = GPS_UBX_UCENTER_STATUS_CONFIG;
        gps_ubx_ucenter.cnt = 0;
      }
      else
      {
        gps_ubx_ucenter.cnt++;
      }
      break;
    // Send Configuration
    case GPS_UBX_UCENTER_STATUS_CONFIG:
      if (gps_ubx_ucenter_configure(gps_ubx_ucenter.cnt) == FALSE)
      {
        gps_ubx_ucenter.status = GPS_UBX_UCENTER_STATUS_STOPPED;
        gps_ubx_ucenter.cnt = 0;
      }
      else
      {
        gps_ubx_ucenter.cnt++;
      }
      break;
    default:
      // stop this module now...
      // todo
      break;
  }
}

/////////////////////////////
// Event Function
// -fetch replies: ACK and VERSION info

void gps_ubx_ucenter_event(void)
{
  // Save processing time inflight
  if (gps_ubx_ucenter.status == GPS_UBX_UCENTER_STATUS_STOPPED)
    return;

  // Read Configuration Reply's
  if (gps_ubx.msg_class == UBX_ACK_ID) {
    if (gps_ubx.msg_id == UBX_ACK_ACK_ID) {
      gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_ACK;
    }
    else
    {
      gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NACK;
    }
  }
  // Version info
  else if (gps_ubx.msg_class == UBX_MON_ID) {
    if (gps_ubx.msg_id == UBX_MON_VER_ID ) {
      gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_VERSION;
      gps_ubx_ucenter.sw_ver_h = UBX_MON_VER_c(gps_ubx.msg_buf,0) - '0';
      gps_ubx_ucenter.sw_ver_l = 10*(UBX_MON_VER_c(gps_ubx.msg_buf,2) - '0');
      gps_ubx_ucenter.sw_ver_l += UBX_MON_VER_c(gps_ubx.msg_buf,3) - '0';
      gps_ubx_ucenter.hw_ver_h = UBX_MON_VER_c(gps_ubx.msg_buf,33) - '0';
      gps_ubx_ucenter.hw_ver_h += 10*(UBX_MON_VER_c(gps_ubx.msg_buf,32) - '0');
      gps_ubx_ucenter.hw_ver_l = UBX_MON_VER_c(gps_ubx.msg_buf,37) - '0';
      gps_ubx_ucenter.hw_ver_l += 10*(UBX_MON_VER_c(gps_ubx.msg_buf,36) - '0');
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//
// UCENTER Configuration Functions


static bool_t gps_ubx_ucenter_autobaud(uint8_t nr)
{
  switch (nr)
  {
  case 0:
  case 1:
    // Very important for some modules:
    // Give the GPS some time to boot (up to 0.75 second)
    break;
  case 2:
    gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
    GpsUartSetBaudrate(B38400); // Try the most common first?
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_VELNED_ID, 0, 1, 0, 0);
    break;
  case 3:
    if (gps_ubx_ucenter.reply == GPS_UBX_UCENTER_REPLY_ACK)
    {
      gps_ubx_ucenter.baud_init = 38400;
      return FALSE;
    }
    gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
    GpsUartSetBaudrate(B9600); // Maybe the factory default?
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_VELNED_ID, 0, 1, 0, 0);
    break;
  case 4:
    if (gps_ubx_ucenter.reply == GPS_UBX_UCENTER_REPLY_ACK)
    {
      gps_ubx_ucenter.baud_init = 9600;
      return FALSE;
    }
    gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
    GpsUartSetBaudrate(B57600); // The high-rate default?
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_VELNED_ID, 0, 1, 0, 0);
    break;
  case 5:
    if (gps_ubx_ucenter.reply == GPS_UBX_UCENTER_REPLY_ACK)
    {
      gps_ubx_ucenter.baud_init = 57600;
      return FALSE;
     }
    gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
    GpsUartSetBaudrate(B4800); // Default NMEA baudrate finally?
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_VELNED_ID, 0, 1, 0, 0);
    break;
  case 6:
    if (gps_ubx_ucenter.reply == GPS_UBX_UCENTER_REPLY_ACK)
    {
      gps_ubx_ucenter.baud_init = 4800;
      return FALSE;
     }

    // Autoconfig Failed... let's setup the failsafe baudrate
    // Should we try even a different baudrate?
    gps_ubx_ucenter.baud_init = 0;
    GpsUartSetBaudrate(B9600);
    return FALSE;
  default:
    break;
  }
  return TRUE;
}

/////////////////////////////////////
// UBlox internal Navigation Solution

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

#define IGNORED 0
#define RESERVED 0

static inline void gps_ubx_ucenter_config_nav(void)
{
    //New ublox firmware v5 or higher uses CFG_NAV5 message, CFG_NAV is no longer available
    if (gps_ubx_ucenter.sw_ver_h < 5)
    {
      UbxSend_CFG_NAV(NAV_DYN_AIRBORNE_2G, 3, 16, 24, 20, 5, 0, 0x3C, 0x3C, 0x14, 0x03E8 ,0x0000, 0x0, 0x17, 0x00FA, 0x00FA, 0x0064, 0x012C, 0x000F, 0x00, 0x00);
    }
    else
    {
      UbxSend_CFG_NAV5(0x05, NAV5_DYN_AIRBORNE_2G, NAV5_3D_ONLY, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, RESERVED, RESERVED, RESERVED, RESERVED);
    }
}



/////////////////////////////////////
// UBlox port and protocol GPS configuration

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

static inline void gps_ubx_ucenter_config_port(void)
{
 // I2C Interface
  #if GPS_PORT_ID == GPS_PORT_DDC
    UbxSend_CFG_PRT(GPS_PORT_ID, 0x0, 0x0, GPS_I2C_SLAVE_ADDR, 0x0, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
  #endif
  // UART Interface
  #if GPS_PORT_ID == GPS_PORT_UART1 || GPS_PORT_ID == GPS_PORT_UART2
    UbxSend_CFG_PRT(GPS_PORT_ID, 0x0, 0x0, 0x000008D0, 38400, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
  #endif
}

#define GPS_SBAS_ENABLED       0x01

#define GPS_SBAS_RANGING       0x01
#define GPS_SBAS_CORRECTIONS   0x02
#define GPS_SBAS_INTEGRITY     0x04

static inline void gps_ubx_ucenter_config_sbas(void)
{
  // Since March 2nd 2011 EGNOS is released for aviation purposes
  const uint8_t nrofsat = 1;
  UbxSend_CFG_SBAS(GPS_SBAS_ENABLED, GPS_SBAS_RANGING | GPS_SBAS_CORRECTIONS | GPS_SBAS_INTEGRITY, nrofsat, 0x00, 0x00);
  //UbxSend_CFG_SBAS(0x00, 0x00, 0x00, 0x00, 0x00);
}

static inline void gps_ubx_ucenter_enable_msg(uint8_t class, uint8_t id, uint8_t rate)
{
  #if GPS_PORT_ID == GPS_PORT_UART1
    UbxSend_CFG_MSG(class, id, 0, rate, 0, 0);
  #endif
  #if GPS_PORT_ID == GPS_PORT_UART2
    UbxSend_CFG_MSG(class, id, 0, 0, rate, 0);
  #endif
  #if GPS_PORT_ID == GPS_PORT_DDC
    UbxSend_CFG_MSG(class, id, rate, 0, 0, 0);
  #endif
}


// Text Telemetry for Debugging
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#undef GOT_PAYLOAD
#include "downlink.h"

static bool_t gps_ubx_ucenter_configure(uint8_t nr) 
{
  // Store the reply of the last configuration step and reset
  if (nr < GPS_UBX_UCENTER_CONFIG_STEPS)
    gps_ubx_ucenter.replies[nr] = gps_ubx_ucenter.reply;
  
  gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
  
  switch (nr) {
  case 0:
    UbxSend_MON_GET_VER();
    break;
  case 1:
  case 2:
  case 3:
  case 4:
    // UBX_G5010 takes 0.7 seconds to answer a firmware request
    // Version info is important for porper configuration as different firmwares have different settings
    break;
  case 5:
    // Send some debugging info: detected baudrate, software version etc...
    gps_ubx_ucenter.replies[0] = (gps_ubx_ucenter.baud_init/1000);
    gps_ubx_ucenter.replies[1] = (gps_ubx_ucenter.baud_init - 1000 * gps_ubx_ucenter.replies[0]) / 100;
    gps_ubx_ucenter.replies[2] = gps_ubx_ucenter.sw_ver_h;
    gps_ubx_ucenter.replies[3] = gps_ubx_ucenter.sw_ver_l;
    gps_ubx_ucenter.replies[4] = gps_ubx_ucenter.hw_ver_h;
    gps_ubx_ucenter.replies[5] = gps_ubx_ucenter.hw_ver_l;
    DOWNLINK_SEND_DEBUG(DefaultChannel,6,gps_ubx_ucenter.replies);

    //////////////////////////////////
    // Actual configuration start   

    // Use old baudrate to issue a baudrate change command
    gps_ubx_ucenter_config_port();
    break;
  case 6:
    // Now the GPS baudrate should have changed
    GpsUartSetBaudrate(B38400);
    gps_ubx_ucenter.baud_run = 38400;
    gps_ubx_ucenter_config_nav();
    break;
  case 7:
    gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_POSLLH_ID,1);
    break;
  case 8:
    gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_VELNED_ID, 1);
    break;
  case 9:
    gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_STATUS_ID, 1);
    break;
  case 10:
    gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_SVINFO_ID, 4);
    break;
  case 11:
    gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_SOL_ID, 8);
    break;
  case 12:
    // Disable UTM on old Lea4P
    gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_POSUTM_ID, 0);
    break;
  case 13:
    gps_ubx_ucenter_config_sbas();
    break;
  case 14:
    UbxSend_CFG_RATE(0x00FA, 0x0001, 0x0000);
    break;
  case 15:
    // Try to save on non-ROM devices...
    UbxSend_CFG_CFG(0x00000000,0xffffffff,0x00000000);
    break;
  case 16:
    // Debug Downlink the result of all configuration steps: see messages
    DOWNLINK_SEND_DEBUG(DefaultChannel,GPS_UBX_UCENTER_CONFIG_STEPS,gps_ubx_ucenter.replies);
    return FALSE;
  default:
    break;
  }
  return TRUE; // Continue, except for the last case
}




