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

/**
 * @file modules/gps/gps_ubx_ucenter.c
 * @brief Configure Ublox GPS
 *
 */

#include "gps_ubx_ucenter.h"
#include "subsystems/gps/gps_ubx.h"
#include "subsystems/datalink/downlink.h"
#include <stdio.h>

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
#define GPS_UBX_UCENTER_REPLY_CFG_PRT     4

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
#if DEBUG_GPS_UBX_UCENTER
      if (gps_ubx_ucenter.baud_init > 0)
      {
        printf("Initial ublox baudrate found: %u\n", gps_ubx_ucenter.baud_init);
      }
      else
      {
        printf("WARNING: Unable to determine the ublox baudrate. Autoconfiguration is unlikely to work.\n");
      }
#endif
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
  switch (gps_ubx.msg_class)
  {
  case UBX_ACK_ID:
    if (gps_ubx.msg_id == UBX_ACK_ACK_ID) {
      gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_ACK;
#if DEBUG_GPS_UBX_UCENTER
      printf("ACK\n");
#endif
    }
    else {
      gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NACK;
#if DEBUG_GPS_UBX_UCENTER
      printf("NACK\n");
#endif
    }
    break;
  case UBX_MON_ID:
    if (gps_ubx.msg_id == UBX_MON_VER_ID ) {
      gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_VERSION;
      gps_ubx_ucenter.sw_ver_h = UBX_MON_VER_c(gps_ubx.msg_buf,0) - '0';
      gps_ubx_ucenter.sw_ver_l = 10*(UBX_MON_VER_c(gps_ubx.msg_buf,2) - '0');
      gps_ubx_ucenter.sw_ver_l += UBX_MON_VER_c(gps_ubx.msg_buf,3) - '0';
      gps_ubx_ucenter.hw_ver_h = UBX_MON_VER_c(gps_ubx.msg_buf,33) - '0';
      gps_ubx_ucenter.hw_ver_h += 10*(UBX_MON_VER_c(gps_ubx.msg_buf,32) - '0');
      gps_ubx_ucenter.hw_ver_l = UBX_MON_VER_c(gps_ubx.msg_buf,37) - '0';
      gps_ubx_ucenter.hw_ver_l += 10*(UBX_MON_VER_c(gps_ubx.msg_buf,36) - '0');
#if DEBUG_GPS_UBX_UCENTER
      printf("ublox sw_ver: %u.%u\n", gps_ubx_ucenter.sw_ver_h, gps_ubx_ucenter.sw_ver_l);
      printf("ublox hw_ver: %u.%u\n", gps_ubx_ucenter.hw_ver_h, gps_ubx_ucenter.hw_ver_l);
#endif
    }
    break;
  case UBX_CFG_ID:
    if (gps_ubx.msg_id == UBX_CFG_PRT_ID) {
      gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_CFG_PRT;
      gps_ubx_ucenter.port_id = UBX_CFG_PRT_PortId(gps_ubx.msg_buf,0);
      gps_ubx_ucenter.baud_run = UBX_CFG_PRT_Baudrate(gps_ubx.msg_buf,0);
#if DEBUG_GPS_UBX_UCENTER
      printf("gps_ubx_ucenter.baud_run: %u\n", gps_ubx_ucenter.baud_run);
      printf("gps_ubx_ucenter.port_id: %u\n", gps_ubx_ucenter.port_id);
#endif
    }
    break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//
// UCENTER Configuration Functions

/**
 * Polls the u-blox port configuration
 * When the payload is omitted (zero length), the configuration for the incoming
 * (currently used) port is reported.
 *
 */
static inline void gps_ubx_ucenter_config_port_poll(void)
{
  UbxSend_CFG_PRT_POLL();
}

/**
 * Enable u-blox message at desired period.  Will enable the message on the port
 * that this command is received on. For example, sending this configuration message
 * over UART1 will cause the desired message to be published on UART1.
 *
 * For more information on u-blox messages, see the protocol specification.
 * http://www.ublox.com/en/download/documents-a-resources.html
 *
 * @param class u-blox message class
 * @param id u-blox message ID
 * @param rate Desired period to send message. Example: Setting 3 would send the message on every 3rd navigation solution.
 */
static inline void gps_ubx_ucenter_enable_msg(uint8_t class, uint8_t id, uint8_t rate)
{
  UbxSend_CFG_MSG(class, id, rate);
}

/**
 * Automatically determine the baudrate of the u-blox module.
 * Only needed when connecting to a UART port on the u-blox.
 * The discovered baudrate is copied to gps_ubx_ucenter.baud_init.
 *
 * @param nr Autobaud step number to perform
 * @return FALSE when completed
 */
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
    gps_ubx_ucenter_config_port_poll();
    break;
  case 3:
    if (gps_ubx_ucenter.reply == GPS_UBX_UCENTER_REPLY_ACK)
    {
      gps_ubx_ucenter.baud_init = gps_ubx_ucenter.baud_run;
      return FALSE;
    }
    gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
    GpsUartSetBaudrate(B9600); // Maybe the factory default?
    gps_ubx_ucenter_config_port_poll();
    break;
  case 4:
    if (gps_ubx_ucenter.reply == GPS_UBX_UCENTER_REPLY_ACK)
    {
      gps_ubx_ucenter.baud_init = gps_ubx_ucenter.baud_run;
      return FALSE;
    }
    gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
    GpsUartSetBaudrate(B57600); // The high-rate default?
    gps_ubx_ucenter_config_port_poll();
    break;
  case 5:
    if (gps_ubx_ucenter.reply == GPS_UBX_UCENTER_REPLY_ACK)
    {
      gps_ubx_ucenter.baud_init = gps_ubx_ucenter.baud_run;
      return FALSE;
    }
    gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
    GpsUartSetBaudrate(B4800); // Default NMEA baudrate?
    gps_ubx_ucenter_config_port_poll();
    break;
  case 6:
    if (gps_ubx_ucenter.reply == GPS_UBX_UCENTER_REPLY_ACK)
    {
      gps_ubx_ucenter.baud_init = gps_ubx_ucenter.baud_run;
      return FALSE;
    }
    gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
    GpsUartSetBaudrate(B115200); // Last possible option for ublox
    gps_ubx_ucenter_config_port_poll();
    break;
  case 7:
    if (gps_ubx_ucenter.reply == GPS_UBX_UCENTER_REPLY_ACK)
    {
      gps_ubx_ucenter.baud_init = gps_ubx_ucenter.baud_run;
      return FALSE;
    }

    // Autoconfig Failed... let's setup the failsafe baudrate
    // Should we try even a different baudrate?
    gps_ubx_ucenter.baud_init = 0; // Set as zero to indicate that we couldn't verify the baudrate
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
#define NAV_DYN_AIRBORNE_2G 6 // paparazzi default
#define NAV_DYN_AIRBORNE_4G 7

#define NAV5_DYN_PORTABLE    0 // ublox default
#define NAV5_DYN_FIXED       1
#define NAV5_DYN_STATIONARY  2
#define NAV5_DYN_PEDESTRIAN  3
#define NAV5_DYN_AUTOMOTIVE  4
#define NAV5_DYN_SEA         5
#define NAV5_DYN_AIRBORNE_1G 6
#define NAV5_DYN_AIRBORNE_2G 7 // paparazzi default
#define NAV5_DYN_AIRBORNE_4G 8

#ifndef GPS_UBX_NAV5_DYNAMICS
#define GPS_UBX_NAV5_DYNAMICS NAV5_DYN_AIRBORNE_2G
#endif

#define NAV5_MASK 0x05 // Apply dynamic model and position fix mode settings

#define NAV5_2D_ONLY 1
#define NAV5_3D_ONLY 2 // paparazzi default
#define NAV5_AUTO    3 // ublox default

#define NAV5_DEFAULT_MIN_ELEV 5 // deg
#define NAV5_DEFAULT_PDOP_MASK 25 // no units
#define NAV5_DEFAULT_TDOP_MASK 25 // no units
#define NAV5_DEFAULT_P_ACC 100 // m
#define NAV5_DEFAULT_T_ACC 300 // m
#define NAV5_DEFAULT_STATIC_HOLD_THRES 0 // cm/s

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
    UbxSend_CFG_NAV5(NAV5_MASK, GPS_UBX_NAV5_DYNAMICS, NAV5_3D_ONLY, IGNORED, IGNORED, NAV5_DEFAULT_MIN_ELEV, RESERVED,
        NAV5_DEFAULT_PDOP_MASK, NAV5_DEFAULT_TDOP_MASK, NAV5_DEFAULT_P_ACC, NAV5_DEFAULT_T_ACC,
        NAV5_DEFAULT_STATIC_HOLD_THRES, RESERVED, RESERVED, RESERVED, RESERVED);
  }
}



/////////////////////////////////////
// UBlox port and protocol GPS configuration

#ifdef GPS_PORT_ID
#warning "GPS_PORT_ID is no longer needed by the ublox ucenter for automatically configuration. Please remove this defined variable and double check that autoconfig is working as expected."
#endif

// UART mode: 8N1 with reserved1 set for compatability with A4
#define UBX_UART_MODE_MASK 0x000008D0

#define UBX_PROTO_MASK  0x0001
#define NMEA_PROTO_MASK 0x0002
#define RTCM_PROTO_MASK 0x0004

#define GPS_PORT_DDC   		0x00
#define GPS_PORT_UART1 		0x01
#define GPS_PORT_UART2 		0x02
#define GPS_PORT_USB   		0x03
#define GPS_PORT_SPI   		0x04
#define GPS_PORT_RESERVED   0x05

#define __UBX_GPS_BAUD(_u) _u##_BAUD
#define _UBX_GPS_BAUD(_u) __UBX_GPS_BAUD(_u)
#define UBX_GPS_BAUD _UBX_GPS_BAUD(GPS_LINK)

#ifndef GPS_UBX_UCENTER_RATE
#define GPS_UBX_UCENTER_RATE 0x00FA // In milliseconds. 0x00FA = 250ms = 4Hz
#endif

static inline void gps_ubx_ucenter_config_port(void)
{
  switch(gps_ubx_ucenter.port_id)
  {
  // I2C Interface
  case GPS_PORT_DDC:
#ifdef GPS_I2C
    UbxSend_CFG_PRT(gps_ubx_ucenter.port_id, 0x0, 0x0, GPS_I2C_SLAVE_ADDR, 0x0, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
#else
    printf("WARNING: Please include the gps_i2c module.\n");
#endif
    break;
    // UART Interface
  case GPS_PORT_UART1:
  case GPS_PORT_UART2:
    UbxSend_CFG_PRT(gps_ubx_ucenter.port_id, 0x0, 0x0, UBX_UART_MODE_MASK, UART_SPEED(UBX_GPS_BAUD), UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
    break;
    // USB Interface
  case GPS_PORT_USB:
    UbxSend_CFG_PRT(gps_ubx_ucenter.port_id, 0x0, 0x0, 0x0, 0x0, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
    break;
  case GPS_PORT_SPI:
    printf("WARNING: ublox SPI port is currently not supported.\n");
    break;
  default:
    printf("WARNING: Unknown ublox port id: %u\n", gps_ubx_ucenter.port_id);
    break;
  }
}

#define GPS_SBAS_ENABLED       0x01

#define GPS_SBAS_RANGING       0x01
#define GPS_SBAS_CORRECTIONS   0x02
#define GPS_SBAS_INTEGRITY     0x04

#define GPS_SBAS_MAX_SBAS	   1 // Default ublox setting uses 3 SBAS channels(?)

#define GPS_SBAS_AUTOSCAN	   0x00

static inline void gps_ubx_ucenter_config_sbas(void)
{
  // Since March 2nd 2011 EGNOS is released for aviation purposes
  UbxSend_CFG_SBAS(GPS_SBAS_ENABLED, GPS_SBAS_RANGING | GPS_SBAS_CORRECTIONS | GPS_SBAS_INTEGRITY, GPS_SBAS_MAX_SBAS, GPS_SBAS_AUTOSCAN, GPS_SBAS_AUTOSCAN);
  //UbxSend_CFG_SBAS(0x00, 0x00, 0x00, 0x00, 0x00);
}

// Text Telemetry for Debugging
#undef GOT_PAYLOAD

static bool_t gps_ubx_ucenter_configure(uint8_t nr)
{
#if DEBUG_GPS_UBX_UCENTER
  printf("gps_ubx_ucenter_configure nr: %u\n",nr);
#endif

  // Store the reply of the last configuration step and reset
  if (nr < GPS_UBX_UCENTER_CONFIG_STEPS)
    gps_ubx_ucenter.replies[nr] = gps_ubx_ucenter.reply;

  switch (nr) {
  case 0:
    // Use old baudrate to issue a baudrate change command
    gps_ubx_ucenter_config_port();
    break;
  case 1:
#if DEBUG_GPS_UBX_UCENTER
    if (gps_ubx_ucenter.reply != GPS_UBX_UCENTER_REPLY_ACK)
    {
      printf("ublox did not acknowledge port configuration.\n");
    }
    else
    {
      printf("Changed ublox baudrate to: %u\n", UART_SPEED(UBX_GPS_BAUD));
    }
#endif
    // Now the GPS baudrate should have changed
    GpsUartSetBaudrate(UBX_GPS_BAUD);
    gps_ubx_ucenter.baud_run = UART_SPEED(UBX_GPS_BAUD);
    UbxSend_MON_GET_VER();
    break;
  case 2:
  case 3:
  case 4:
  case 5:
    // UBX_G5010 takes 0.7 seconds to answer a firmware request
    // Version info is important for proper configuration as different firmwares have different settings
    break;
  case 6:
    // Send some debugging info: detected baudrate, software version etc...
    gps_ubx_ucenter.replies[0] = (gps_ubx_ucenter.baud_init/1000);
    gps_ubx_ucenter.replies[1] = (gps_ubx_ucenter.baud_init - 1000 * gps_ubx_ucenter.replies[0]) / 100;
    gps_ubx_ucenter.replies[2] = gps_ubx_ucenter.sw_ver_h;
    gps_ubx_ucenter.replies[3] = gps_ubx_ucenter.sw_ver_l;
    gps_ubx_ucenter.replies[4] = gps_ubx_ucenter.hw_ver_h;
    gps_ubx_ucenter.replies[5] = gps_ubx_ucenter.hw_ver_l;
#if DEBUG_GPS_UBX_UCENTER
    DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice,6,gps_ubx_ucenter.replies);
#endif
    // Configure CFG-NAV(5) message
    gps_ubx_ucenter_config_nav();
    break;
  case 7:
    // Geodetic Position Solution
    gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_POSLLH_ID,1);
    break;
  case 8:
    // Velocity Solution in NED
    gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_VELNED_ID, 1);
    break;
  case 9:
    // Receiver Navigation Status
    gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_STATUS_ID, 1);
    break;
  case 10:
    // Space Vehicle Information
    gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_SVINFO_ID, 4);
    break;
  case 11:
    // Navigation Solution Information
#if GPS_UBX_UCENTER_SLOW_NAV_SOL
    gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_SOL_ID, 8);
#else
    gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_SOL_ID, 1);
#endif
    break;
  case 12:
    // Disable UTM on old Lea4P
    gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_POSUTM_ID, 0);
    break;
  case 13:
    // SBAS Configuration
    gps_ubx_ucenter_config_sbas();
    break;
  case 14:
    // Poll Navigation/Measurement Rate Settings
    UbxSend_CFG_RATE(GPS_UBX_UCENTER_RATE, 0x0001, 0x0000);
    break;
  case 15:
    // Raw Measurement Data
#if USE_GPS_UBX_RXM_RAW
    gps_ubx_ucenter_enable_msg(UBX_RXM_ID, UBX_RXM_RAW_ID, 1);
#endif
    break;
  case 16:
    // Subframe Buffer
#if USE_GPS_UBX_RXM_SFRB
    gps_ubx_ucenter_enable_msg(UBX_RXM_ID, UBX_RXM_SFRB_ID, 1);
#endif
    break;
  case 17:
    // Try to save on non-ROM devices...
    UbxSend_CFG_CFG(0x00000000,0xffffffff,0x00000000);
    break;
  case 18:
#if DEBUG_GPS_UBX_UCENTER
    // Debug Downlink the result of all configuration steps: see messages
    // To view, enable DEBUG message in your telemetry configuration .xml
    DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice,GPS_UBX_UCENTER_CONFIG_STEPS,gps_ubx_ucenter.replies);
    for (int i = 0; i < GPS_UBX_UCENTER_CONFIG_STEPS; i++)
    {
      printf("%u\n", gps_ubx_ucenter.replies[i]);
    }
#endif
    return FALSE;
  default:
    break;
  }

  gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
  return TRUE; // Continue, except for the last case
}
