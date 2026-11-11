/*
 * Copyright (C) 2008-2011 The Paparazzi Team
 *               2026 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * Initial author: C. De Wagter
 */

/**
 * @file modules/gps/gps_ubx_ucenter.c
 * @brief Configure Ublox GPS
 *
 * After finding the baudrate, the receiver is configured with the legacy UBX-CFG messages, or with
 * the configuration interface (UBX-CFG-VALSET) when it doesn't have them anymore (e.g. X20).
 */

#include "modules/gps/gps_ubx_ucenter.h"
#include "modules/gps/gps_ubx.h"
#include "ubx_protocol.h"
#include "modules/datalink/downlink.h"
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

#if GPS_UBX_NB > 1
#warning "Only one GPS_UBX is supported for ucenter configuration."
#endif

#ifdef GPS_PORT_ID
#warning "GPS_PORT_ID is no longer needed by the ublox ucenter for automatically configuration. Please remove this defined variable and double check that autoconfig is working as expected."
#endif

//////////////////////////////////////////////////////////////////////////////////////
// Configuration, the same for both interfaces where possible

/** Measurement period [ms]: 5 Hz by default, 10 Hz with the configuration interface (X20). When the
 *  receiver refuses it, the fallback (4 Hz legacy, e.g. LEA-4, and 5 Hz with the configuration interface) */
#ifdef GPS_UBX_UCENTER_RATE
#define GPS_UBX_UCENTER_RATE_LEGACY GPS_UBX_UCENTER_RATE
#define GPS_UBX_UCENTER_RATE_VALSET GPS_UBX_UCENTER_RATE
#else
#define GPS_UBX_UCENTER_RATE_LEGACY 200
#define GPS_UBX_UCENTER_RATE_VALSET 100
#endif
#define GPS_UBX_UCENTER_RATE_LEGACY_FALLBACK 250
#define GPS_UBX_UCENTER_RATE_VALSET_FALLBACK 200

/** Dynamic platform model (same values for CFG-NAV5 and CFG-NAVSPG-DYNMODEL) */
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

/** Position fix mode (CFG-NAV5 and CFG-NAVSPG-FIXMODE) */
#define NAV5_2D_ONLY 1
#define NAV5_3D_ONLY 2 // paparazzi default
#define NAV5_AUTO    3 // ublox default

/** NMEA output next to UBX, disabled by default */
#if GPS_UBX_UCENTER_ENABLE_NMEA
#define GPS_UBX_ENABLE_NMEA_DATA_MASK 0xff
#else
#define GPS_UBX_ENABLE_NMEA_DATA_MASK 0x00
#endif

/** Save the configuration to battery backed RAM and flash (UBX-CFG-CFG) */
#ifndef GPS_UBX_UCENTER_SAVE
#define GPS_UBX_UCENTER_SAVE TRUE
#endif

/** Receiver port numbers (UBX-CFG-PRT), also the offset of the CFG-MSGOUT keys */
#define GPS_PORT_DDC      0x00
#define GPS_PORT_UART1    0x01
#define GPS_PORT_UART2    0x02
#define GPS_PORT_USB      0x03
#define GPS_PORT_SPI      0x04

//////////////////////////////////////////////////////////////////////////////////////
// UCENTER: init, periodic and event

#ifndef GPS_I2C
static bool gps_ubx_ucenter_autobaud(uint8_t nr);
#endif
static bool gps_ubx_ucenter_configure(uint8_t nr);
static bool gps_ubx_ucenter_configure_valset(uint8_t nr);

#define GPS_UBX_UCENTER_STATUS_STOPPED    0
#define GPS_UBX_UCENTER_STATUS_AUTOBAUD   1
#define GPS_UBX_UCENTER_STATUS_CONFIG     2
#define GPS_UBX_UCENTER_STATUS_WAITING    3

#define GPS_UBX_UCENTER_REPLY_NONE        0
#define GPS_UBX_UCENTER_REPLY_ACK         1
#define GPS_UBX_UCENTER_REPLY_NACK        2
#define GPS_UBX_UCENTER_REPLY_VERSION     3
#define GPS_UBX_UCENTER_REPLY_CFG_PRT     4

// Target baudrate for the module
#define UBX_GPS_BAUD (UBX_GPS_PORT).baudrate

// All U-Center data
struct gps_ubx_ucenter_struct gps_ubx_ucenter;


/**
 * Satellite systems in use: from the satellites in the solution (UBX-NAV-SAT), else as reported by UBX-MON-GNSS.
 * The X20 reports only GPS in UBX-MON-GNSS while it uses more systems.
 */
static inline uint8_t gps_ubx_ucenter_gnss(void)
{
  return (gps_ubx[0].gnss_in_use != 0) ? gps_ubx[0].gnss_in_use : gps_ubx_ucenter.gnss_in_use;
}

#if PERIODIC_TELEMETRY
/** Receiver information, slowly for the GCS and the logs (the configuration itself is only done at start) */
static void send_ublox_info(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t gnss = gps_ubx_ucenter_gnss();
  pprz_msg_send_UBLOX_INFO(trans, dev, AC_ID, &gps_ubx_ucenter.baud_run, &gps_ubx_ucenter.sw_ver_h,
                           &gps_ubx_ucenter.sw_ver_l, &gps_ubx_ucenter.hw_ver_h, &gps_ubx_ucenter.hw_ver_l,
                           &gps_ubx_ucenter.sbas_in_use, &gnss);
}
#endif

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
  gps_ubx_ucenter.baud_target = UBX_GPS_BAUD;

  gps_ubx_ucenter.sw_ver_h = 0;
  gps_ubx_ucenter.sw_ver_l = 0;
  gps_ubx_ucenter.hw_ver_h = 0;
  gps_ubx_ucenter.hw_ver_l = 0;
  gps_ubx_ucenter.sbas_in_use = 0;
  gps_ubx_ucenter.gnss_in_use = 0;

  gps_ubx_ucenter.cfg_prt_reply = false;
  gps_ubx_ucenter.version_reply = false;
  gps_ubx_ucenter.protver = 0;
  gps_ubx_ucenter.valset = false;

  for (int i = 0; i < GPS_UBX_UCENTER_CONFIG_STEPS; i++) {
    gps_ubx_ucenter.replies[i] = 0;
  }

  gps_ubx_ucenter.dev = &(UBX_GPS_PORT).device;

#if PERIODIC_TELEMETRY
  // The init also runs again when the configuration is restarted from the settings
  static bool telemetry_registered = false;
  if (!telemetry_registered) {
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_UBLOX_INFO, send_ublox_info);
    telemetry_registered = true;
  }
#endif
}


/////////////////////////////
// Periodic Function
// -time-based configuration

/** One configuration step with the interface the receiver supports, false when done */
static bool gps_ubx_ucenter_configure_step(uint8_t nr)
{
  if (gps_ubx_ucenter.valset) {
    return gps_ubx_ucenter_configure_valset(nr);
  }
  return gps_ubx_ucenter_configure(nr);
}

void gps_ubx_ucenter_periodic(void)
{
  switch (gps_ubx_ucenter.status) {
    // Save processing time inflight
    case GPS_UBX_UCENTER_STATUS_STOPPED:
      return;
    // Automatically Determine Current Baudrate
    case GPS_UBX_UCENTER_STATUS_AUTOBAUD:
#ifdef GPS_I2C
      gps_ubx_ucenter.cnt = 0;
      gps_ubx_ucenter.status = GPS_UBX_UCENTER_STATUS_CONFIG;
#else
      if (!gps_ubx_ucenter_autobaud(gps_ubx_ucenter.cnt)) {
        gps_ubx_ucenter.status = GPS_UBX_UCENTER_STATUS_CONFIG;
        gps_ubx_ucenter.cnt = 0;
      } else {
        gps_ubx_ucenter.cnt++;
      }
#endif /* GPS_I2C */
      break;
    // Send Configuration
    case GPS_UBX_UCENTER_STATUS_CONFIG:
      if (!gps_ubx_ucenter_configure_step(gps_ubx_ucenter.cnt)) {
        gps_ubx_ucenter.status = GPS_UBX_UCENTER_STATUS_STOPPED;
#ifdef GPS_I2C
        gps_i2c_begin();
#endif
        gps_ubx_ucenter.cnt = 0;
      } else {
        gps_ubx_ucenter.cnt++;
#ifdef GPS_I2C
        gps_ubx_ucenter.status = GPS_UBX_UCENTER_STATUS_WAITING;
#endif
      }
      break;
#ifdef GPS_I2C
    case GPS_UBX_UCENTER_STATUS_WAITING:
      if (gps_i2c_tx_is_ready()) {
        gps_ubx_ucenter.status = GPS_UBX_UCENTER_STATUS_CONFIG;
      }
      break;
#endif /* GPS_I2C */
    default:
      break;
  }
}

/////////////////////////////
// Event Function
// -fetch replies: ACK and VERSION info

/** Parse "major.minor" from a version string (e.g. "ROM CORE 3.01 (107888)" or "5.00 (28462)") */
static void gps_ubx_ucenter_parse_sw_version(uint8_t *payload)
{
  // swVersion: 30 characters, the version is the first number
  for (uint8_t i = 0; i < 26; i++) {
    uint8_t c = UBX_MON_VER_c(payload, i);
    if (c >= '0' && c <= '9' && UBX_MON_VER_c(payload, i + 1) == '.') {
      gps_ubx_ucenter.sw_ver_h = c - '0';
      gps_ubx_ucenter.sw_ver_l = 10 * (UBX_MON_VER_c(payload, i + 2) - '0') + (UBX_MON_VER_c(payload, i + 3) - '0');
      return;
    }
  }
}

/** Parse the protocol version major from the extensions (30 characters each, after the software and
 *  hardware versions), e.g. "PROTVER=27.31" (generation 8 and later), 0 when not found */
static uint8_t gps_ubx_ucenter_parse_protver(uint8_t *payload, uint16_t len)
{
  static const char protver[] = "PROTVER";
  for (uint16_t ext = 40; ext + 30 <= len; ext += 30) {
    uint8_t i = 0;
    while (i < 7 && UBX_MON_VER_c(payload, ext + i) == protver[i]) {
      i++;
    }
    if (i == 7) {
      // "PROTVER=" or "PROTVER " followed by the major
      uint8_t major = 0;
      for (i = 8; i < 11; i++) {
        uint8_t c = UBX_MON_VER_c(payload, ext + i);
        if (c < '0' || c > '9') {
          break;
        }
        major = 10 * major + (c - '0');
      }
      return major;
    }
  }
  return 0;
}

/** Value of a hexadecimal character (0 when it isn't one) */
static uint8_t gps_ubx_ucenter_hex(char c)
{
  if (c >= '0' && c <= '9') {
    return c - '0';
  } else if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  } else if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  return 0;
}

void gps_ubx_ucenter_event(void)
{
  // Save processing time inflight
  if (gps_ubx_ucenter.status == GPS_UBX_UCENTER_STATUS_STOPPED) {
    return;
  }

  // Read Configuration Reply's
  switch (gps_ubx[0].msg_class) {
    case UBX_ACK_ID:
      if (gps_ubx[0].msg_id == UBX_ACK_ACK_ID) {
        gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_ACK;
      } else if (gps_ubx[0].msg_id == UBX_ACK_NAK_ID) {
        gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NACK;
      }
      break;
    case UBX_MON_ID:
      if (gps_ubx[0].msg_id == UBX_MON_VER_ID) {
        gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_VERSION;
        gps_ubx_ucenter.version_reply = true;
        gps_ubx_ucenter_parse_sw_version(gps_ubx[0].msg_buf);
        gps_ubx_ucenter.protver = gps_ubx_ucenter_parse_protver(gps_ubx[0].msg_buf, gps_ubx[0].len);
        // hwVersion: 10 hexadecimal characters at 30, e.g. "00190000" (generation 9), "000B0000" (X20)
        gps_ubx_ucenter.hw_ver_h = 16 * gps_ubx_ucenter_hex(UBX_MON_VER_c(gps_ubx[0].msg_buf, 32)) +
                                   gps_ubx_ucenter_hex(UBX_MON_VER_c(gps_ubx[0].msg_buf, 33));
        gps_ubx_ucenter.hw_ver_l = 16 * gps_ubx_ucenter_hex(UBX_MON_VER_c(gps_ubx[0].msg_buf, 36)) +
                                   gps_ubx_ucenter_hex(UBX_MON_VER_c(gps_ubx[0].msg_buf, 37));
      } else if (gps_ubx[0].msg_id == UBX_MON_GNSS_ID) {
        gps_ubx_ucenter.gnss_in_use = UBX_MON_GNSS_enabled(gps_ubx[0].msg_buf);
      } else if (gps_ubx[0].msg_id == UBX_MON_COMMS_ID) {
        // Port this reply is output on (X20, 0 when not reported): 1 I2C, 2 UART1, 3 UART2, 4 USB, 5 SPI
        uint8_t output_port = (UBX_MON_COMMS_txErrors(gps_ubx[0].msg_buf) >> 2) & 0x07;
        if (output_port != 0) {
          gps_ubx_ucenter.port_id = output_port - 1;
        }
      }
      break;
    case UBX_CFG_ID:
      if (gps_ubx[0].msg_id == UBX_CFG_PRT_ID) {
        gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_CFG_PRT;
        gps_ubx_ucenter.cfg_prt_reply = true;
        gps_ubx_ucenter.port_id = UBX_CFG_PRT_PortId(gps_ubx[0].msg_buf, 0);
        gps_ubx_ucenter.baud_run = UBX_CFG_PRT_Baudrate(gps_ubx[0].msg_buf, 0);
      }
      break;
    default:
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////
// Autobaud

#ifndef GPS_I2C
/** Baudrates tried, after the target one: the common one, the factory default, the high-rate default,
 *  the NMEA default and the higher ones */
static const uint32_t gps_ubx_ucenter_bauds[] = {B38400, B9600, B57600, B4800, B115200, B230400,
#ifdef B460800
                                                  B460800,
#endif
#ifdef B921600
                                                  B921600,
#endif
                                                 };
#define GPS_UBX_UCENTER_NB_BAUDS (sizeof(gps_ubx_ucenter_bauds) / sizeof(gps_ubx_ucenter_bauds[0]))

/**
 * Poll the port configuration (legacy UBX-CFG-PRT), the version (UBX-MON-VER, all receivers) and the
 * port information (UBX-MON-COMMS, generation 9 and later). Any reply tells that the baudrate is right.
 * The port the autopilot is connected to is reported by CFG-PRT (without a payload: the port the poll
 * was received on) or, on receivers without CFG-PRT (X20), by MON-COMMS.
 */
static void gps_ubx_ucenter_autobaud_poll(void)
{
  gps_ubx_ucenter.cfg_prt_reply = false;
  gps_ubx_ucenter.version_reply = false;
  UbxSend_CFG_PRT_POLL(gps_ubx_ucenter.dev);
  UbxSend_MON_GET_VER(gps_ubx_ucenter.dev);
  UbxSend_MON_GET_COMMS(gps_ubx_ucenter.dev);
}

/**
 * Automatically determine the baudrate of the u-blox module and its configuration interface.
 * Only needed when connecting to a UART port on the u-blox.
 * The discovered baudrate is copied to gps_ubx_ucenter.baud_init.
 *
 * @param nr Autobaud step number to perform
 * @return FALSE when completed
 */
static bool gps_ubx_ucenter_autobaud(uint8_t nr)
{
  // Very important for some modules: give the GPS some time to boot (up to 0.75 second)
  if (nr < 2) {
    return true;
  }

  // The polls of the previous step were answered at that baudrate
  if (nr > 2 && (gps_ubx_ucenter.cfg_prt_reply || gps_ubx_ucenter.version_reply)) {
    gps_ubx_ucenter.baud_init = gps_ubx_ucenter.baud_run;
    // The legacy UBX-CFG messages are gone from protocol version 40 (e.g. X20). The configuration
    // interface exists from 27 (generation 9, F9 HPG 1.00): also used then when the CFG-PRT poll isn't answered.
    gps_ubx_ucenter.valset = gps_ubx_ucenter.protver >= 40 ||
                             (gps_ubx_ucenter.protver >= 27 && !gps_ubx_ucenter.cfg_prt_reply);
    // Not over I2C: when the receiver didn't report the port (still the I2C/DDC default), assume UART1
    if (gps_ubx_ucenter.port_id == GPS_PORT_DDC) {
      gps_ubx_ucenter.port_id = GPS_PORT_UART1;
    }
    return false;
  }

  // Try the target baudrate first (often already configured), then the others
  uint8_t i = nr - 2;
  if (i <= GPS_UBX_UCENTER_NB_BAUDS) {
    uint32_t baud = (i == 0) ? gps_ubx_ucenter.baud_target : gps_ubx_ucenter_bauds[i - 1];
    gps_ubx_ucenter.baud_run = UART_SPEED(baud); // the CFG-PRT reply overwrites it with the reported one
    uart_periph_set_baudrate(&(UBX_GPS_PORT), baud);
    gps_ubx_ucenter_autobaud_poll();
    return true;
  }

  // Autoconfig failed... set the failsafe baudrate
  gps_ubx_ucenter.baud_init = 0; // Set as zero to indicate that we couldn't verify the baudrate
  uart_periph_set_baudrate(&(UBX_GPS_PORT), B9600);
  return false;
}
#endif /* GPS_I2C */

/** Send the configuration replies (DEBUG message) and the receiver information to the ground */
static void gps_ubx_ucenter_send_info(uint8_t sbas)
{
  gps_ubx_ucenter.sbas_in_use = sbas;
#if DEBUG_GPS_UBX_UCENTER
  // To view, enable the DEBUG message in your telemetry configuration
  DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, GPS_UBX_UCENTER_CONFIG_STEPS, gps_ubx_ucenter.replies);
#endif
#ifdef DOWNLINK_TRANSPORT
  uint8_t gnss = gps_ubx_ucenter_gnss();
  DOWNLINK_SEND_UBLOX_INFO(DefaultChannel, DefaultDevice, &gps_ubx_ucenter.baud_run, &gps_ubx_ucenter.sw_ver_h,
                           &gps_ubx_ucenter.sw_ver_l, &gps_ubx_ucenter.hw_ver_h, &gps_ubx_ucenter.hw_ver_l,
                           &gps_ubx_ucenter.sbas_in_use, &gnss);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////
// Legacy UBX-CFG messages

/**
 * Enable u-blox message at desired period.  Will enable the message on the port
 * that this command is received on. For example, sending this configuration message
 * over UART1 will cause the desired message to be published on UART1.
 *
 * @param class u-blox message class
 * @param id u-blox message ID
 * @param rate Desired period to send message. Example: Setting 3 would send the message on every 3rd navigation solution.
 */
static inline void gps_ubx_ucenter_enable_msg(uint8_t class, uint8_t id, uint8_t rate)
{
  UbxSend_CFG_MSG(gps_ubx_ucenter.dev, class, id, rate);
}

#define NAV_DYN_AIRBORNE_2G 6 // CFG-NAV (firmware before 5)

#define NAV5_MASK 0x05 // Apply dynamic model and position fix mode settings

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
  // New ublox firmware v5 or higher uses CFG_NAV5 message, CFG_NAV is no longer available
  // If version message couldn't be fetched, default to NAV5
  if (gps_ubx_ucenter.sw_ver_h < 5 && gps_ubx_ucenter.hw_ver_h < 6 &&
      gps_ubx_ucenter.sw_ver_h != 0 && gps_ubx_ucenter.hw_ver_h != 0) {
    UbxSend_CFG_NAV(gps_ubx_ucenter.dev,
                    NAV_DYN_AIRBORNE_2G, 3, 16, 24, 20, 5, 0, 0x3C,
                    0x3C, 0x14, 0x03E8 , 0x0000, 0x0, 0x17, 0x00FA, 0x00FA,
                    0x0064, 0x012C, 0x000F, 0x00, 0x00);
  } else {
    UbxSend_CFG_NAV5(gps_ubx_ucenter.dev,
                     NAV5_MASK, GPS_UBX_NAV5_DYNAMICS, NAV5_3D_ONLY, IGNORED, IGNORED, NAV5_DEFAULT_MIN_ELEV, RESERVED,
                     NAV5_DEFAULT_PDOP_MASK, NAV5_DEFAULT_TDOP_MASK, NAV5_DEFAULT_P_ACC, NAV5_DEFAULT_T_ACC,
                     NAV5_DEFAULT_STATIC_HOLD_THRES, 125, 0, 0, RESERVED, 0, 0, RESERVED, RESERVED);
  }
}

// UART mode: 8N1 with reserved1 set for compatability with A4
#define UBX_UART_MODE_MASK 0x000008D0

#define UBX_PROTO_MASK    0x0001
#define NMEA_PROTO_MASK   0x0002
#define RTCM_PROTO_MASK   0x0004
#define RTCM3_PROTO_MASK  0x0020

static inline void gps_ubx_ucenter_config_port(void)
{
  switch (gps_ubx_ucenter.port_id) {
    // I2C Interface
    case GPS_PORT_DDC:
#ifdef GPS_I2C
      UbxSend_CFG_PRT(gps_ubx_ucenter.dev, gps_ubx_ucenter.port_id, 0x0, 0x0, (0x42 << 1), 0x0, UBX_PROTO_MASK,
                      UBX_PROTO_MASK, 0x0, 0x0);
#endif
      break;
    // UART Interface
    case GPS_PORT_UART1:
    case GPS_PORT_UART2:
      UbxSend_CFG_PRT(gps_ubx_ucenter.dev,
                      gps_ubx_ucenter.port_id, RESERVED, RESERVED,
                      UBX_UART_MODE_MASK, UART_SPEED(gps_ubx_ucenter.baud_target), UBX_PROTO_MASK | NMEA_PROTO_MASK | RTCM3_PROTO_MASK,
                      UBX_PROTO_MASK | (NMEA_PROTO_MASK & GPS_UBX_ENABLE_NMEA_DATA_MASK), 0x0, 0x0);
      break;
    // USB Interface
    case GPS_PORT_USB:
      UbxSend_CFG_PRT(gps_ubx_ucenter.dev,
                      gps_ubx_ucenter.port_id, 0x0, 0x0, 0x0, 0x0,
                      UBX_PROTO_MASK | NMEA_PROTO_MASK | RTCM3_PROTO_MASK, UBX_PROTO_MASK | (NMEA_PROTO_MASK & GPS_UBX_ENABLE_NMEA_DATA_MASK), 0x0, 0x0);
      break;
    default:
      // SPI is not supported
      break;
  }
}

#define GPS_SBAS_ENABLED       0x01

#define GPS_SBAS_RANGING       0x01
#define GPS_SBAS_CORRECTIONS   0x02
#define GPS_SBAS_INTEGRITY     0x04 // If enabled, the receiver will only use GPS satellites for which integrity information is available (not recommended)

#define GPS_SBAS_MAX_SBAS    3 // Default ublox setting uses 3 SBAS channels(?)

#define GPS_SBAS_AUTOSCAN    0x00

static inline void gps_ubx_ucenter_config_sbas(void)
{
  // Since March 2nd 2011 EGNOS is released for aviation purposes
  UbxSend_CFG_SBAS(gps_ubx_ucenter.dev, GPS_SBAS_ENABLED, GPS_SBAS_RANGING | GPS_SBAS_CORRECTIONS,
                   GPS_SBAS_MAX_SBAS,
                   GPS_SBAS_AUTOSCAN, GPS_SBAS_AUTOSCAN);
}

/** Configure the receiver with the legacy UBX-CFG messages, one step per call (4 Hz) */
static bool gps_ubx_ucenter_configure(uint8_t nr)
{
  // Store the reply of the last configuration step and reset
  if (nr < GPS_UBX_UCENTER_CONFIG_STEPS) {
    gps_ubx_ucenter.replies[nr] = gps_ubx_ucenter.reply;
  }

  switch (nr) {
    case 0:
      // Use old baudrate to issue a baudrate change command
      gps_ubx_ucenter_config_port();
      break;
    case 1:
#ifndef GPS_I2C
      // Now the GPS baudrate should have changed, unless the receiver refused it
      if (gps_ubx_ucenter.reply != GPS_UBX_UCENTER_REPLY_NACK) {
        uart_periph_set_baudrate(&(UBX_GPS_PORT), gps_ubx_ucenter.baud_target);
        gps_ubx_ucenter.baud_run = UART_SPEED(gps_ubx_ucenter.baud_target);
      }
#endif /*GPS_I2C*/
      UbxSend_MON_GET_VER(gps_ubx_ucenter.dev);
      break;
    case 2:
    case 3:
    case 4:
    case 5:
      // UBX_G5010 takes 0.7 seconds to answer a firmware request
      // Version info is important for proper configuration as different firmwares have different settings
      break;
    case 6:
      // Configure CFG-NAV(5) message
      gps_ubx_ucenter_config_nav();
      break;
#if GPS_UBX_UCENTER_PVT_ONLY // disable other messages, except SAT
    case 7:
      gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_POSLLH_ID, 0);
      break;
    case 8:
      gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_VELNED_ID, 0);
      break;
    case 9:
      gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_STATUS_ID, 0);
      break;
    case 10:
      // Satelite information
      gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_SAT_ID, 10);
      break;
    case 11:
      gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_SOL_ID, 0);
      break;
    case 12:
      gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_SVINFO_ID, 0);
      break;
    case 13:
      // Enable Position Velocity time solution
      gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_PVT_ID, 1);
      break;
#else
    case 7:
      // Geodetic Position Solution
      gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_POSLLH_ID, 1);
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
      gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_SVINFO_ID, 10);
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
      // Enable Position Velocity time solution
      gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_PVT_ID, 1);
      break;
#endif // !PVT_ONLY
    case 14:
      // SBAS Configuration
      gps_ubx_ucenter_config_sbas();
      break;
    case 15:
      // Navigation/Measurement Rate Settings
      UbxSend_CFG_RATE(gps_ubx_ucenter.dev, GPS_UBX_UCENTER_RATE_LEGACY, 0x0001, 0x0000);
      break;
    case 16:
      // A slower measurement rate when it is refused (NAK, e.g. the LEA-4 up to 4 Hz)
      if (gps_ubx_ucenter.reply == GPS_UBX_UCENTER_REPLY_NACK &&
          GPS_UBX_UCENTER_RATE_LEGACY < GPS_UBX_UCENTER_RATE_LEGACY_FALLBACK) {
        UbxSend_CFG_RATE(gps_ubx_ucenter.dev, GPS_UBX_UCENTER_RATE_LEGACY_FALLBACK, 0x0001, 0x0000);
      }
      // Raw Measurement Data
#if USE_GPS_UBX_RXM_RAW
      gps_ubx_ucenter_enable_msg(UBX_RXM_ID, UBX_RXM_RAW_ID, 1);
#endif
      break;
    case 17:
      // Subframe Buffer
#if USE_GPS_UBX_RXM_SFRB
      gps_ubx_ucenter_enable_msg(UBX_RXM_ID, UBX_RXM_SFRB_ID, 1);
#endif
      break;
#if USE_GPS_UBX_RTCM
    case 18:
      UbxSend_CFG_DGNSS(gps_ubx_ucenter.dev, 0x03, RESERVED, RESERVED);
      break;
    case 19:
      gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_RELPOSNED_ID, 1);
      break;
    case 20:
      gps_ubx_ucenter_enable_msg(UBX_NAV_ID, UBX_NAV_HPPOSLLH_ID, 1);
      break;
    case 21:
      gps_ubx_ucenter_enable_msg(UBX_RXM_ID, UBX_RXM_RTCM_ID, 1);
      break;
#endif
    case 22:
#if GPS_UBX_UCENTER_SAVE
      // Try to save on non-ROM devices...
      UbxSend_CFG_CFG(gps_ubx_ucenter.dev, 0x00000000, 0xffffffff, 0x00000000);
#endif
      break;
    case 23:
      UbxSend_MON_GET_GNSS(gps_ubx_ucenter.dev);
      break;
    case 24:
      break;
    case 25:
      gps_ubx_ucenter_send_info(GPS_SBAS_RANGING | GPS_SBAS_CORRECTIONS | GPS_SBAS_INTEGRITY);
      return false;
    default:
      break;
  }

  gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
  return true; // Continue, except for the last case
}

//////////////////////////////////////////////////////////////////////////////////////
// Configuration interface (UBX-CFG-VALSET), u-blox generation 9 and later

/** Written to RAM only, saved afterwards like with the legacy messages (UBX-CFG-CFG): flash is not on
 *  every receiver, the whole message would be refused then */
#define VALSET_LAYER_RAM 0x01

/* Configuration keys (u-blox interface description), the value size is in bits 28-30 of the key */
#define CFG_RATE_MEAS            0x30210001 // U2 measurement period [ms]
#define CFG_RATE_NAV             0x30210002 // U2 measurements per navigation solution
#define CFG_NAVSPG_FIXMODE       0x20110011 // E1 position fix mode
#define CFG_NAVSPG_DYNMODEL      0x20110021 // E1 dynamic platform model
#define CFG_UART1_BAUDRATE       0x40520001 // U4
#define CFG_UART2_BAUDRATE       0x40530001
#define CFG_UART1INPROT_RTCM3X   0x10730004 // L RTCM3 corrections in
#define CFG_UART2INPROT_RTCM3X   0x10750004
#define CFG_UART1OUTPROT_NMEA    0x10740002 // L NMEA out
#define CFG_UART2OUTPROT_NMEA    0x10760002
/* Message output rate keys (per navigation solution) for the I2C port, the key for the other ports is
 * the port number (CFG-PRT) higher: UART1 +1, UART2 +2, USB +3, SPI +4 */
#define CFG_MSGOUT_NAV_PVT       0x20910006
#define CFG_MSGOUT_NAV_POSLLH    0x20910029
#define CFG_MSGOUT_NAV_VELNED    0x20910042
#define CFG_MSGOUT_NAV_STATUS    0x2091001a
#define CFG_MSGOUT_NAV_SAT       0x20910015
#define CFG_MSGOUT_NAV_RELPOSNED 0x2091008d
#define CFG_MSGOUT_NAV_DAHEADING 0x209103df // X20D

#if USE_GPS_UBX_RTCM
#define VALSET_RELPOSNED_RATE 1
#else
#define VALSET_RELPOSNED_RATE 0
#endif

/** What a configuration item applies to */
#define VALSET_RECEIVER 0 // the receiver
#define VALSET_UART     1 // the UART the autopilot is connected to: key for UART1, key_uart2 for UART2
#define VALSET_MSGOUT   2 // a message output on the port the autopilot is connected to

/** Configuration, one item per step: the receiver refuses the whole message when it doesn't know one key */
static const struct {
  uint8_t type;
  uint32_t key;
  uint32_t key_uart2;
  uint32_t value;
} gps_ubx_ucenter_valset_cfg[] = {
  { VALSET_UART, CFG_UART1OUTPROT_NMEA, CFG_UART2OUTPROT_NMEA, (GPS_UBX_ENABLE_NMEA_DATA_MASK != 0) },
  { VALSET_UART, CFG_UART1INPROT_RTCM3X, CFG_UART2INPROT_RTCM3X, 1 },
  { VALSET_RECEIVER, CFG_NAVSPG_DYNMODEL, 0, GPS_UBX_NAV5_DYNAMICS },
  { VALSET_RECEIVER, CFG_NAVSPG_FIXMODE, 0, NAV5_3D_ONLY },
  { VALSET_RECEIVER, CFG_RATE_MEAS, 0, GPS_UBX_UCENTER_RATE_VALSET },
  { VALSET_RECEIVER, CFG_RATE_NAV, 0, 1 },
  // Position, velocity and time in one message, the others are not needed
  { VALSET_MSGOUT, CFG_MSGOUT_NAV_PVT, 0, 1 },
  { VALSET_MSGOUT, CFG_MSGOUT_NAV_POSLLH, 0, 0 },
  { VALSET_MSGOUT, CFG_MSGOUT_NAV_VELNED, 0, 0 },
  { VALSET_MSGOUT, CFG_MSGOUT_NAV_STATUS, 0, 0 },
  // Satellite information, every 10th solution
  { VALSET_MSGOUT, CFG_MSGOUT_NAV_SAT, 0, 10 },
  // Relative position (RTK base or dual antenna heading)
  { VALSET_MSGOUT, CFG_MSGOUT_NAV_RELPOSNED, 0, VALSET_RELPOSNED_RATE },
  // Dual antenna heading (X20D, off by default), the other receivers refuse it
  { VALSET_MSGOUT, CFG_MSGOUT_NAV_DAHEADING, 0, 1 },
};
#define VALSET_CFG_NB (sizeof(gps_ubx_ucenter_valset_cfg) / sizeof(gps_ubx_ucenter_valset_cfg[0]))

/** Key of a configuration item for the port the autopilot is connected to, 0 when it doesn't apply */
static uint32_t gps_ubx_ucenter_valset_key(uint8_t type, uint32_t key, uint32_t key_uart2)
{
  switch (type) {
    case VALSET_UART:
      if (gps_ubx_ucenter.port_id == GPS_PORT_UART1) {
        return key;
      } else if (gps_ubx_ucenter.port_id == GPS_PORT_UART2) {
        return key_uart2;
      }
      return 0;
    case VALSET_MSGOUT:
      return key + gps_ubx_ucenter.port_id;
    default:
      return key;
  }
}

/** Set one configuration item in RAM */
static void gps_ubx_ucenter_valset(uint32_t key, uint32_t value)
{
  switch ((key >> 28) & 0x07) {
    case 0: // no key
      break;
    case 1: // one bit
    case 2: // one byte
      UbxSend_CFG_VALSET_U1(gps_ubx_ucenter.dev, 0, VALSET_LAYER_RAM, RESERVED, key, value);
      break;
    case 3: // two bytes
      UbxSend_CFG_VALSET_U2(gps_ubx_ucenter.dev, 0, VALSET_LAYER_RAM, RESERVED, key, value);
      break;
    default: // four bytes
      UbxSend_CFG_VALSET_U4(gps_ubx_ucenter.dev, 0, VALSET_LAYER_RAM, RESERVED, key, value);
      break;
  }
}

/** Configure the receiver with UBX-CFG-VALSET, one step per call (4 Hz) */
static bool gps_ubx_ucenter_configure_valset(uint8_t nr)
{
  // Store the reply of the last configuration step and reset
  if (nr < GPS_UBX_UCENTER_CONFIG_STEPS) {
    gps_ubx_ucenter.replies[nr] = gps_ubx_ucenter.reply;
  }

  if (nr == 0) {
    // Use the found baudrate to change it to the target one
    gps_ubx_ucenter_valset(gps_ubx_ucenter_valset_key(VALSET_UART, CFG_UART1_BAUDRATE, CFG_UART2_BAUDRATE),
                           UART_SPEED(gps_ubx_ucenter.baud_target));
  } else if (nr == 1) {
#ifndef GPS_I2C
    // Now the receiver baudrate should have changed, unless the receiver refused it
    if (gps_ubx_ucenter.reply != GPS_UBX_UCENTER_REPLY_NACK) {
      uart_periph_set_baudrate(&(UBX_GPS_PORT), gps_ubx_ucenter.baud_target);
      gps_ubx_ucenter.baud_run = UART_SPEED(gps_ubx_ucenter.baud_target);
    }
#endif
  } else if (nr < 2 + VALSET_CFG_NB) {
    uint8_t i = nr - 2;
    // A slower measurement rate when it is refused (NAK, e.g. in moving base heading mode)
    if (i > 0 && gps_ubx_ucenter_valset_cfg[i - 1].key == CFG_RATE_MEAS &&
        gps_ubx_ucenter.reply == GPS_UBX_UCENTER_REPLY_NACK &&
        gps_ubx_ucenter_valset_cfg[i - 1].value < GPS_UBX_UCENTER_RATE_VALSET_FALLBACK) {
      gps_ubx_ucenter_valset(CFG_RATE_MEAS, GPS_UBX_UCENTER_RATE_VALSET_FALLBACK);
    }
    gps_ubx_ucenter_valset(gps_ubx_ucenter_valset_key(gps_ubx_ucenter_valset_cfg[i].type,
                           gps_ubx_ucenter_valset_cfg[i].key, gps_ubx_ucenter_valset_cfg[i].key_uart2),
                           gps_ubx_ucenter_valset_cfg[i].value);
  } else if (nr == 2 + VALSET_CFG_NB) {
#if GPS_UBX_UCENTER_SAVE
    // Save the configuration in RAM to battery backed RAM and flash
    UbxSend_CFG_CFG(gps_ubx_ucenter.dev, 0x00000000, 0xffffffff, 0x00000000);
#endif
  } else if (nr == 3 + VALSET_CFG_NB) {
    UbxSend_MON_GET_GNSS(gps_ubx_ucenter.dev);
  } else if (nr == 4 + VALSET_CFG_NB) {
    // Wait for the GNSS reply
  } else {
    gps_ubx_ucenter_send_info(0);
    return false;
  }

  gps_ubx_ucenter.reply = GPS_UBX_UCENTER_REPLY_NONE;
  return true; // Continue, except for the last step
}

int gps_ubx_ucenter_get_status(void)
{
  return gps_ubx_ucenter.status;
}
