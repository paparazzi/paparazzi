/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file modules/gps/gps_piksi.c
 *
 * Driver for Piksi modules from Swift-Nav
 *
 * http://docs.swiftnav.com/wiki/Piksi_Integration_Tutorial
 * https://github.com/swift-nav/sbp_tutorial
 */

#include "modules/gps/gps_piksi.h"
#include "modules/gps/gps.h"
#include "modules/core/abi.h"
#include "mcu_periph/uart.h"
#include "math/pprz_geodetic_double.h"

// get NAV_MSL0 for geoid separation
#include "generated/flight_plan.h"

#include <libsbp/sbp.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/tracking.h>
#include <libsbp/system.h>
#include <libsbp/settings.h>
#include <libsbp/piksi.h>

#define SBP_FIX_MODE_SPP 0X00
#define SBP_FIX_MODE_FLOAT 0X02
#define SPB_FIX_MODE_FIXED 0X01

#define POS_ECEF_TIMEOUT 1000

/*
 * implementation specific gps state
 */
struct GpsState gps_piksi;
struct GpsTimeSync gps_piksi_time_sync;

static uint32_t time_since_last_heartbeat;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_piksi_heartbeat(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_PIKSI_HEARTBEAT(trans, dev, AC_ID,
                        &time_since_last_heartbeat);
}

#endif

/*
 * Set the Piksi GPS antenna (default is Patch, internal)
 */
#if USE_PIKSI_EXT_ANTENNA
static const char SBP_ANT_SET[] = "frontend""\x00""antenna_selection""\x00""External";
#elif USE_PIKSI_AUTO_ANTENNA
static const char SBP_ANT_SET[] = "frontend""\x00""antenna_selection""\x00""Auto";
#else
static const char SBP_ANT_SET[] = "frontend""\x00""antenna_selection""\x00""Patch";
#endif

/*
 * Set the UART config depending on which UART is connected
 */
#if USE_PIKSI_UARTA
static const char SBP_UART_SET1[] = "uart_uarta""\x00""mode""\x00""SBP";
static const char SBP_UART_SET2[] = "uart_uarta""\x00""sbp_message_mask""\x00""784"; //0x310 which masks all navigation and tracking messages
static const char SBP_UART_SET3[] = "uart_uarta""\x00""configure_telemetry_radio_on_boot""\x00""False";
#else
static const char SBP_UART_SET1[] = "uart_uartb""\x00""mode""\x00""SBP";
static const char SBP_UART_SET2[] = "uart_uartb""\x00""sbp_message_mask""\x00""784"; //0x310 which masks all navigation and tracking messages
static const char SBP_UART_SET3[] = "uart_uartb""\x00""configure_telemetry_radio_on_boot""\x00""False";
#endif

/*
 * State of the SBP message parser.
 * Must be statically allocated.
 */
sbp_state_t sbp_state;

/*
 * SBP callback nodes must be statically allocated. Each message ID / callback
 * pair must have a unique sbp_msg_callbacks_node_t associated with it.
 */
sbp_msg_callbacks_node_t pos_ecef_node;
sbp_msg_callbacks_node_t vel_ecef_node;
sbp_msg_callbacks_node_t pos_llh_node;
sbp_msg_callbacks_node_t vel_ned_node;
sbp_msg_callbacks_node_t dops_node;
sbp_msg_callbacks_node_t gps_time_node;
sbp_msg_callbacks_node_t tracking_state_node;
sbp_msg_callbacks_node_t tracking_state_dep_a_node;
sbp_msg_callbacks_node_t heartbeat_node;


static void gps_piksi_publish(void);
static uint8_t get_fix_mode(uint8_t flags);
uint32_t gps_piksi_read(uint8_t *buff, uint32_t n, void *context __attribute__((unused)));
uint32_t gps_piksi_write(uint8_t *buff, uint32_t n, void *context __attribute__((unused)));
static uint32_t time_since_last_pos_update;

/*
 * Callback functions to interpret SBP messages.
 * Every message ID has a callback associated with it to
 * receive and interpret the message payload.
 */
static void sbp_pos_ecef_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len __attribute__((unused)),
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  time_since_last_pos_update = get_sys_time_msec();
  msg_pos_ecef_t pos_ecef = *(msg_pos_ecef_t *)msg;

  // Check if we got RTK fix (FIXME when libsbp has a nicer way of doing this)
  gps_piksi.fix = get_fix_mode(pos_ecef.flags);
  // get_fix_mode() will still return fix > 3D even if the current flags are spp so ignore when it is spp
  if ( ( (gps_piksi.fix > GPS_FIX_3D) )
    && pos_ecef.flags == SBP_FIX_MODE_SPP) {
    return;
  }

  gps_piksi.ecef_pos.x = (int32_t)(pos_ecef.x * 100.0);
  gps_piksi.ecef_pos.y = (int32_t)(pos_ecef.y * 100.0);
  gps_piksi.ecef_pos.z = (int32_t)(pos_ecef.z * 100.0);
  SetBit(gps_piksi.valid_fields, GPS_VALID_POS_ECEF_BIT);
  gps_piksi.pacc = (uint32_t)(pos_ecef.accuracy);// FIXME not implemented yet by libswiftnav
  gps_piksi.num_sv = pos_ecef.n_sats;
  gps_piksi.tow = pos_ecef.tow;
  gps_piksi_publish(); // Only if RTK position
}

static void sbp_vel_ecef_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len __attribute__((unused)),
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  msg_vel_ecef_t vel_ecef = *(msg_vel_ecef_t *)msg;
  gps_piksi.ecef_vel.x = (int32_t)(vel_ecef.x / 10);
  gps_piksi.ecef_vel.y = (int32_t)(vel_ecef.y / 10);
  gps_piksi.ecef_vel.z = (int32_t)(vel_ecef.z / 10);
  SetBit(gps_piksi.valid_fields, GPS_VALID_VEL_ECEF_BIT);
  gps_piksi.sacc = (uint32_t)(vel_ecef.accuracy);

  // Solution available (VEL_ECEF is the last message to be send)
  gps_piksi_publish(); // TODO: filter out if got RTK position
}

static void sbp_pos_llh_callback(uint16_t sender_id __attribute__((unused)),
                                 uint8_t len __attribute__((unused)),
                                 uint8_t msg[],
                                 void *context __attribute__((unused)))
{
  static uint8_t last_flags = 0;
  msg_pos_llh_t pos_llh = *(msg_pos_llh_t *)msg;

  // Check if we got RTK fix (FIXME when libsbp has a nicer way of doing this)
  if (pos_llh.flags > 0 || last_flags == 0) {
    gps_piksi.lla_pos.lat = (int32_t)(pos_llh.lat * 1e7);
    gps_piksi.lla_pos.lon = (int32_t)(pos_llh.lon * 1e7);

    int32_t alt = (int32_t)(pos_llh.height * 1000.);
    // height is above ellipsoid or MSL according to bit flag (but not both are available)
    // 0: above ellipsoid
    // 1: above MSL
    // we have to get the HMSL from the flight plan for now
    if (bit_is_set(pos_llh.flags, 3)) {
      gps_piksi.hmsl = alt;
      gps_piksi.lla_pos.alt = alt + NAV_MSL0;
    } else {
      gps_piksi.lla_pos.alt = alt;
      gps_piksi.hmsl = alt - NAV_MSL0;
    }

    SetBit(gps_piksi.valid_fields, GPS_VALID_POS_LLA_BIT);
    SetBit(gps_piksi.valid_fields, GPS_VALID_HMSL_BIT);
  }
  last_flags = pos_llh.flags;
}

static void sbp_vel_ned_callback(uint16_t sender_id __attribute__((unused)),
                                 uint8_t len __attribute__((unused)),
                                 uint8_t msg[],
                                 void *context __attribute__((unused)))
{
  msg_vel_ned_t vel_ned = *(msg_vel_ned_t *)msg;
  gps_piksi.ned_vel.x = (int32_t)(vel_ned.n / 10);
  gps_piksi.ned_vel.y = (int32_t)(vel_ned.e / 10);
  gps_piksi.ned_vel.z = (int32_t)(vel_ned.d / 10);
  SetBit(gps_piksi.valid_fields, GPS_VALID_VEL_NED_BIT);

  gps_piksi.gspeed = int32_sqrt(gps_piksi.ned_vel.x * gps_piksi.ned_vel.x + gps_piksi.ned_vel.y * gps_piksi.ned_vel.y);
  gps_piksi.course = (int32_t)(1e7 * atan2(gps_piksi.ned_vel.y, gps_piksi.ned_vel.x));
  SetBit(gps_piksi.valid_fields, GPS_VALID_COURSE_BIT);
}

static void sbp_dops_callback(uint16_t sender_id __attribute__((unused)),
                              uint8_t len __attribute__((unused)),
                              uint8_t msg[],
                              void *context __attribute__((unused)))
{
  msg_dops_t dops = *(msg_dops_t *)msg;
  gps_piksi.pdop = dops.pdop;
}

static void sbp_gps_time_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len __attribute__((unused)),
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  msg_gps_time_t gps_time = *(msg_gps_time_t *)msg;
  gps_piksi.week = gps_time.wn;
  gps_piksi.tow = gps_time.tow;
}

static void sbp_tracking_state_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len,
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  uint8_t channels_cnt = len/sizeof(tracking_channel_state_t);
  msg_tracking_state_t *tracking_state = (msg_tracking_state_t *)msg;

  for(uint8_t i = 0; i < channels_cnt; i++) {
    if(tracking_state->states[i].state == 1) {
      gps_piksi.svinfos[i].svid = tracking_state->states[i].sid + 1;
      gps_piksi.svinfos[i].cno  = tracking_state->states[i].cn0;
    }
  }
}

static void sbp_tracking_state_dep_a_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len,
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  uint8_t channels_cnt = len/sizeof(tracking_channel_state_dep_a_t);
  msg_tracking_state_dep_a_t *tracking_state = (msg_tracking_state_dep_a_t *)msg;

  for(uint8_t i = 0; i < channels_cnt; i++) {
    if(tracking_state->states[i].state == 1) {
      gps_piksi.svinfos[i].svid = tracking_state->states[i].prn + 1;
      gps_piksi.svinfos[i].cno  = tracking_state->states[i].cn0;
    }
  }
}

static void spb_heartbeat_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len __attribute__((unused)),
                                  uint8_t msg[] __attribute__((unused)),
                                  void *context __attribute__((unused)))
{
  time_since_last_heartbeat = get_sys_time_msec();
}

/*
 * Return fix mode based on present and past flags
 */
 static uint8_t get_fix_mode(uint8_t flags)
{
  static uint8_t n_since_last_rtk = 0;
  if (flags == SBP_FIX_MODE_SPP) {
    n_since_last_rtk++;
    if ( n_since_last_rtk > 2 ) {
      return GPS_FIX_3D;
    } else {
      return gps.fix;
    }
  } else if (flags == SBP_FIX_MODE_FLOAT) {
    n_since_last_rtk = 0;
    return GPS_FIX_DGPS;
  } else if (flags == SPB_FIX_MODE_FIXED) {
    n_since_last_rtk = 0;
    return GPS_FIX_RTK;
  } else {
    return GPS_FIX_NONE;
  }

}

/*
 * Initialize the Piksi GPS and write the settings
 */
void gps_piksi_init(void)
{
  /* Setup SBP nodes */
  sbp_state_init(&sbp_state);

  /* Register a node and callback, and associate them with a specific message ID. */
  sbp_register_callback(&sbp_state, SBP_MSG_POS_ECEF, &sbp_pos_ecef_callback, NULL, &pos_ecef_node);
  sbp_register_callback(&sbp_state, SBP_MSG_VEL_ECEF, &sbp_vel_ecef_callback, NULL, &vel_ecef_node);
  sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, &sbp_pos_llh_callback, NULL, &pos_llh_node);
  sbp_register_callback(&sbp_state, SBP_MSG_VEL_NED, &sbp_vel_ned_callback, NULL, &vel_ned_node);
  sbp_register_callback(&sbp_state, SBP_MSG_DOPS, &sbp_dops_callback, NULL, &dops_node);
  sbp_register_callback(&sbp_state, SBP_MSG_GPS_TIME, &sbp_gps_time_callback, NULL, &gps_time_node);
  sbp_register_callback(&sbp_state, SBP_MSG_TRACKING_STATE, &sbp_tracking_state_callback, NULL, &tracking_state_node);
  sbp_register_callback(&sbp_state, SBP_MSG_TRACKING_STATE_DEP_A, &sbp_tracking_state_dep_a_callback, NULL, &tracking_state_dep_a_node);
  sbp_register_callback(&sbp_state, SBP_MSG_HEARTBEAT, &spb_heartbeat_callback, NULL, &heartbeat_node);

  /* Write settings */
  sbp_send_message(&sbp_state, SBP_MSG_SETTINGS_WRITE, SBP_SENDER_ID, sizeof(SBP_ANT_SET), (u8*)(&SBP_ANT_SET), gps_piksi_write);
  sbp_send_message(&sbp_state, SBP_MSG_SETTINGS_WRITE, SBP_SENDER_ID, sizeof(SBP_UART_SET1), (u8*)(&SBP_UART_SET1), gps_piksi_write);
  sbp_send_message(&sbp_state, SBP_MSG_SETTINGS_WRITE, SBP_SENDER_ID, sizeof(SBP_UART_SET2), (u8*)(&SBP_UART_SET2), gps_piksi_write);
  sbp_send_message(&sbp_state, SBP_MSG_SETTINGS_WRITE, SBP_SENDER_ID, sizeof(SBP_UART_SET3), (u8*)(&SBP_UART_SET3), gps_piksi_write);
  sbp_send_message(&sbp_state, SBP_MSG_SETTINGS_SAVE, SBP_SENDER_ID, 0, NULL, gps_piksi_write);
  /*msg_base_pos_t base_pos;
  base_pos.lat = 51.991152;
  base_pos.lon = 4.378052;
  base_pos.height = 50.;
  sbp_send_message(&sbp_state, SBP_MSG_BASE_POS, SBP_SENDER_ID, sizeof(msg_base_pos_t), (u8*)(&base_pos), gps_piksi_write);*/

  // gps.nb_channels = GPS_NB_CHANNELS;
  gps_piksi.comp_id = GPS_PIKSI_ID;

  gps_piksi.nb_channels = 10;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PIKSI_HEARTBEAT, send_piksi_heartbeat);
#endif
}

/*
 * Event handler for reading the GPS UART bytes
 */
void gps_piksi_event(void)
{
  if ( get_sys_time_msec() - time_since_last_pos_update > POS_ECEF_TIMEOUT ) {
    gps_piksi.fix = GPS_FIX_NONE;
  }
  // call sbp event function
  if (uart_char_available(&(PIKSI_GPS_LINK)))
    sbp_process(&sbp_state, &gps_piksi_read);
}

/*
 * Publish the GPS data from the Piksi on the ABI bus
 */
static void gps_piksi_publish(void)
{
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();

  gps_piksi.last_msg_ticks = sys_time.nb_sec_rem;
  gps_piksi.last_msg_time = sys_time.nb_sec;
  if (gps_piksi.fix >= GPS_FIX_3D) {
    gps_piksi.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps_piksi.last_3dfix_time = sys_time.nb_sec;
  }
  AbiSendMsgGPS(GPS_PIKSI_ID, now_ts, &gps_piksi);
}

/*
 * Read bytes from the Piksi UART connection
 * This is a wrapper functions used in the libsbp library
 */
uint32_t gps_piksi_read(uint8_t *buff, uint32_t n, void *context __attribute__((unused)))
{
  uint32_t i;
  for (i = 0; i < n; i++) {
    if (!uart_char_available(&(PIKSI_GPS_LINK)))
      break;

    buff[i] = uart_getch(&(PIKSI_GPS_LINK));
  }
  return i;
}

/*
 * Write bytes to the Piksi UART connection
 * This is a wrapper functions used in the libsbp library
 */
uint32_t gps_piksi_write(uint8_t *buff, uint32_t n, void *context __attribute__((unused)))
{
  uint32_t i = 0;
  for (i = 0; i < n; i++) {
    uart_put_byte(&(PIKSI_GPS_LINK), 0, buff[i]);
  }
  return n;
}

/**
 * Override the default GPS packet injector to inject the data trough UART
 */
void gps_inject_data(uint8_t packet_id, uint8_t length, uint8_t *data)
{
  sbp_send_message(&sbp_state, packet_id, SBP_SENDER_ID, length, data, gps_piksi_write);
}
