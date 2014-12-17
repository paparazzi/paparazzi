/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file subsystems/gps/gps_piksi.c
 *
 * Driver for Piksi modules from Swift-Nav
 *
 * http://docs.swiftnav.com/wiki/Piksi_Integration_Tutorial
 * https://github.com/swift-nav/sbp_tutorial
 */

#include <sbp.h>
#include <sbp_messages.h>
#include "subsystems/gps.h"
#include "mcu_periph/uart.h"
#include "math/pprz_geodetic_double.h"
#if GPS_USE_LATLONG
#include "math/pprz_geodetic_float.h"
#include "subsystems/navigation/common_nav.h"
#include "generated/flight_plan.h"
#endif

#ifndef USE_PIKSI_ECEF
#define USE_PIKSI_ECEF 1
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
#if USE_PIKSI_BASELINE_ECEF
sbp_msg_callbacks_node_t baseline_ecef_node;
#endif
//#if USE_PIKSI_BASELINE_NED
//sbp_msg_callbacks_node_t baseline_ned_node;
//#endif

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
  sbp_pos_ecef_t pos_ecef = *(sbp_pos_ecef_t *)msg;
  gps.ecef_pos.x = (int32_t)(pos_ecef.x * 100.0);
  gps.ecef_pos.y = (int32_t)(pos_ecef.y * 100.0);
  gps.ecef_pos.z = (int32_t)(pos_ecef.z * 100.0);
  gps.pacc = (uint32_t)(pos_ecef.accuracy);
  gps.num_sv = pos_ecef.n_sats;
  gps.fix = GPS_FIX_3D;
  gps.tow = pos_ecef.tow;
}

#if USE_PIKSI_BASELINE_ECEF
static void sbp_baseline_ecef_callback(uint16_t sender_id __attribute__((unused)),
                                       uint8_t len __attribute__((unused)),
                                       uint8_t msg[],
                                       void *context __attribute__((unused)))
{
  sbp_baseline_ecef_t baseline_ecef = *(sbp_baseline_ecef_t *)msg;
  gps.ecef_pos.x = (int32_t)(baseline_ecef.x / 10);
  gps.ecef_pos.y = (int32_t)(baseline_ecef.y / 10);
  gps.ecef_pos.z = (int32_t)(baseline_ecef.z / 10);
  gps.pacc = (uint32_t)(baseline_ecef.accuracy);
  gps.num_sv = baseline_ecef.n_sats;
  gps.tow = baseline_ecef.tow;

  // High precision solution available
  gps_piksi_available = TRUE;
}
#endif

static void sbp_vel_ecef_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len __attribute__((unused)),
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  sbp_vel_ecef_t vel_ecef = *(sbp_vel_ecef_t *)msg;
  gps.ecef_vel.x = (int32_t)(vel_ecef.x / 10);
  gps.ecef_vel.y = (int32_t)(vel_ecef.y / 10);
  gps.ecef_vel.z = (int32_t)(vel_ecef.z / 10);
  gps.sacc = (uint32_t)(vel_ecef.accuracy);

  // Solution available (VEL_ECEF is the last message to be send)
  gps_piksi_available = TRUE;
}

static void sbp_pos_llh_callback(uint16_t sender_id __attribute__((unused)),
                                 uint8_t len __attribute__((unused)),
                                 uint8_t msg[],
                                 void *context __attribute__((unused)))
{
  sbp_pos_llh_t pos_llh = *(sbp_pos_llh_t *)msg;
  gps.lla_pos.lat = (int32_t)(pos_llh.lat * 1e7);
  gps.lla_pos.lon = (int32_t)(pos_llh.lon * 1e7);
  int32_t alt = (int32_t)(pos_llh.height * 1000.);
#if GPS_USE_LATLONG
  /* Computes from (lat, long) in the referenced UTM zone */
  struct LlaCoor_f lla_f;
  LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
  struct UtmCoor_f utm_f;
  utm_f.zone = nav_utm_zone0;
  /* convert to utm */
  utm_of_lla_f(&utm_f, &lla_f);
  /* copy results of utm conversion */
  gps.utm_pos.east = utm_f.east * 100;
  gps.utm_pos.north = utm_f.north * 100;
  gps.utm_pos.alt = gps.lla_pos.alt;
  gps.utm_pos.zone = nav_utm_zone0;
  // height is above ellipsoid or MSL according to bit flag (but not both are available)
  // 0: above ellipsoid
  // 1: above MSL
  // we have to get the HMSL from the flight plan for now
  if (bit_is_set(pos_llh.flags, 3)) {
    gps.hmsl = alt;
    gps.lla_pos.alt = alt + NAV_MSL0;
  } else {
    gps.lla_pos.alt = alt;
    gps.hmsl = alt - NAV_MSL0;
  }
#else
  // but here we fill the two alt with the same value since we don't know HMSL
  gps.lla_pos.alt = alt;
  gps.hmsl = alt;
#endif
}

//#if USE_PIKSI_BASELINE_NED
//static void sbp_baseline_ned_callback(uint16_t sender_id __attribute__((unused)),
//                                      uint8_t len __attribute__((unused)),
//                                      uint8_t msg[],
//                                      void *context __attribute__((unused)))
//{
//  sbp_baseline_ned_t baseline_ned = *(sbp_baseline_ned_t *)msg;
//}
//#endif

static void sbp_vel_ned_callback(uint16_t sender_id __attribute__((unused)),
                                 uint8_t len __attribute__((unused)),
                                 uint8_t msg[],
                                 void *context __attribute__((unused)))
{
  sbp_vel_ned_t vel_ned = *(sbp_vel_ned_t *)msg;
  gps.ned_vel.x = (int32_t)(vel_ned.n / 10);
  gps.ned_vel.y = (int32_t)(vel_ned.e / 10);
  gps.ned_vel.z = (int32_t)(vel_ned.d / 10);
#if GPS_USE_LATLONG
  gps.gspeed = int32_sqrt(gps.ned_vel.x * gps.ned_vel.x + gps.ned_vel.y * gps.ned_vel.y);
  gps.course = (int32_t)(1e7 * atan2(gps.ned_vel.y, gps.ned_vel.x));
#endif
}

static void sbp_dops_callback(uint16_t sender_id __attribute__((unused)),
                              uint8_t len __attribute__((unused)),
                              uint8_t msg[],
                              void *context __attribute__((unused)))
{
  sbp_dops_t dops = *(sbp_dops_t *)msg;
  gps.pdop = dops.pdop;
}

static void sbp_gps_time_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len __attribute__((unused)),
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  sbp_gps_time_t gps_time = *(sbp_gps_time_t *)msg;
  gps.week = gps_time.wn;
  gps.tow = gps_time.tow;
}

bool_t gps_piksi_available;

void gps_impl_init(void)
{
  gps_piksi_available = FALSE;

  /* Setup SBP nodes */
  sbp_state_init(&sbp_state);
  /* Register a node and callback, and associate them with a specific message ID. */
  sbp_register_callback(&sbp_state, SBP_POS_ECEF, &sbp_pos_ecef_callback, NULL, &pos_ecef_node);
  sbp_register_callback(&sbp_state, SBP_VEL_ECEF, &sbp_vel_ecef_callback, NULL, &vel_ecef_node);
  sbp_register_callback(&sbp_state, SBP_POS_LLH, &sbp_pos_llh_callback, NULL, &pos_llh_node);
  sbp_register_callback(&sbp_state, SBP_VEL_NED, &sbp_vel_ned_callback, NULL, &vel_ned_node);
  sbp_register_callback(&sbp_state, SBP_DOPS, &sbp_dops_callback, NULL, &dops_node);
  sbp_register_callback(&sbp_state, SBP_GPS_TIME, &sbp_gps_time_callback, NULL, &gps_time_node);
#if USE_PIKSI_BASELINE_ECEF
  sbp_register_callback(&sbp_state, SBP_BASELINE_ECEF, &sbp_baseline_ecef_callback, NULL, &baseline_ecef_node);
#endif
//#if USE_PIKSI_BASELINE_NED
//  sbp_register_callback(&sbp_state, SBP_BASELINE_NED, &sbp_baseline_ned_callback, NULL, &baseline_ned_node);
//#endif
}


/*
 * FIFO to hold received UART bytes before
 * libswiftnav SBP submodule parses them.
 */
#define FIFO_LEN 512
char sbp_msg_fifo[FIFO_LEN];

/* FIFO functions */
uint8_t fifo_empty(void);
uint8_t fifo_full(void);
uint8_t fifo_write(char c);
uint8_t fifo_read_char(char *c);
uint32_t fifo_read(uint8_t *buff, uint32_t n, void *context);


/*
 * Event function
 */
void gps_piksi_event(void)
{
  // fill fifo with new uart bytes
  while (uart_char_available(&(GPS_LINK))) {
    uint8_t c = uart_getch(&(GPS_LINK));
    fifo_write(c);
  }
  // call sbp event function
  sbp_process(&sbp_state, &fifo_read);
}

/*
 * FIFO implementation
 */
uint16_t head = 0;
uint16_t tail = 0;

/* Return 1 if true, 0 otherwise. */
uint8_t fifo_empty(void)
{
  if (head == tail) {
    return 1;
  }
  return 0;
}

/*
 * Append a character to our SBP message fifo.
 * Returns 1 if char successfully appended to fifo.
 * Returns 0 if fifo is full.
 */
uint8_t fifo_write(char c)
{
  if (fifo_full()) {
    return 0;
  }
  sbp_msg_fifo[tail] = c;
  tail = (tail + 1) % FIFO_LEN;
  return 1;
}

/*
 * Read 1 char from fifo.
 * Returns 0 if fifo is empty, otherwise 1.
 */
uint8_t fifo_read_char(char *c)
{
  if (fifo_empty()) {
    return 0;
  }
  *c = sbp_msg_fifo[head];
  head = (head + 1) % FIFO_LEN;
  return 1;
}

/*
 * Read arbitrary number of chars from FIFO. Must conform to
 * function definition that is passed to the function
 * sbp_process().
 * Returns the number of characters successfully read.
 */
uint32_t fifo_read(uint8_t *buff, uint32_t n, void *context __attribute__((unused)))
{
  uint32_t i;
  for (i = 0; i < n; i++) {
    if (!fifo_read_char((char *)(buff + i))) {
      break;
    }
  }
  return i;
}

/* Return 1 if true, 0 otherwise. */
uint8_t fifo_full(void)
{
  if (((tail + 1) % FIFO_LEN) == head) {
    return 1;
  }
  return 0;
}


