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

/** @file gps_ubx.h
 * @brief UBX protocol specific code
 *
 */

#ifndef GPS_UBX_H
#define GPS_UBX_H

#include "subsystems/gps.h"

#ifdef GPS_CONFIGURE
#warning "Please use gps_ubx_ucenter.xml module instead of GPS_CONFIGURE"
#endif

#ifdef GPS_I2C
#include "modules/gps/gps_ubx_i2c.h"
#else
#include "mcu_periph/uart.h"
#endif

#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_UBX
#endif

extern void gps_ubx_init(void);
extern void gps_ubx_event(void);

#define gps_ubx_periodic_check() gps_periodic_check(&gps_ubx.state)

#define GPS_UBX_MAX_PAYLOAD 512
struct GpsUbx {
  bool msg_available;
  uint8_t msg_buf[GPS_UBX_MAX_PAYLOAD] __attribute__((aligned));
  uint8_t msg_id;
  uint8_t msg_class;

  uint8_t status;
  uint16_t len;
  uint16_t msg_idx;
  uint8_t ck_a, ck_b;
  uint8_t send_ck_a, send_ck_b;
  uint8_t error_cnt;
  uint8_t error_last;

  uint8_t status_flags;
  uint8_t sol_flags;

  struct GpsState state;
};

extern struct GpsUbx gps_ubx;

/*
 * This part is used by the autopilot to read data from a uart
 */
#include "pprzlink/pprzlink_device.h"

extern void ubx_header(struct link_device *dev, uint8_t nav_id, uint8_t msg_id, uint16_t len);
extern void ubx_trailer(struct link_device *dev);
extern void ubx_send_bytes(struct link_device *dev, uint8_t len, uint8_t *bytes);
extern void ubx_send_cfg_rst(struct link_device *dev, uint16_t bbr, uint8_t reset_mode);

extern void gps_ubx_read_message(void);
extern void gps_ubx_parse(uint8_t c);
extern void gps_ubx_msg(void);

/*
 * GPS Reset
 */

#define CFG_RST_Reset_Hardware 0x00
#define CFG_RST_Reset_Controlled 0x01
#define CFG_RST_Reset_Controlled_GPS_only 0x02
#define CFG_RST_Reset_Controlled_GPS_stop 0x08
#define CFG_RST_Reset_Controlled_GPS_start 0x09

#define CFG_RST_BBR_Hotstart  0x0000
#define CFG_RST_BBR_Warmstart 0x0001
#define CFG_RST_BBR_Coldstart 0xffff

#define gps_ubx_Reset(_val) {                               \
    gps_ubx.reset = _val;                                       \
    if (gps_ubx.reset > CFG_RST_BBR_Warmstart)                  \
      gps_ubx.reset = CFG_RST_BBR_Coldstart;                    \
    ubx_send_cfg_rst(&(UBX_GPS_LINK).device, gps_ubx.reset, CFG_RST_Reset_Controlled);   \
  }

#endif /* GPS_UBX_H */
