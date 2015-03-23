/*
 * Copyright (C) 2011 The Paparazzi Team
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


#ifndef MTK_H
#define MTK_H

#include "mcu_periph/uart.h"

/** Includes macros generated from mtk.xml */
#include "mtk_protocol.h"

#define GPS_MTK_MAX_PAYLOAD 255

struct GpsMtk {
  bool_t msg_available;
  uint8_t msg_buf[GPS_MTK_MAX_PAYLOAD] __attribute__((aligned));
  uint8_t msg_id;
  uint8_t msg_class;

  uint8_t status;
  uint16_t len;
  uint8_t msg_idx;
  uint8_t ck_a, ck_b;
  uint8_t send_ck_a, send_ck_b;
  uint8_t error_cnt;
  uint8_t error_last;

  uint8_t status_flags;
  uint8_t sol_flags;
};

extern struct GpsMtk gps_mtk;


/*
 * This part is used by the autopilot to read data from a uart
 */
#include "mcu_periph/link_device.h"

#ifdef GPS_CONFIGURE
extern bool_t gps_configuring;
#define GpsConfigure() {            \
    if (gps_configuring)            \
      gps_configure();              \
  }
#else
#define GpsConfigure() {}
#endif

extern void gps_mtk_read_message(void);
extern void gps_mtk_parse(uint8_t c);

static inline void GpsEvent(void (* _sol_available_callback)(void))
{
  struct link_device *dev = &((GPS_LINK).device);

  if (dev->char_available(dev->periph)) {
    while (dev->char_available(dev->periph) && !gps_mtk.msg_available) {
      gps_mtk_parse(dev->get_byte(dev->periph));
    }
    GpsConfigure();
  }
  if (gps_mtk.msg_available) {
    gps_mtk_msg(_sol_available_callback);
  }
}

/*
 * dynamic GPS configuration
 */
#ifdef GPS_CONFIGURE
extern void gps_configure(void);
extern void gps_configure_uart(void);
#endif

#endif /* MTK_H */
