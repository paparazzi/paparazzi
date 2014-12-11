/*
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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

/** \file downlink.c
 *  \brief Common code for AP and FBW telemetry
 *
 */


#include "subsystems/datalink/downlink.h"

struct downlink downlink;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#include "subsystems/datalink/datalink.h"
#include "mcu_periph/sys_time.h"

static void send_downlink(struct transport_tx *trans, struct link_device *dev) {
  static uint32_t last_nb_bytes = 0;
  // timestamp in usec when last message was send
  static uint32_t last_ts = 0.;
  // current timestamp
  uint32_t now_ts = get_sys_time_msec();
  // compute downlink byte rate
  if (now_ts > last_ts) {
    uint16_t rate = (1000 * ((uint32_t)downlink.nb_bytes - last_nb_bytes)) / (now_ts - last_ts);
    last_ts = now_ts;
    last_nb_bytes = downlink.nb_bytes;

    // TODO uplink nb received msg
    uint16_t uplink_nb_msgs = 0;
    pprz_msg_send_DATALINK_REPORT(trans, dev, AC_ID,
        &datalink_time, &uplink_nb_msgs,
        &downlink.nb_msgs, &rate, &downlink.nb_ovrn);
  }
}
#endif

void downlink_init(void)
{
  downlink.nb_ovrn = 0;
  downlink.nb_bytes = 0;
  downlink.nb_msgs = 0;

#if defined DATALINK
#if DATALINK == PPRZ || DATALINK == SUPERBITRF || DATALINK == W5100
  pprz_transport_init();
#endif
#if DATALINK == XBEE
  xbee_init();
#endif
#if DATALINK == W5100
  w5100_init();
#endif
#endif

#if SITL
  ivy_transport_init();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "DATALINK_REPORT", send_downlink);
#endif
}

