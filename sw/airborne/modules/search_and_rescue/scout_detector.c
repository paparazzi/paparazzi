/*
 * Copyright (C) 2026 Fabien-B <fabien-B@github.com>
 *                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 */

/** @file "modules/search_and_rescue/scout_detector.c"
 * Scoutector driver
 * Initially made for IMAV 2026 !
 */

#include "modules/search_and_rescue/scout_detector.h"
#include "string.h"
#include "uavcan/uavcan.h"
#include "uavcan.protocol.debug.KeyValue.h"
#include "modules/datalink/downlink.h"
#ifdef SITL
#include "generated/flight_plan.h"
#endif

static uavcan_event scout_detector_uavcan_ev;

scoutector_t scout_data;
struct search_map_t scout_map;

static void scout_detector_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer) {
  struct uavcan_protocol_debug_KeyValue msg;

  if (uavcan_protocol_debug_KeyValue_decode(transfer, &msg)) {
    return;
  }

  if (msg.key.len != 3) {
    return;
  }

  if (strncmp("snr", (const char*)msg.key.data, 3) == 0) {
    // first receive signal to noise value
    scout_data.snr = msg.value;
  } else if (strncmp("lit", (const char*)msg.key.data, 3) == 0) {
    // lit (value in [0,1]) always received after snr
    scout_data.lit = msg.value;
    // FIXME better sync of data or single message !!!
    float data = scout_data.snr * (1.f + scout_data.lit);
    // update map
    search_map_update(&scout_map, data, *stateGetPositionNed_f());
  }
}

void scout_detector_init(void)
{
  // Init map at dummy waypoint (0, 0)
  // proper initialization according to the mission should be done in the flight plan
  search_map_init_from_wp(&scout_map, 0, 1.f);

  uavcan_bind(UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID, UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE, &scout_detector_uavcan_ev, &scout_detector_uavcan_cb);
}

void scout_detector_report(void) {
  if(scout_data.snr > 0 || scout_data.lit > 0) {
    float f[2] = {scout_data.snr, scout_data.lit};
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 2, f);
  }
}

void scout_detector_sim(void) {
#if (defined SITL) && (defined WP_SCOUT)
  struct EnuCoor_f scout = *waypoint_get_enu_f(WP_SCOUT);
  struct EnuCoor_f pos = *stateGetPositionEnu_f();
  struct FloatVect3 dp;
  VECT3_DIFF(dp, pos, scout);
  float dist = float_vect3_norm(&dp);
  // scout at 3 m ~ 95 dB
  // sound level: L(d) = L(d0) - 20 * log10(d/d0)
  // snr = 20 db at 3 m with propellers -> L(d0) = 20
  float snr = 20. - 20.f*log10f(dist/3.f);
  if (snr > 5.f) { // avoid to detect at too long range
    scout_data.snr = snr;
  } else {
    scout_data.snr = 0.f;
  }
  search_map_update(&scout_map, scout_data.snr, *stateGetPositionNed_f());
#endif
}

