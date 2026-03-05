/*
 * Copyright (C) 2023 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file modules/sensors/range_sensor_uavcan.c
 * Range sensor on the uavcan bus
 */

#include "range_sensor_uavcan.h"
#include "uavcan/uavcan.h"
#include "core/abi.h"
#include "uavcan.equipment.range_sensor.Measurement.h"


/* Local variables */
static struct uavcan_equipment_range_sensor_Measurement range_sensor_uavcan = {0};
static uavcan_event range_sensor_uavcan_ev;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void range_sensor_uavcan_send_lidar(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t nul = 0;
  pprz_msg_send_LIDAR(trans, dev, AC_ID,
                      &range_sensor_uavcan.range,
                      &range_sensor_uavcan.reading_type,
                      &nul);
}

#endif

static void range_sensor_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer) {
  if(uavcan_equipment_range_sensor_Measurement_decode(transfer, &range_sensor_uavcan)) {
    return;   // decode error
  }

  // Send the range over ABI
  if(!isnan(range_sensor_uavcan.range)) {
    uint32_t now_ts = get_sys_time_usec();
    AbiSendMsgAGL(AGL_UAVCAN_ID, now_ts, range_sensor_uavcan.range);
  }
}

void range_sensor_uavcan_init(void)
{
  // Bind uavcan MEASUREMENT message from EQUIPMENT.RANGE_SENSOR
  uavcan_bind(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE, &range_sensor_uavcan_ev, &range_sensor_uavcan_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LIDAR, range_sensor_uavcan_send_lidar);
#endif
}
