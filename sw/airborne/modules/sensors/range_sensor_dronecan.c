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

/** @file modules/sensors/range_sensor_dronecan.c
 * Range sensor on the dronecan bus
 */

#include "range_sensor_dronecan.h"
#include "dronecan/dronecan.h"
#include "core/abi.h"

/* Local structure */
struct range_sensor_dronecan_t {
  float range;
  uint8_t reading_type;
};

/* Local variables */
static struct range_sensor_dronecan_t range_sensor_dronecan = {0};
static dronecan_event range_sensor_dronecan_ev;

// #if FDCAN_PERIPH
// static const FDCANExtendedFilter filters[] = {
//   {
//     0x00041A00, // filter RangeSensorMeasurement broadcast
//     FILTERING_FEC_FIFO_1,
//     0x00FFFF80, // mask
//     0,
//     FILTERING_FT_MASK // classic filter-mask mode
//   }
// };
// #endif

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void range_sensor_dronecan_send_lidar(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t nul = 0;
  pprz_msg_send_LIDAR(trans, dev, AC_ID,
                      &range_sensor_dronecan.range,
                      &range_sensor_dronecan.reading_type,
                      &nul);
}

#endif

static void range_sensor_dronecan_cb(struct dronecan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer) {
  struct uavcan_equipment_range_sensor_Measurement measurement;
  bool decode_error;
  decode_error = uavcan_equipment_range_sensor_Measurement_decode(transfer, &measurement);
  if (!decode_error){
    range_sensor_dronecan.range = measurement.range;
    range_sensor_dronecan.reading_type = measurement.reading_type;

    // Send the range over ABI
    if(!isnan(range_sensor_dronecan.range)) {
      uint32_t now_ts = get_sys_time_usec();
      AbiSendMsgAGL(AGL_DRONECAN_ID, now_ts, range_sensor_dronecan.range);
    }
  }
}

void range_sensor_dronecan_init(void)
{
// #if FDCAN_PERIPH
// #if DRONECAN_USE_CAN1
//   canSTM32SetExtendedFilters(&dronecan1->can_driver, 1, filters);
// #endif
// #if DRONECAN_USE_CAN2
//   canSTM32SetExtendedFilters(&dronecan2->can_driver, 1, filters);
// #endif
// #endif

  // Bind dronecan MEASUREMENT message from EQUIPMENT.RANGE_SENSOR
  dronecan_bind(CanardTransferTypeBroadcast, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE, &range_sensor_dronecan_ev, &range_sensor_dronecan_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LIDAR, range_sensor_dronecan_send_lidar);
#endif
}
