/*
 * Copyright (C) C. De Wagter
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
/**
 * @file "modules/sideslip/sideslip.c"
 * @author C. De Wagter
 * sideslip-measurements
 */

#include "modules/sideslip/sideslip.h"

#include "subsystems/imu.h"

float low_pass_filtered_sideslip_value = 0.0f;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void sideslip_send_telem(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SIDESLIP(trans, dev, AC_ID, &low_pass_filtered_sideslip_value);
}
#endif


void sideslip_init() {
  low_pass_filtered_sideslip_value = 0.0f;
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SIDESLIP, sideslip_send_telem);
#endif
}

void sideslip_periodic() {
  float acc_lat = ACCEL_FLOAT_OF_BFP(imu.accel.y);            ///< accelerometer measurements in m/s^2 in BFP with #INT32_ACCEL_FRAC

  low_pass_filtered_sideslip_value += (acc_lat - low_pass_filtered_sideslip_value) / 100;
}


