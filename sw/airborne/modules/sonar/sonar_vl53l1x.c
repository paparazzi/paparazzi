/*
 * Copyright (C) 2020 Tom van Dijk <tomvand@users.noreply.github.com>
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

#include "sonar_vl53l1x.h"

#include "mcu_periph/i2c.h"
#include "peripherals/vl53l1x_nonblocking.h"
#include "subsystems/abi.h"

#ifdef SITL
#include "state.h"
#endif

#include "subsystems/datalink/downlink.h"


#ifndef SONAR_VL53L1X_I2C_ADDR
#define SONAR_VL53L1X_I2C_ADDR 0x52
#endif

// Signed distance offset in mm
#ifndef SONAR_VL53L1X_OFFSET
#define SONAR_VL53L1X_OFFSET 0
#endif


/* VL53L1X configuration */
// Time budget for single measurement
// Allowed values: 15, 20, 33, 50, 100, 200, 500
// see VL53L1X_SetTimingBudgetInMs
#ifndef SONAR_VL53L1X_TIMINGBUDGET_MS
#define SONAR_VL53L1X_TIMINGBUDGET_MS 100
#endif

// Allowed values: 1 (short, max ~1.3m), 2 (long, max ~4m)
// see VL53L1X_SetDistanceMode
#ifndef SONAR_VL53L1X_DISTANCEMODE
#define SONAR_VL53L1X_DISTANCEMODE 2
#endif

// Time between measurements
// Should be larger than or equal to timing budget
// see VL53L1X_SetInterMeasurementInMs
// Note: may be limited by module periodic frequency
#ifndef SONAR_VL53L1X_INTERMEASUREMENT_MS
#define SONAR_VL53L1X_INTERMEASUREMENT_MS SONAR_VL53L1X_TIMINGBUDGET_MS
#endif
#if SONAR_VL53L1X_INTERMEASUREMENT_MS < SONAR_VL53L1X_TIMINGBUDGET_MS
#warning SONAR_VL53L1X_INTERMEASUREMENT_MS should be greater than or equal to SONAR_VL53L1X_TIMINGBUDGET_MS
#endif


struct sonar_vl53l1x_dev sonar_vl53l1x;


static void sonar_vl53l1x_publish(uint16_t range_mm)
{
  float range_ofs_m = (range_mm + sonar_vl53l1x.offset_mm) * 1.0e-3f;

  // Send ABI message
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgAGL(AGL_VL53L1X_ID, now_ts, range_ofs_m);

#ifdef SENSOR_SYNC_SEND_SONAR
  // Send Telemetry report
  DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &range_mm, &range_ofs_m);
#endif
}


void sonar_vl53l1x_init(void)
{
  // Set up structs
  sonar_vl53l1x.dev.i2c_p = &SONAR_VL53L1X_I2C_DEV;
  sonar_vl53l1x.dev.i2c_trans.slave_addr = SONAR_VL53L1X_I2C_ADDR;
  sonar_vl53l1x.offset_mm = SONAR_VL53L1X_OFFSET;

  /* Initialize sensor */
#ifndef SITL
  uint8_t state;
  do {
    VL53L1X_BootState(&sonar_vl53l1x.dev, &state);
  } while (!state);
  VL53L1X_SensorInit(&sonar_vl53l1x.dev);
  /* Configure sensor */
  VL53L1X_SetTimingBudgetInMs(&sonar_vl53l1x.dev, SONAR_VL53L1X_TIMINGBUDGET_MS);
  VL53L1X_SetDistanceMode(&sonar_vl53l1x.dev, SONAR_VL53L1X_DISTANCEMODE);
  VL53L1X_SetInterMeasurementInMs(&sonar_vl53l1x.dev, SONAR_VL53L1X_INTERMEASUREMENT_MS);
  /* Start measurement */
  VL53L1X_StartRanging(&sonar_vl53l1x.dev);
#endif
}


void sonar_vl53l1x_read(void)
{
#ifndef SITL
  uint8_t isDataReady;
  uint16_t range_mm;
  switch (sonar_vl53l1x.read_state) {
    case 0:
      // Wait for data ready
      if (!VL53L1X_NonBlocking_CheckForDataReady(&sonar_vl53l1x.dev, &isDataReady)) { return; } // Check in progress
      sonar_vl53l1x.read_state++;
    /* Falls through. */
    case 1:
      // Get ranging data
      if (!VL53L1X_NonBlocking_GetDistance(&sonar_vl53l1x.dev, &range_mm)) { return; } // Read in progress
      sonar_vl53l1x_publish(range_mm);
      sonar_vl53l1x.read_state++;
    /* Falls through. */
    case 2:
      // Clear interrupt
      if (!VL53L1X_NonBlocking_ClearInterrupt(&sonar_vl53l1x.dev)) { return; } // Clear in progress
      sonar_vl53l1x.read_state = 0;
      break;
    default: return;
  }
#else // SITL
  float range_mm = stateGetPositionEnu_f()->z * 1.0e3f;
  if (range_mm > 0.0f && range_mm < 5.0f) {
    sonar_vl53l1x_publish(range_mm)
  }
#endif // SITL
}
