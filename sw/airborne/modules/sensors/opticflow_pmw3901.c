/*
 * Copyright (C) Tom van Dijk
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
 * @file "modules/sensors/opticflow_pmw3901.c"
 * @author Tom van Dijk
 * Driver for PMW3901 optical flow sensor
 */

#include "modules/sensors/opticflow_pmw3901.h"

#include "peripherals/pmw3901.h"

#include "subsystems/abi.h"
#include "subsystems/datalink/downlink.h"


struct pmw3901_t pmw;


static void opticflow_pmw3901_publish(int16_t delta_x, int16_t delta_y, uint32_t ts) {
  AbiSendMsgOPTICAL_FLOW(FLOW_OPTICFLOW_PMW3901_ID,
      ts,       /* stamp [us] */
      delta_x,  /* flow_x [subpixels] */ // TODO unit
      delta_y,  /* flow_y [subpixels] */
      0,        /* flow_der_x [subpixels] */ // TODO later
      0,        /* flow_der_y [subpixels] */
      0.f,      /* quality [???]*/
      0.f       /* size_divergence [1/s] */
      );
  // TODO conditions
  AbiSendMsgVELOCITY_ESTIMATE(VEL_OPTICFLOW_PMW3901_ID,
      ts,       /* stamp [us] */
      0.f,      /* x [m/s] */
      0.f,      /* y [m/s] */
      0.f,      /* z [m/s] */
      0.2f,     /* noise_x [m/s] */
      0.2f,     /* noise_y [m/s] */
      -1.f      /* noise_z [disabled] */
      );
#if SENSOR_SYNC_SEND_OPTICFLOW_PMW3901
  float dummy_f = 0.f;
  uint16_t dummy_u16 = 0;
  int16_t dummy_i16 = 0;
  DOWNLINK_SEND_OPTIC_FLOW_EST(DefaultChannel, DefaultDevice,
      &dummy_f,     /* fps */
      &dummy_u16,   /* corner_cnt */
      &dummy_u16,   /* tracked_cnt */
      &delta_x,     /* flow_x */      /* UNITS?? */
      &delta_y,     /* flow_y */      /* UNITS?? */
      &dummy_i16,   /* flow_der_x */  /* TODO */
      &dummy_i16,   /* flow_der_y */  /* TODO */
      &dummy_f,     /* vel_x */       /* TODO */
      &dummy_f,     /* vel_y */       /* TODO */
      &dummy_f,     /* vel_z */       /* TODO */
      &dummy_f,     /* div_size */
      &dummy_f,     /* surface_roughness */
      &dummy_f      /* divergence */
      );
#endif
}


void opticflow_pmw3901_init(void) {
  pmw3901_init(&pmw, &OPTICFLOW_PMW3901_SPI_DEV, OPTICFLOW_PMW3901_SPI_SLAVE_IDX);
}

void opticflow_pmw3901_periodic(void) {
  if (pmw3901_is_idle(&pmw)) {
    pmw3901_start_read(&pmw);
  }
}

void opticflow_pmw3901_event(void) {
  pmw3901_event(&pmw);
  if (pmw3901_data_available(&pmw)) {
    int16_t delta_x, delta_y;
    pmw3901_get_data(&pmw, &delta_x, &delta_y);
    opticflow_pmw3901_publish(delta_x, delta_y, get_sys_time_usec());
  }
}


