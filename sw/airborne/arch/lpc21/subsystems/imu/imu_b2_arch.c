/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 Gautier Hattenberger
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

#include "subsystems/imu.h"


#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100

static void SSP_ISR(void) {
  ISR_ENTRY();

  switch (imu_ssp_status) {
  case IMU_SSP_STA_BUSY_MAX1168:
    Max1168OnSpiInt();
    if (ms2100_status == MS2100_IDLE || ms2100_status == MS2100_GOT_EOC) {
      ImuSetSSP8bits();
      if (ms2100_status == MS2100_IDLE) {
        Ms2100SendReq();
      }
      else { /* MS2100_GOT_EOC */
        Ms2100ReadRes();
      }
      imu_ssp_status = IMU_SSP_STA_BUSY_MS2100;
    }
    else {
      imu_ssp_status = IMU_SSP_STA_IDLE;
    }
    break;
  case IMU_SSP_STA_BUSY_MS2100:
    Ms2100OnSpiInt();
    if (ms2100_status == MS2100_IDLE) {
      Ms2100SendReq();
      imu_ssp_status = IMU_SSP_STA_BUSY_MS2100;
    }
    else
      imu_ssp_status = IMU_SSP_STA_IDLE;
    break;

   // default:
   // spurious interrupt
   // FIXME LED_ON(1);
  }

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

#else //no IMU_B2_MAG_MS2100

static void SSP_ISR(void) {
  ISR_ENTRY();

  switch (imu_ssp_status) {
  case IMU_SSP_STA_BUSY_MAX1168:
    Max1168OnSpiInt();
    imu_ssp_status = IMU_SSP_STA_IDLE;
    break;

    // default:
    // spurious interrupt
    // FIXME LED_ON(1);
  }

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

#endif //no IMU_B2_MAG_MS2100
