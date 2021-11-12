/*
 * Copyright (C) 2016 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
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

/**
 * @file modules/imu/imu_vectornav.h
 *
 * Vectornav VN-200 IMU module, to be used with other AHRS/INS algorithms.
 */

#ifndef IMU_VECTORNAV_H
#define IMU_VECTORNAV_H

// Subsystem
#include "modules/imu/imu.h"

// Peripheral
#include "peripherals/vn200_serial.h"


struct ImuVectornav {
  // Packet data
  struct VNPacket vn_packet;///< Packet struct
  struct VNData vn_data; ///< Data struct
  enum VNStatus vn_status;  ///< VN status
  float vn_freq;            ///< data frequency
};


extern struct ImuVectornav imu_vn;

void imu_vectornav_init(void);
void imu_vectornav_event(void);
void imu_vectornav_periodic(void);
void imu_vectornav_propagate(void);

/* no scaling (the WEAK attribute has no effect */
#ifndef IMU_GYRO_P_SENS_NUM
#define IMU_GYRO_P_SENS_NUM 1
#endif

#ifndef IMU_GYRO_P_SENS_DEN
#define IMU_GYRO_P_SENS_DEN 1
#endif

#ifndef IMU_GYRO_Q_SENS_NUM
#define IMU_GYRO_Q_SENS_NUM 1
#endif

#ifndef IMU_GYRO_Q_SENS_DEN
#define IMU_GYRO_Q_SENS_DEN 1
#endif

#ifndef IMU_GYRO_R_SENS_NUM
#define IMU_GYRO_R_SENS_NUM 1
#endif

#ifndef IMU_GYRO_R_SENS_DEN
#define IMU_GYRO_R_SENS_DEN 1
#endif

#endif /* IMU_VECTORNAV_H */
