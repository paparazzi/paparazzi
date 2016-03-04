/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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

/**
 * @file subsystems/abi_sender_ids.h
 *
 * Convenience defines for ABI sender IDs.
 */

#ifndef ABI_SENDER_IDS_H
#define ABI_SENDER_IDS_H

/** default onboard baro */
#ifndef BARO_BOARD_SENDER_ID
#define BARO_BOARD_SENDER_ID 1
#endif

/*
 * IDs of baro modules that can be loaded
 */
#ifndef BARO_MS5611_SENDER_ID
#define BARO_MS5611_SENDER_ID 10
#endif

#ifndef BARO_AMSYS_SENDER_ID
#define BARO_AMSYS_SENDER_ID 11
#endif

#ifndef BARO_BMP_SENDER_ID
#define BARO_BMP_SENDER_ID 12
#endif

#ifndef BARO_ETS_SENDER_ID
#define BARO_ETS_SENDER_ID 13
#endif

#ifndef BARO_MS5534A_SENDER_ID
#define BARO_MS5534A_SENDER_ID 14
#endif

#ifndef BARO_HCA_SENDER_ID
#define BARO_HCA_SENDER_ID 15
#endif

#ifndef BARO_MPL3115_SENDER_ID
#define BARO_MPL3115_SENDER_ID 16
#endif

#ifndef BARO_SCP_SENDER_ID
#define BARO_SCP_SENDER_ID 17
#endif

#ifndef BARO_PBN_SENDER_ID
#define BARO_PBN_SENDER_ID 18
#endif

#ifndef BARO_SIM_SENDER_ID
#define BARO_SIM_SENDER_ID 19
#endif

#ifndef METEO_STICK_SENDER_ID
#define METEO_STICK_SENDER_ID 30
#endif

/*
 * IDs of differential pressure sensors
 * can usually also publish temperature like baro sensors
 */
#ifndef MS45XX_SENDER_ID
#define MS45XX_SENDER_ID 40
#endif

/*
 * IDs of AGL measurment modules that can be loaded (sonars,...)
 */
#ifndef AGL_SONAR_ADC_ID
#define AGL_SONAR_ADC_ID 1
#endif

#ifndef AGL_SONAR_ARDRONE2_ID
#define AGL_SONAR_ARDRONE2_ID 2
#endif

#ifndef AGL_SONAR_NPS_ID
#define AGL_SONAR_NPS_ID 3
#endif

/*
 * IDs of magnetometer sensors (including IMUs with mag)
 */

#ifndef MAG_HMC58XX_SENDER_ID
#define MAG_HMC58XX_SENDER_ID 2
#endif

/*
 * IDs of GPS sensors
 */
#ifndef GPS_UBX_ID
#define GPS_UBX_ID 1
#endif

#ifndef GPS_NMEA_ID
#define GPS_NMEA_ID 2
#endif

#ifndef GPS_SIRF_ID
#define GPS_SIRF_ID 3
#endif

#ifndef GPS_SKYTRAQ_ID
#define GPS_SKYTRAQ_ID 4
#endif

#ifndef GPS_MTK_ID
#define GPS_MTK_ID 5
#endif

#ifndef GPS_PIKSI_ID
#define GPS_PIKSI_ID 6
#endif

#ifndef GPS_XSENS_ID
#define GPS_XSENS_ID 7
#endif

#ifndef GPS_DATALINK_ID
#define GPS_DATALINK_ID 8
#endif

#ifndef GPS_UDP_ID
#define GPS_UDP_ID 9
#endif

#ifndef GPS_ARDRONE2_ID
#define GPS_ARDRONE2_ID 10
#endif

#ifndef GPS_SIM_ID
#define GPS_SIM_ID 11
#endif

#ifndef GPS_MULTI_ID
#define GPS_MULTI_ID 12
#endif

#ifndef GPS_VECTORNAV_ID
#define GPS_VECTORNAV_ID 13
#endif

/*
 * IDs of IMU sensors (accel, gyro)
 */
#ifndef IMU_BOARD_ID
#define IMU_BOARD_ID 1
#endif

#ifndef IMU_ANALOG_ID
#define IMU_ANALOG_ID 2
#endif

#ifndef IMU_ASPIRIN_ID
#define IMU_ASPIRIN_ID 3
#endif

#ifndef IMU_ASPIRIN2_ID
#define IMU_ASPIRIN2_ID 4
#endif

#ifndef IMU_B2_ID
#define IMU_B2_ID 5
#endif

#ifndef IMU_CRISTA_ID
#define IMU_CRISTA_ID 6
#endif

#ifndef IMU_DROTEK_ID
#define IMU_DROTEK_ID 7
#endif

#ifndef IMU_GL1_ID
#define IMU_GL1_ID 8
#endif

#ifndef IMU_MPU6000_ID
#define IMU_MPU6000_ID 9
#endif

#ifndef IMU_MPU6000_HMC_ID
#define IMU_MPU6000_HMC_ID 10
#endif

#ifndef IMU_MPU9250_ID
#define IMU_MPU9250_ID 11
#endif

#ifndef IMU_PPZUAV_ID
#define IMU_PPZUAV_ID 12
#endif

#ifndef IMU_UM6_ID
#define IMU_UM6_ID 13
#endif

#ifndef IMU_GX3_ID
#define IMU_GX3_ID 14
#endif

#ifndef IMU_XSENS_ID
#define IMU_XSENS_ID 15
#endif

#ifndef IMU_MPU60X0_ID
#define IMU_MPU60X0_ID 16
#endif

#ifndef PX4FLOW_VELOCITY_ID
#define PX4FLOW_VELOCITY_ID 17
#endif

#ifndef IMU_PX4
#define IMU_PX4_ID 18
#endif

/*
 * IDs of RSSI measurements (message 13)
 */
#ifndef RSSI_BLUEGIGA_ID
#define RSSI_BLUEGIGA_ID 1
#endif


#endif /* ABI_SENDER_IDS_H */
