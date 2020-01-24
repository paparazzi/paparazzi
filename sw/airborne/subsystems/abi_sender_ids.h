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
 * IDs of baro modules that can be loaded (message 0)
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

#ifndef BARO_SIM_SENDER_ID
#define BARO_SIM_SENDER_ID 19
#endif

#ifndef BARO_BMP3_SENDER_ID
#define BARO_BMP3_SENDER_ID 20
#endif

#ifndef METEO_STICK_SENDER_ID
#define METEO_STICK_SENDER_ID 30
#endif

/*
 * IDs of differential pressure sensors (message 1)
 * can usually also publish temperature like baro sensors
 */
#ifndef MS45XX_SENDER_ID
#define MS45XX_SENDER_ID 40
#endif

#ifndef SDP3X_SENDER_ID
#define SDP3X_SENDER_ID 41
#endif

/*
 * IDs of airspeed sensors (message 14)
 */
#ifndef AIRSPEED_NPS_ID
#define AIRSPEED_NPS_ID 1
#endif

#ifndef AIRSPEED_ADC_ID
#define AIRSPEED_ADC_ID 2
#endif

#ifndef AIRSPEED_SDP3X_ID
#define AIRSPEED_SDP3X_ID 3
#endif

/*
 * IDs of Incidence angles (message 24)
 */
#ifndef AOA_ADC_ID
#define AOA_ADC_ID 1
#endif

#ifndef AOA_PWM_ID
#define AOA_PWM_ID 2
#endif

#ifndef INCIDENCE_NPS_ID
#define INCIDENCE_NPS_ID 20
#endif

/*
 * IDs of AGL measurment modules that can be loaded (sonars,...) (message 2)
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

#ifndef AGL_SONAR_PX4FLOW_ID
#define AGL_SONAR_PX4FLOW_ID 4
#endif

#ifndef AGL_TERARANGER_ONE_ID
#define AGL_TERARANGER_ONE_ID 5
#endif

#ifndef AGL_LIDAR_LITE_ID
#define AGL_LIDAR_LITE_ID 6
#endif

#ifndef AGL_PX4FLOW_ID
#define AGL_PX4FLOW_ID 7
#endif

#ifndef AGL_LIDAR_SF11_ID
#define AGL_LIDAR_SF11_ID 8
#endif

#ifndef AGL_VL53L0_LASER_ARRAY_ID
#define AGL_VL53L0_LASER_ARRAY_ID 9
#endif

#ifndef AGL_RAY_SENSOR_GAZEBO_ID
#define AGL_RAY_SENSOR_GAZEBO_ID 10
#endif

#ifndef AGL_LIDAR_TFMINI_ID
#define AGL_LIDAR_TFMINI_ID 11
#endif

/*
 * IDs of magnetometer sensors (including IMUs with mag)
 */

#ifndef MAG_HMC58XX_SENDER_ID
#define MAG_HMC58XX_SENDER_ID 2
#endif

#ifndef MAG_LIS3MDL_SENDER_ID
#define MAG_LIS3MDL_SENDER_ID 3
#endif

#ifndef IMU_MAG_PITOT_ID
#define IMU_MAG_PITOT_ID 50
#endif

/*
 * IDs of GPS sensors (message 10)
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

#ifndef GPS_IMCU_ID
#define GPS_IMCU_ID 14
#endif

#ifndef GPS_DW1000_ID
#define GPS_DW1000_ID 15
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

#ifndef IMU_PX4_ID
#define IMU_PX4_ID 17
#endif

#ifndef IMU_VECTORNAV_ID
#define IMU_VECTORNAV_ID 18
#endif

#ifndef IMU_BMI088_ID
#define IMU_BMI088_ID 19
#endif

// prefiltering with OneEuro filter
#ifndef IMU_F1E_ID
#define IMU_F1E_ID 30
#endif

/*
 * IDs of OPTICFLOW estimates (message 11)
 */
#ifndef FLOW_OPTICFLOW_ID
#define FLOW_OPTICFLOW_ID 1
#endif

/*
 * IDs of VELOCITY estimates (message 12)
 */
#ifndef VEL_DRAGSPEED_ID
#define VEL_DRAGSPEED_ID 1
#endif

#ifndef VEL_PX4FLOW_ID
#define VEL_PX4FLOW_ID 2
#endif

#ifndef VEL_OPTICFLOW_ID
#define VEL_OPTICFLOW_ID 3
#endif

#ifndef VEL_STEREOCAM_ID
#define VEL_STEREOCAM_ID 4
#endif

/*
 * IDs of RSSI measurements (message 13)
 */
#ifndef RSSI_BLUEGIGA_ID
#define RSSI_BLUEGIGA_ID 1
#endif

/*
 * IDs of RPM sensors (message 15)
 */
#ifndef RPM_SENSOR_ID
#define RPM_SENSOR_ID 1
#endif

/*
 * IDs of THRUST increment calculation (message 16)
 */
#ifndef THRUST_INCREMENT_ID
#define THRUST_INCREMENT_ID 1
#endif

#ifndef MAG_CALIB_UKF_ID
#define MAG_CALIB_UKF_ID 20
#endif

/*
 * UWB communication (message 19)
*/
#ifndef UWB_COMM_ID
#define UWB_COMM_ID 1
#endif

/*
 * IDs of Obstacle detection systems
 */

#ifndef OBS_DETECTION_COLOR_ID
#define OBS_DETECTION_COLOR_ID 1
#endif

#ifndef OBS_DETECTION_RANGE_ARRAY_ID
#define OBS_DETECTION_RANGE_ARRAY_ID 2
#endif

#ifndef OBS_DETECTION_RANGE_ARRAY_NPS_ID
#define OBS_DETECTION_RANGE_ARRAY_NPS_ID 3
#endif

/*
 * ID's of forcefield generating type functions
 */

#ifndef RANGE_FORCEFIELD_ID
#define RANGE_FORCEFIELD_ID 1
#endif

/*
 * ID's for camera type sensors
 */

#ifndef CAM_JEVOIS_ID
#define CAM_JEVOIS_ID 1
#endif

/*
 * IDs of ACCEL_SP senders (message 21)
 */

#ifndef ACCEL_SP_FCR_ID
#define ACCEL_SP_FCR_ID 1 // Formation Control Rotorcraft
#endif

/*
 * RELATIVE_LOCALIZATION communication (message 24)
*/
#ifndef RELATIVE_LOCALIZATION_ID
#define RELATIVE_LOCALIZATION_ID 1
#endif

#ifndef DETECT_GATE_ABI_ID
#define DETECT_GATE_ABI_ID 33
#endif

/*
 * VISUAL_DETECTION communication (message 27)
*/
#ifndef COLOR_OBJECT_DETECTION1_ID
#define COLOR_OBJECT_DETECTION1_ID 1
#endif

#ifndef COLOR_OBJECT_DETECTION2_ID
#define COLOR_OBJECT_DETECTION2_ID 2
#endif

/*
 * JOYSTICK message (used for payload or control, but not as a RC)
 */
#ifndef JOYSTICK_ID
#define JOYSTICK_ID 1
#endif

#endif /* ABI_SENDER_IDS_H */
