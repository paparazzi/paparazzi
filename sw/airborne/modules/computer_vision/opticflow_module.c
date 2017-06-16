/*
 * Copyright (C) 2014 Hann Woei Ho
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow_module.c
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */


#include "opticflow_module.h"

#include <stdio.h>
#include <pthread.h>
#include "state.h"
#include "subsystems/abi.h"

#include "lib/v4l/v4l2.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "errno.h"

#include "cv.h"

/* ABI messages sender ID */
#ifndef OPTICFLOW_AGL_ID
#define OPTICFLOW_AGL_ID ABI_BROADCAST    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(OPTICFLOW_AGL_ID)

#ifndef OPTICFLOW_IMU_ID
#define OPTICFLOW_IMU_ID ABI_BROADCAST    ///< Default IMU (accelerometers) to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(OPTICFLOW_IMU_ID)

#ifndef OPTICFLOW_BODY_TO_IMU_ID
#define OPTICFLOW_BODY_TO_IMU_ID ABI_BROADCAST    ///< Default body to IMU to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(OPTICFLOW_BODY_TO_IMU_ID)

#ifndef OPTICFLOW_SEND_ABI_ID
#define OPTICFLOW_SEND_ABI_ID 1       ///< Default ID to send abi messages
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SEND_ABI_ID)


/* The main opticflow variables */
struct opticflow_t opticflow;                      ///< Opticflow calculations
static struct opticflow_result_t opticflow_result; ///< The opticflow result
static struct opticflow_state_t opticflow_state;   ///< State of the drone to communicate with the opticflow
static abi_event opticflow_imu_accel_ev;                 ///< The altitude ABI event
static abi_event opticflow_agl_ev;                 ///< The accelerometers ABI event
static abi_event opticflow_body_to_imu_ev;        ///< The body-to-imu ABI event

static bool opticflow_got_result;                ///< When we have an optical flow calculation
static pthread_mutex_t opticflow_mutex;            ///< Mutex lock fo thread safety

/* Static functions */
struct image_t *opticflow_module_calc(struct image_t *img);     ///< The main optical flow calculation thread
static void opticflow_agl_cb(uint8_t sender_id, float distance);    ///< Callback function of the ground altitude
static void opticflow_imu_accel_cb(uint8_t sender_id, uint32_t stamp,
                                   struct Int32Vect3 *accel); ///< Callback function of the IMU's accelerometers
static void opticflow_body_to_imu_cb(uint8_t sender_id,
                                     struct FloatQuat *q_b2i_f); ///< Callback function of imu to body

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
/**
 * Send optical flow telemetry information
 * @param[in] *trans The transport structure to send the information over
 * @param[in] *dev The link to send the data over
 */
static void opticflow_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pthread_mutex_lock(&opticflow_mutex);
  if (opticflow_result.noise_measurement < 0.8) {
    pprz_msg_send_OPTIC_FLOW_EST(trans, dev, AC_ID,
                                 &opticflow_result.fps, &opticflow_result.corner_cnt,
                                 &opticflow_result.tracked_cnt, &opticflow_result.flow_x,
                                 &opticflow_result.flow_y, &opticflow_result.flow_der_x,
                                 &opticflow_result.flow_der_y, &opticflow_result.vel_x,
                                 &opticflow_result.vel_y, &opticflow_result.div_size,
                                 &opticflow_result.surface_roughness, &opticflow_result.divergence); // TODO: no noise measurement here...
  }
  pthread_mutex_unlock(&opticflow_mutex);
}
#endif

/**
 * Initialize the optical flow module for the bottom camera
 */
void opticflow_module_init(void)
{
  // Subscribe ABI messages
  AbiBindMsgAGL(OPTICFLOW_AGL_ID, &opticflow_agl_ev, opticflow_agl_cb); // ABI to the altitude above ground level
  AbiBindMsgIMU_ACCEL_INT32(OPTICFLOW_IMU_ID, &opticflow_imu_accel_ev,
                            &opticflow_imu_accel_cb); // ABI to the IMU accelerometer measurements
  AbiBindMsgBODY_TO_IMU_QUAT(OPTICFLOW_BODY_TO_IMU_ID, &opticflow_body_to_imu_ev,
                             &opticflow_body_to_imu_cb); // ABI to the quaternion of body to imu

  // Set the opticflow state to 0
  FLOAT_RATES_ZERO(opticflow_state.rates);
  float_quat_identity(&opticflow_state.imu_to_body_quat);
  INT_VECT3_ZERO(opticflow_state.accel_imu_meas);
  opticflow_state.agl = 0;

  // Initialize the opticflow calculation
  opticflow_got_result = false;
  opticflow_calc_init(&opticflow);

  cv_add_to_device(&OPTICFLOW_CAMERA, opticflow_module_calc);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTIC_FLOW_EST, opticflow_telem_send);
#endif

}

/**
 * Update the optical flow state for the calculation thread
 * and update the stabilization loops with the newest result
 */
void opticflow_module_run(void)
{
  pthread_mutex_lock(&opticflow_mutex);
  // Update the stabilization loops on the current calculation
  if (opticflow_got_result) {
    uint32_t now_ts = get_sys_time_usec();
    AbiSendMsgOPTICAL_FLOW(OPTICFLOW_SEND_ABI_ID, now_ts,
                           opticflow_result.flow_x,
                           opticflow_result.flow_y,
                           opticflow_result.flow_der_x,
                           opticflow_result.flow_der_y,
                           opticflow_result.noise_measurement,
                           opticflow_result.div_size,
                           opticflow_state.agl);
    //TODO Find an appropiate quality measure for the noise model in the state filter, for now it is tracked_cnt
    if (opticflow_result.noise_measurement < 0.8) {
      AbiSendMsgVELOCITY_ESTIMATE(OPTICFLOW_SEND_ABI_ID, now_ts,
                                  opticflow_result.vel_body_x,
                                  opticflow_result.vel_body_y,
                                  0.0f,
                                  opticflow_result.noise_measurement
                                 );
    }
    opticflow_got_result = false;
  }
  pthread_mutex_unlock(&opticflow_mutex);
}

/**
 * The main optical flow calculation thread
 * This thread passes the images trough the optical flow
 * calculator
 * @param[in] *img The image_t structure of the captured image
 * @return *img The processed image structure
 */
struct image_t *opticflow_module_calc(struct image_t *img)
{
  // Copy the state
  // TODO : put accelerometer values at pose of img timestamp
  struct opticflow_state_t temp_state;
  struct pose_t pose = get_rotation_at_timestamp(img->pprz_ts);
  temp_state = opticflow_state;
  temp_state.rates = pose.rates;

  // Do the optical flow calculation
  static struct opticflow_result_t temp_result = {}; // static so that the number of corners is kept between frames
  opticflow_calc_frame(&opticflow, &temp_state, img, &temp_result);

  // Copy the result if finished
  pthread_mutex_lock(&opticflow_mutex);
  opticflow_result = temp_result;
  opticflow_got_result = true;


  // release the mutex as we are done with editing the opticflow result
  pthread_mutex_unlock(&opticflow_mutex);
  return img;
}

/**
 * Get the altitude above ground of the drone
 * @param[in] sender_id The id that send the ABI message (unused)
 * @param[in] distance The distance above ground level in meters
 */
static void opticflow_agl_cb(uint8_t sender_id __attribute__((unused)), float distance)
{
  // Update the distance if we got a valid measurement
  if (distance > 0) {
    opticflow_state.agl = distance;
  }
}

/**
 * Get the accelerometer measurements of the imu
 * @param[in] sender_id The id that send the ABI message (unused)
 * @param[in] stamp  The timestamp of when the message is send
 * @param[in] accel  The accelerometer measurements of the imu
 */
static void opticflow_imu_accel_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, struct Int32Vect3 *accel)
{
  opticflow_state.accel_imu_meas = *accel;
}

/**
 * Get the body-to-imu quaternion
 * @param[in] sender_id The id that send the ABI message (unused)
 * @param[in] q_b2i_f  The body-to-imu quaternion
 */
static void opticflow_body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                                     struct FloatQuat *q_b2i_f)
{
  struct FloatQuat imu_to_body_quat_temp;
  float_quat_invert(&imu_to_body_quat_temp, q_b2i_f); // invert quaternion for body-to-imu to imu-to-body
  opticflow_state.imu_to_body_quat = imu_to_body_quat_temp;
}
