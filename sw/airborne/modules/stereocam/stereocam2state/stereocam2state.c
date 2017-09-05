/*
 * Copyright (C) Kimberly McGuire
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/stereocam/stereocam2state/stereocam2state.c"
 * @author Kimberly McGuire
 * This module sends the data retreived from an external stereocamera modules, to the state filter of the drone. This is done so that the guidance modules can use that information for couadcopter
 */

#include "modules/stereocam/stereocam2state/stereocam2state.h"

#include "subsystems/abi.h"
#include "subsystems/datalink/telemetry.h"

#ifndef STEREOCAM2STATE_SENDER_ID
#define STEREOCAM2STATE_SENDER_ID ABI_BROADCAST
#endif

#ifndef STEREOCAM2STATE_RECEIVED_DATA_TYPE
#define STEREOCAM2STATE_RECEIVED_DATA_TYPE 0
#endif
PRINT_CONFIG_VAR(STEREOCAM2STATE_RECEIVED_DATA_TYPE)

#if STEREOCAM2STATE_RECEIVED_DATA_TYPE == 0
#ifndef STEREOCAM2STATE_EDGEFLOW_PIXELWISE
#define STEREOCAM2STATE_EDGEFLOW_PIXELWISE FALSE
PRINT_CONFIG_VAR(STEREOCAM2STATE_EDGEFLOW_PIXELWISE)
#endif
uint8_t stereocam_medianfilter_on = 1;
#endif

#include "filters/median_filter.h"
struct MedianFilter3Float velFilter;

#include "subsystems/datalink/telemetry.h"

void stereocam_to_state(void);

void stereo_to_state_init(void)
{
  InitMedianFilterVect3Float(velFilter, MEDIAN_DEFAULT_SIZE);
}

void stereo_to_state_periodic(void)
{
  if (stereocam_data.fresh) {
    stereocam_to_state();
    stereocam_data.fresh = 0;
  }
}

void stereocam_to_state(void)
{
  int16_t RES = 100;

  // Sort the info from stereocam data from UART

  // 0 = stereoboard's #define SEND_EDGEFLOW
#if STEREOCAM2STATE_RECEIVED_DATA_TYPE == 0
  // opticflow and divergence (unscaled with depth)
  int16_t div_x = (int16_t)stereocam_data.data[0] << 8;
  div_x |= (int16_t)stereocam_data.data[1];
  int16_t flow_x = (int16_t)stereocam_data.data[2] << 8;
  flow_x |= (int16_t)stereocam_data.data[3];
  int16_t div_y = (int16_t)stereocam_data.data[4] << 8;
  div_y |= (int16_t)stereocam_data.data[5];
  int16_t flow_y = (int16_t)stereocam_data.data[6] << 8;
  flow_y |= (int16_t)stereocam_data.data[7];

// uint8_t agl = stereocam_data.data[8]; // in cm //TODO: use agl for in a guided obstacle avoidance.
  float fps = (float)stereocam_data.data[9];

  // velocity global
  int16_t vel_x_global_int = (int16_t)stereocam_data.data[10] << 8;
  vel_x_global_int |= (int16_t)stereocam_data.data[11];
  int16_t vel_y_global_int = (int16_t)stereocam_data.data[12] << 8;
  vel_y_global_int |= (int16_t)stereocam_data.data[13];
  int16_t vel_z_global_int = (int16_t)stereocam_data.data[14] << 8;
  vel_z_global_int |= (int16_t)stereocam_data.data[15];

  // Velocity Pixelwise
  int16_t vel_x_pixelwise_int = (int16_t)stereocam_data.data[16] << 8;
  vel_x_pixelwise_int |= (int16_t)stereocam_data.data[17];
  int16_t vel_z_pixelwise_int = (int16_t)stereocam_data.data[18] << 8;
  vel_z_pixelwise_int |= (int16_t)stereocam_data.data[19];

// Select what type of velocity estimate fom edgeflow is wanted
#if STEREOCAM2STATE_EDGEFLOW_PIXELWISE == TRUE
  struct FloatVect3 camera_frame_vel;
  camera_frame_vel.x = (float)vel_x_pixelwise_int / RES;
  camera_frame_vel.y = (float)vel_y_global_int / RES;
  camera_frame_vel.z = (float)vel_z_pixelwise_int / RES;

#else
  struct FloatVect3 camera_frame_vel;
  camera_frame_vel.x = (float)vel_x_global_int / RES;
  camera_frame_vel.y = (float)vel_y_global_int / RES;
  camera_frame_vel.z = (float)vel_z_global_int / RES;

#endif

//Rotate velocity back to quad's frame
  struct FloatVect3 quad_body_vel;
  float_rmat_transp_vmult(&quad_body_vel, &body_to_stereocam, &camera_frame_vel);

  //Send velocity estimate to state
  //TODO:: Make variance dependable on line fit error, after new horizontal filter is made
  uint32_t now_ts = get_sys_time_usec();

  float vel_body_x_processed = quad_body_vel.x;
  float vel_body_y_processed = quad_body_vel.y;
  float vel_body_z_processed = quad_body_vel.z;

  if (stereocam_medianfilter_on == 1) {
    // Use a slight median filter to filter out the large outliers before sending it to state
    // TODO: if a float median filter exist, replace this version
    UpdateMedianFilterVect3Float(velFilter, quad_body_vel);
  }

  //Send velocities to state
  AbiSendMsgVELOCITY_ESTIMATE(STEREOCAM2STATE_SENDER_ID, now_ts,
                              vel_body_x_processed,
                              vel_body_y_processed,
                              vel_body_z_processed,
                              0.3f
                             );

  // Reusing the OPTIC_FLOW_EST telemetry messages, with some values replaced by 0
  uint16_t dummy_uint16 = 0;
  int16_t dummy_int16 = 0;
  float dummy_float = 0;

  DOWNLINK_SEND_OPTIC_FLOW_EST(DefaultChannel, DefaultDevice, &fps, &dummy_uint16, &dummy_uint16, &flow_x, &flow_y,
                               &dummy_int16, &dummy_int16, &vel_body_x_processed, &vel_body_y_processed,
                               &dummy_float, &dummy_float, &dummy_float);

#endif

}
