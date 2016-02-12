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

//#include "subsystems/gps.h"

#ifndef SENDER_ID
#define SENDER_ID 1
#endif

/** ABI binding for gps messages*/
#ifndef STEREOCAM_GPS_ID
#define STEREOCAM_GPS_ID GPS_MULTI_ID
#endif
static abi_event gps_ev;


/** For extra functionality for derotation of velocity to state measurements*/
#ifndef USE_DEROTATION_OPTICFLOW
#define USE_DEROTATION_OPTICFLOW FALSE
#endif
#ifndef STATE_MEASURE_OPTICFLOW
#define STATE_MEASURE_OPTICFLOW TRUE
#endif

static float prev_phi;
static float prev_theta;

struct GpsStereoCam gps_stereocam;

void stereocam_to_state(float dphi, float dtheta);

static void stereocam_gps_cb(uint8_t sender_id __attribute__((unused)),
                             uint32_t stamp __attribute__((unused)),
                             struct GpsState *gps_s)
{
  gps_stereocam.ecef_vel.x = gps_s->ecef_vel.x;
  gps_stereocam.ecef_vel.y = gps_s->ecef_vel.y;
  gps_stereocam.ecef_vel.z = gps_s->ecef_vel.y;

}

void stereo_to_state_init(void)
{
  //subscribe to GPS abi-messages for state measurements
  AbiBindMsgGPS(STEREOCAM_GPS_ID, &gps_ev, stereocam_gps_cb);

}
void stereo_to_state_periodic(void)
{

  if (stereocam_data.fresh) {
    stereocam_data.fresh = 0;
    float phi = stateGetNedToBodyEulers_f()->phi;
    float theta = stateGetNedToBodyEulers_f()->theta;
    float dphi =  phi - prev_phi;
    float dtheta = theta - prev_theta;

    stereocam_to_state(dphi, dtheta);

    prev_theta = theta;
    prev_phi = phi;
  }
}

void stereocam_to_state(float dphi, float dtheta)
{

  // Get info from stereocam data
  float vel_hor = ((float)(stereocam_data.data[8]) - 127) / 100;
  float vel_ver = ((float)(stereocam_data.data[9]) - 127) / 100;
  float vel_x = 0;
  float vel_y = 0;


  // Calculate derotated velocity
#if USE_DEROTATION_OPTICFLOW
  float agl_stereo = (float)(stereocam_data.data[4]) / 10;


  float diff_flow_hor = dtheta * 128 / 1.04;
  float diff_flow_ver = dphi * 96 / 0.785;

  float diff_vel_hor = diff_flow_hor * agl_stereo * 12 * 1.04 / 128;
  float diff_vel_ver = diff_flow_ver * agl_stereo * 12 * 0.785 / 96;

  vel_x = - (vel_ver - diff_vel_ver);
  vel_y = (vel_hor - diff_vel_hor);
#endif

  // Derotate velocity and transform from frame to body coordinates
  vel_x = - (vel_ver);
  vel_y = (vel_hor);


#if STATE_MEASURE_OPTICFLOW
  // Calculate velocity in body fixed coordinates from opti-track and the state filter
  struct NedCoor_f coordinates_speed_state;
  coordinates_speed_state.x = stateGetSpeedNed_f()->x;
  coordinates_speed_state.y = stateGetSpeedNed_f()->y;
  coordinates_speed_state.z = stateGetSpeedNed_f()->z;

  struct NedCoor_f opti_state;
  opti_state.x = (float)(gps_stereocam.ecef_vel.y) / 100;
  opti_state.y = (float)(gps_stereocam.ecef_vel.x) / 100;
  opti_state.z = -(float)(gps_stereocam.ecef_vel.z) / 100;

  struct FloatVect3 velocity_rot_state;
  struct FloatVect3 velocity_rot_gps;

  float_rmat_vmult(&velocity_rot_state , stateGetNedToBodyRMat_f(), (struct FloatVect3 *)&coordinates_speed_state);
  float_rmat_vmult(&velocity_rot_gps , stateGetNedToBodyRMat_f(), (struct FloatVect3 *)&opti_state);

  float vel_x_opti = -((float)(velocity_rot_gps.y));
  float vel_y_opti = ((float)(velocity_rot_gps.x));

  // Calculate velocity error
  float vel_x_error = vel_x_opti - vel_x;
  float vel_y_error = vel_y_opti - vel_y;

//TODO:: Check out why vel_x_opti is 10 x big as stereocamera's output
  stereocam_data.data[8] = (uint8_t)((vel_x * 10) + 127); // dm/s
  stereocam_data.data[9] = (uint8_t)((vel_y * 10) + 127); // dm/s
  stereocam_data.data[19] = (uint8_t)((vel_x_opti) * 10 + 127); // dm/s
  stereocam_data.data[20] = (uint8_t)((vel_y_opti) * 10 + 127); // dm/s
  stereocam_data.data[21] = (uint8_t)((vel_x_error) * 10 + 127); // dm/s
  stereocam_data.data[22] = (uint8_t)((vel_y_error) * 10 + 127); // dm/s
  stereocam_data.data[23] = (uint8_t)((velocity_rot_state.x) * 10 + 127); // dm/s
  stereocam_data.data[24] = (uint8_t)((velocity_rot_state.y) * 10 + 127); // dm/s

  //Send measurement values in same structure as stereocam message (make sure SEND_STEREO in stereocam.c is FALSE)
  uint8_t frequency = 0;
  DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, &frequency, &(stereocam_data.len), stereocam_data.len,
                           stereocam_data.data);
#endif

  //Send velocity estimate to state
  //TODO:: Make variance dependable on line fit error, after new horizontal filter is made
  uint32_t now_ts = get_sys_time_usec();

  if (!(abs(vel_y) > 0.5 || abs(vel_x) > 0.5) || abs(dphi) > 0.05 || abs(dtheta) > 0.05) {
    AbiSendMsgVELOCITY_ESTIMATE(SENDER_ID, now_ts,
                                vel_x,
                                vel_y,
                                0.0f,
                                0.3f
                               );
  }

}
