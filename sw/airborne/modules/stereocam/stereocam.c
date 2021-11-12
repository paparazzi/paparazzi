/*
 * Copyright (C) 2015 Kirk Scheper
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

/** @file modules/stereocam/stereocam.c
 *  @brief interface to TU Delft serial stereocam
 *  Include stereocam.xml to your airframe file.
 *  Parameters STEREO_PORT, STEREO_BAUD, SEND_STEREO should be configured with stereocam.xml.
 */

#include "modules/stereocam/stereocam.h"

#include "mcu_periph/uart.h"
#include "modules/datalink/telemetry.h"
#include "pprzlink/messages.h"
#include "pprzlink/intermcu_msg.h"

#include "mcu_periph/sys_time.h"
#include "modules/core/abi.h"

#include "stereocam_follow_me/follow_me.h"


// forward received image to ground station
#ifndef FORWARD_IMAGE_DATA
#define FORWARD_IMAGE_DATA FALSE
#endif


/* This defines the location of the stereocamera with respect to the body fixed coordinates.
 *
 *    Coordinate system stereocam (image coordinates)
 *    z      x
 * (( * ))----->
 *    |                       * = arrow pointed into the frame away from you
 *    | y
 *    V
 *
 * The conversion order in euler angles is psi (yaw) -> theta (pitch) -> phi (roll)
 *
 * Standard rotations: MAV NED body to stereocam in Deg:
 * - facing forward:   90 -> 0 -> 90
 * - facing backward: -90 -> 0 -> 90
 * - facing downward:  90 -> 0 -> 0
 */

// general stereocam definitions
#if !defined(STEREO_BODY_TO_STEREO_PHI) || !defined(STEREO_BODY_TO_STEREO_THETA) || !defined(STEREO_BODY_TO_STEREO_PSI)
#warning "STEREO_BODY_TO_STEREO_XXX not defined. Using default Euler rotation angles (0,0,0)"
#endif

#ifndef STEREO_BODY_TO_STEREO_PHI
#define STEREO_BODY_TO_STEREO_PHI 0
#endif

#ifndef STEREO_BODY_TO_STEREO_THETA
#define STEREO_BODY_TO_STEREO_THETA 0
#endif

#ifndef STEREO_BODY_TO_STEREO_PSI
#define STEREO_BODY_TO_STEREO_PSI 0
#endif

struct stereocam_t stereocam = {
  .device = (&((UART_LINK).device)),
  .msg_available = false
};
static uint8_t stereocam_msg_buf[256]  __attribute__((aligned));   ///< The message buffer for the stereocamera

#ifndef STEREOCAM_USE_MEDIAN_FILTER
#define STEREOCAM_USE_MEDIAN_FILTER 0
#endif

#include "filters/median_filter.h"
struct MedianFilter3Float medianfilter;

void stereocam_init(void)
{
  struct FloatEulers euler = {STEREO_BODY_TO_STEREO_PHI, STEREO_BODY_TO_STEREO_THETA, STEREO_BODY_TO_STEREO_PSI};
  float_rmat_of_eulers(&stereocam.body_to_cam, &euler);

  // Initialize transport protocol
  pprz_transport_init(&stereocam.transport);

  InitMedianFilterVect3Float(medianfilter, MEDIAN_DEFAULT_SIZE);
}

/* Parse the InterMCU message */
static void stereocam_parse_msg(void)
{
  uint32_t now_ts = get_sys_time_usec();

  /* Parse the mag-pitot message */
  uint8_t msg_id = stereocam_msg_buf[1];
  switch (msg_id) {

  case DL_STEREOCAM_VELOCITY: {
    static struct FloatVect3 camera_vel;

    float res = (float)DL_STEREOCAM_VELOCITY_resolution(stereocam_msg_buf);

    camera_vel.x = (float)DL_STEREOCAM_VELOCITY_velx(stereocam_msg_buf)/res;
    camera_vel.y = (float)DL_STEREOCAM_VELOCITY_vely(stereocam_msg_buf)/res;
    camera_vel.z = (float)DL_STEREOCAM_VELOCITY_velz(stereocam_msg_buf)/res;

    float noise = 1-(float)DL_STEREOCAM_VELOCITY_vRMS(stereocam_msg_buf)/res;

    // Rotate camera frame to body frame
    struct FloatVect3 body_vel;
    float_rmat_transp_vmult(&body_vel, &stereocam.body_to_cam, &camera_vel);

    //todo make setting
    if (STEREOCAM_USE_MEDIAN_FILTER) {
      // Use a slight median filter to filter out the large outliers before sending it to state
      UpdateMedianFilterVect3Float(medianfilter, body_vel);
    }

    //Send velocities to state
    AbiSendMsgVELOCITY_ESTIMATE(VEL_STEREOCAM_ID, now_ts,
                                body_vel.x,
                                body_vel.y,
                                body_vel.z,
                                noise,
                                noise,
                                noise
                               );

    // todo activate this after changing optical flow message to be dimentionless instead of in pixels
    /*
    static struct FloatVect3 camera_flow;

    float avg_dist = (float)DL_STEREOCAM_VELOCITY_avg_dist(stereocam_msg_buf)/res;

    camera_flow.x = (float)DL_STEREOCAM_VELOCITY_velx(stereocam_msg_buf)/DL_STEREOCAM_VELOCITY_avg_dist(stereocam_msg_buf);
    camera_flow.y = (float)DL_STEREOCAM_VELOCITY_vely(stereocam_msg_buf)/DL_STEREOCAM_VELOCITY_avg_dist(stereocam_msg_buf);
    camera_flow.z = (float)DL_STEREOCAM_VELOCITY_velz(stereocam_msg_buf)/DL_STEREOCAM_VELOCITY_avg_dist(stereocam_msg_buf);

    struct FloatVect3 body_flow;
    float_rmat_transp_vmult(&body_flow, &body_to_stereocam, &camera_flow);

    AbiSendMsgOPTICAL_FLOW(STEREOCAM2STATE_SENDER_ID, now_ts,
                                body_flow.x,
                                body_flow.y,
                                body_flow.z,
                                quality,
                                body_flow.z,
                                avg_dist
                               );
    */
    break;
  }

  case DL_STEREOCAM_ARRAY: {
#if FORWARD_IMAGE_DATA
    // forward image to ground station
    uint8_t type = DL_STEREOCAM_ARRAY_type(stereocam_msg_buf);
    uint8_t w = DL_STEREOCAM_ARRAY_width(stereocam_msg_buf);
    uint8_t h = DL_STEREOCAM_ARRAY_height(stereocam_msg_buf);
    uint8_t nb = DL_STEREOCAM_ARRAY_package_nb(stereocam_msg_buf);
    uint8_t l = DL_STEREOCAM_ARRAY_image_data_length(stereocam_msg_buf);

    DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, &type, &w, &h, &nb,
        l, DL_STEREOCAM_ARRAY_image_data(stereocam_msg_buf));
#endif
    break;
  }

#ifdef STEREOCAM_FOLLOWME
  // todo is follow me still used?
  case DL_STEREOCAM_FOLLOW_ME: {
    follow_me( DL_STEREOCAM_FOLLOW_ME_headingToFollow(stereocam_msg_buf),
               DL_STEREOCAM_FOLLOW_ME_heightObject(stereocam_msg_buf),
               DL_STEREOCAM_FOLLOW_ME_distanceToObject(stereocam_msg_buf));
    break;
  }
#endif

    default:
      break;
  }
}

/* We need to wait for incomming messages */
void stereocam_event(void) {
  // Check if we got some message from the Magneto or Pitot
  pprz_check_and_parse(stereocam.device, &stereocam.transport, stereocam_msg_buf, &stereocam.msg_available);

  // If we have a message we should parse it
  if (stereocam.msg_available) {
    stereocam_parse_msg();
    stereocam.msg_available = false;
  }
}

/* Send state to camera to facilitate derotation
 *
 */
void state2stereocam(void)
{
  // rotate body angles to camera reference frame
  static struct FloatEulers cam_angles;
  float_rmat_mult(&cam_angles, &stereocam.body_to_cam, stateGetNedToBodyEulers_f());

  float agl = 0;//stateGetAgl);
  pprz_msg_send_STEREOCAM_STATE(&(stereocam.transport.trans_tx), stereocam.device,
      AC_ID, &(cam_angles.phi), &(cam_angles.theta), &(cam_angles.psi), &agl);
}
