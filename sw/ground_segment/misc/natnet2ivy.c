/*
 * Copyright (C) 2014 Freek van Tienen
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** \file natnet2ivy.c
*  \brief NatNet (GPS) to ivy forwarder
*
*   This receives aircraft position information through the Optitrack system
* NatNet UDP stream and forwards it to the ivy bus. An aircraft with the gps
* subsystem "datalink" is then able to parse the GPS position and use it to
* navigate inside the Optitrack system.
*/

#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <time.h>
#include <sys/time.h>

#include "std.h"
#include "arch/linux/udp_socket.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_algebra_double.h"

/** Debugging options */
uint8_t verbose = 0;
#define printf_natnet(...)   if(verbose > 1) fprintf (stderr, __VA_ARGS__)
#define printf_debug(...)    if(verbose > 0) fprintf (stderr, __VA_ARGS__)

/** NatNet defaults */
char *natnet_addr               = "255.255.255.255";
char *natnet_multicast_addr     = "239.255.42.99";
uint16_t natnet_cmd_port        = 1510;
uint16_t natnet_data_port       = 1511;
uint8_t natnet_major            = 2;
uint8_t natnet_minor            = 7;

/** Logging */
FILE *fp;
char *nameOfLogfile             = "natnet_log.dat";
bool log_exists = 0;
bool must_log = 0;

/** Ivy Bus default */
#ifdef __APPLE__
char *ivy_bus                   = "224.255.255.255";
#else
char *ivy_bus                   = "127.255.255.255:2010";
#endif

/** Sample frequency and derevitive defaults */
uint32_t freq_transmit          = 30;     ///< Transmitting frequency in Hz
uint16_t min_velocity_samples   = 4;      ///< The amount of position samples needed for a valid velocity
bool small_packets              = FALSE;

/** Connection timeout when not receiving **/
#define CONNECTION_TIMEOUT          .5

/** NatNet parsing defines */
#define MAX_PACKETSIZE    100000
#define MAX_NAMELENGTH    256
#define MAX_RIGIDBODIES   128

#define NAT_PING                    0
#define NAT_PINGRESPONSE            1
#define NAT_REQUEST                 2
#define NAT_RESPONSE                3
#define NAT_REQUEST_MODELDEF        4
#define NAT_MODELDEF                5
#define NAT_REQUEST_FRAMEOFDATA     6
#define NAT_FRAMEOFDATA             7
#define NAT_MESSAGESTRING           8
#define NAT_UNRECOGNIZED_REQUEST    100
#define UNDEFINED                   999999.9999

/** Tracked rigid bodies */
struct RigidBody {
  int id;                           ///< Rigid body ID from the tracking system
  float x, y, z;                    ///< Rigid body x, y and z coordinates in meters (note y and z are swapped)
  float qx, qy, qz, qw;             ///< Rigid body qx, qy, qz and qw rotations (note qy and qz are swapped)
  int nMarkers;                     ///< Number of markers inside the rigid body (both visible and not visible)
  float error;                      ///< Error of the position in cm
  int nSamples;                     ///< Number of samples since last transmit
  bool posSampled;                  ///< If the position is sampled last sampling

  double vel_x, vel_y, vel_z;       ///< Sum of the (last_vel_* - current_vel_*) during nVelocitySamples
  struct EcefCoor_d ecef_vel;       ///< Last valid ECEF velocity in meters
  int nVelocitySamples;             ///< Number of velocity samples gathered
  int totalVelocitySamples;         ///< Total amount of velocity samples possible
  int nVelocityTransmit;            ///< Amount of transmits since last valid velocity transmit
};
struct RigidBody rigidBodies[MAX_RIGIDBODIES];    ///< All rigid bodies which are tracked

/** Mapping between rigid body and aircraft */
struct Aircraft {
  uint8_t ac_id;
  float lastSample;
  bool connected;
};
struct Aircraft aircrafts[MAX_RIGIDBODIES];                  ///< Mapping from rigid body ID to aircraft ID

/** Natnet socket connections */
struct UdpSocket natnet_data, natnet_cmd;

/** Tracking location LTP and angle offset from north */
struct LtpDef_d tracking_ltp;       ///< The tracking system LTP definition
double tracking_offset_angle;       ///< The offset from the tracking system to the North in degrees

/** Save the latency from natnet */
float natnet_latency;

/** Parse the packet from NatNet */
void natnet_parse(unsigned char *in)
{
  int i, j, k;

  // Create a pointer to go trough the packet
  char *ptr = (char *)in;
  printf_natnet("Begin Packet\n-------\n");

  // Message ID
  int MessageID = 0;
  memcpy(&MessageID, ptr, 2); ptr += 2;
  printf_natnet("Message ID : %d\n", MessageID);

  // Packet size
  int nBytes = 0;
  memcpy(&nBytes, ptr, 2); ptr += 2;
  printf_natnet("Byte count : %d\n", nBytes);

  if (MessageID == NAT_FRAMEOFDATA) {   // FRAME OF MOCAP DATA packet
    // Frame number
    int frameNumber = 0; memcpy(&frameNumber, ptr, 4); ptr += 4;
    printf_natnet("Frame # : %d\n", frameNumber);

    // ========== MARKERSETS ==========
    // Number of data sets (markersets, rigidbodies, etc)
    int nMarkerSets = 0; memcpy(&nMarkerSets, ptr, 4); ptr += 4;
    printf_natnet("Marker Set Count : %d\n", nMarkerSets);

    for (i = 0; i < nMarkerSets; i++) {
      // Markerset name
      char szName[256];
      strcpy(szName, ptr);
      int nDataBytes = (int) strlen(szName) + 1;
      ptr += nDataBytes;
      printf_natnet("Model Name: %s\n", szName);

      // marker data
      int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
      printf_natnet("Marker Count : %d\n", nMarkers);

      for (j = 0; j < nMarkers; j++) {
        float x = 0; memcpy(&x, ptr, 4); ptr += 4;
        float y = 0; memcpy(&y, ptr, 4); ptr += 4;
        float z = 0; memcpy(&z, ptr, 4); ptr += 4;
        printf_natnet("\tMarker %d : [x=%3.2f,y=%3.2f,z=%3.2f]\n", j, x, y, z);
      }
    }

    // Unidentified markers
    int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
    printf_natnet("Unidentified Marker Count : %d\n", nOtherMarkers);
    for (j = 0; j < nOtherMarkers; j++) {
      float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
      float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
      float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
      printf_natnet("\tMarker %d : pos = [%3.2f,%3.2f,%3.2f]\n", j, x, y, z);
    }

    // ========== RIGID BODIES ==========
    // Rigid bodies
    int nRigidBodies = 0;
    memcpy(&nRigidBodies, ptr, 4); ptr += 4;
    printf_natnet("Rigid Body Count : %d\n", nRigidBodies);

    // Check if there ie enough space for the rigid bodies
    if (nRigidBodies > MAX_RIGIDBODIES) {
      fprintf(stderr,
              "Could not sample all the rigid bodies because the amount of rigid bodies is bigger then %d (MAX_RIGIDBODIES).\r\n",
              MAX_RIGIDBODIES);
      exit(EXIT_FAILURE);
    }

    for (j = 0; j < nRigidBodies; j++) {
      // rigid body pos/ori
      struct RigidBody old_rigid;
      memcpy(&old_rigid, &rigidBodies[j], sizeof(struct RigidBody));

      memcpy(&rigidBodies[j].id, ptr, 4); ptr += 4;
      memcpy(&rigidBodies[j].y, ptr, 4); ptr += 4;   //x --> Y
      memcpy(&rigidBodies[j].z, ptr, 4); ptr += 4;   //y --> Z
      memcpy(&rigidBodies[j].x, ptr, 4); ptr += 4;   //z --> X
      memcpy(&rigidBodies[j].qx, ptr, 4); ptr += 4;  //qx --> QX
      memcpy(&rigidBodies[j].qz, ptr, 4); ptr += 4;  //qy --> QZ
      memcpy(&rigidBodies[j].qy, ptr, 4); ptr += 4;  //qz --> QY
      memcpy(&rigidBodies[j].qw, ptr, 4); ptr += 4;  //qw --> QW
      printf_natnet("ID (%d) : %d\n", j, rigidBodies[j].id);
      printf_natnet("pos: [%3.2f,%3.2f,%3.2f]\n", rigidBodies[j].x, rigidBodies[j].y, rigidBodies[j].z);
      printf_natnet("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", rigidBodies[j].qx, rigidBodies[j].qy, rigidBodies[j].qz,
                    rigidBodies[j].qw);

      // Differentiate the position to get the speed (TODO: crossreference with labeled markers for occlussion)
      rigidBodies[j].totalVelocitySamples++;
      if (old_rigid.x != rigidBodies[j].x || old_rigid.y != rigidBodies[j].y || old_rigid.z != rigidBodies[j].z
          || old_rigid.qx != rigidBodies[j].qx || old_rigid.qy != rigidBodies[j].qy || old_rigid.qz != rigidBodies[j].qz
          || old_rigid.qw != rigidBodies[j].qw) {

        if (old_rigid.posSampled) {
          rigidBodies[j].vel_x += (rigidBodies[j].x - old_rigid.x);
          rigidBodies[j].vel_y += (rigidBodies[j].y - old_rigid.y);
          rigidBodies[j].vel_z += (rigidBodies[j].z - old_rigid.z);
          rigidBodies[j].nVelocitySamples++;
        }

        rigidBodies[j].nSamples++;
        rigidBodies[j].posSampled = TRUE;
      } else {
        rigidBodies[j].posSampled = FALSE;
      }

      // When marker id changed, reset the velocity
      if (old_rigid.id != rigidBodies[j].id) {
        rigidBodies[j].vel_x = 0;
        rigidBodies[j].vel_y = 0;
        rigidBodies[j].vel_z = 0;
        rigidBodies[j].nSamples = 0;
        rigidBodies[j].nVelocitySamples = 0;
        rigidBodies[j].totalVelocitySamples = 0;
        rigidBodies[j].posSampled = FALSE;
      }

      // Associated marker positions
      memcpy(&rigidBodies[j].nMarkers, ptr, 4); ptr += 4;
      printf_natnet("Marker Count: %d\n", rigidBodies[j].nMarkers);
      int nBytes = rigidBodies[j].nMarkers * 3 * sizeof(float);
      float *markerData = (float *)malloc(nBytes);
      memcpy(markerData, ptr, nBytes);
      ptr += nBytes;

      if (natnet_major >= 2) {
        // Associated marker IDs
        nBytes = rigidBodies[j].nMarkers * sizeof(int);
        int *markerIDs = (int *)malloc(nBytes);
        memcpy(markerIDs, ptr, nBytes);
        ptr += nBytes;

        // Associated marker sizes
        nBytes = rigidBodies[j].nMarkers * sizeof(float);
        float *markerSizes = (float *)malloc(nBytes);
        memcpy(markerSizes, ptr, nBytes);
        ptr += nBytes;

        for (k = 0; k < rigidBodies[j].nMarkers; k++) {
          printf_natnet("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k],
                        markerData[k * 3], markerData[k * 3 + 1], markerData[k * 3 + 2]);
        }

        if (markerIDs) {
          free(markerIDs);
        }
        if (markerSizes) {
          free(markerSizes);
        }

      } else {
        for (k = 0; k < rigidBodies[j].nMarkers; k++) {
          printf_natnet("\tMarker %d: pos = [%3.2f,%3.2f,%3.2f]\n", k, markerData[k * 3], markerData[k * 3 + 1],
                        markerData[k * 3 + 2]);
        }
      }
      if (markerData) {
        free(markerData);
      }

      if (natnet_major >= 2) {
        // Mean marker error
        memcpy(&rigidBodies[j].error, ptr, 4); ptr += 4;
        printf_natnet("Mean marker error: %3.8f\n", rigidBodies[j].error);
      }

      // 2.6 and later
      if (((natnet_major == 2) && (natnet_minor >= 6)) || (natnet_major > 2) || (natnet_major == 0)) {
        // params
        short params = 0; memcpy(&params, ptr, 2); ptr += 2;
//           bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
      }
    } // next rigid body

    // ========== SKELETONS ==========
    // Skeletons (version 2.1 and later)
    if (((natnet_major == 2) && (natnet_minor > 0)) || (natnet_major > 2)) {
      int nSkeletons = 0;
      memcpy(&nSkeletons, ptr, 4); ptr += 4;
      printf_natnet("Skeleton Count : %d\n", nSkeletons);
      for (j = 0; j < nSkeletons; j++) {
        // Skeleton id
        int skeletonID = 0;
        memcpy(&skeletonID, ptr, 4); ptr += 4;
        // # of rigid bodies (bones) in skeleton
        int nRigidBodies = 0;
        memcpy(&nRigidBodies, ptr, 4); ptr += 4;
        printf_natnet("Rigid Body Count : %d\n", nRigidBodies);
        for (j = 0; j < nRigidBodies; j++) {
          // Rigid body pos/ori
          int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
          float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
          float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
          float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
          float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
          float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
          float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
          float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
          printf_natnet("ID : %d\n", ID);
          printf_natnet("pos: [%3.2f,%3.2f,%3.2f]\n", x, y, z);
          printf_natnet("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx, qy, qz, qw);

          // Sssociated marker positions
          int nRigidMarkers = 0;  memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
          printf_natnet("Marker Count: %d\n", nRigidMarkers);
          int nBytes = nRigidMarkers * 3 * sizeof(float);
          float *markerData = (float *)malloc(nBytes);
          memcpy(markerData, ptr, nBytes);
          ptr += nBytes;

          // Associated marker IDs
          nBytes = nRigidMarkers * sizeof(int);
          int *markerIDs = (int *)malloc(nBytes);
          memcpy(markerIDs, ptr, nBytes);
          ptr += nBytes;

          // Associated marker sizes
          nBytes = nRigidMarkers * sizeof(float);
          float *markerSizes = (float *)malloc(nBytes);
          memcpy(markerSizes, ptr, nBytes);
          ptr += nBytes;

          for (k = 0; k < nRigidMarkers; k++) {
            printf_natnet("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k],
                          markerData[k * 3], markerData[k * 3 + 1], markerData[k * 3 + 2]);
          }

          // Mean marker error
          float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
          printf_natnet("Mean marker error: %3.2f\n", fError);

          // Release resources
          if (markerIDs) {
            free(markerIDs);
          }
          if (markerSizes) {
            free(markerSizes);
          }
          if (markerData) {
            free(markerData);
          }
        } // next rigid body
      } // next skeleton
    }

    // ========== LABELED MARKERS ==========
    // Labeled markers (version 2.3 and later)
    if (((natnet_major == 2) && (natnet_minor >= 3)) || (natnet_major > 2)) {
      int nLabeledMarkers = 0;
      memcpy(&nLabeledMarkers, ptr, 4); ptr += 4;
      printf_natnet("Labeled Marker Count : %d\n", nLabeledMarkers);
      for (j = 0; j < nLabeledMarkers; j++) {
        int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
        float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
        float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
        float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
        float size = 0.0f; memcpy(&size, ptr, 4); ptr += 4;

        printf_natnet("ID  : %d\n", ID);
        printf_natnet("pos : [%3.2f,%3.2f,%3.2f]\n", x, y, z);
        printf_natnet("size: [%3.2f]\n", size);
      }
    }

    // Latency
    natnet_latency = 0.0f; memcpy(&natnet_latency, ptr, 4); ptr += 4;
    printf_natnet("latency : %3.3f\n", natnet_latency);

    // Timecode
    unsigned int timecode = 0;  memcpy(&timecode, ptr, 4);  ptr += 4;
    unsigned int timecodeSub = 0; memcpy(&timecodeSub, ptr, 4); ptr += 4;
    printf_natnet("timecode : %d %d", timecode, timecodeSub);

    // End of data tag
    int eod = 0; memcpy(&eod, ptr, 4); ptr += 4;
    printf_natnet("End Packet\n-------------\n");
  } else {
    printf("Error: Unrecognized packet type from Optitrack NatNet.\n");
  }
}

/** The transmitter periodic function */
gboolean timeout_transmit_callback(gpointer data)
{
  int i;

  // Loop trough all the available rigidbodies (TODO: optimize)
  for (i = 0; i < MAX_RIGIDBODIES; i++) {
    // Check if ID's are correct
    if (rigidBodies[i].id >= MAX_RIGIDBODIES) {
      fprintf(stderr,
              "Could not parse rigid body %d from NatNet, because ID is higher then or equal to %d (MAX_RIGIDBODIES-1).\r\n",
              rigidBodies[i].id, MAX_RIGIDBODIES - 1);
      exit(EXIT_FAILURE);
    }

    // Check if we want to transmit (follow) this rigid
    if (aircrafts[rigidBodies[i].id].ac_id == 0) {
      continue;
    }

    // When we don track anymore and timeout or start tracking
    if (rigidBodies[i].nSamples < 1
        && aircrafts[rigidBodies[i].id].connected
        && (natnet_latency - aircrafts[rigidBodies[i].id].lastSample) > CONNECTION_TIMEOUT) {
      aircrafts[rigidBodies[i].id].connected = FALSE;
      fprintf(stderr, "#error Lost tracking rigid id %d, aircraft id %d.\n",
              rigidBodies[i].id, aircrafts[rigidBodies[i].id].ac_id);
    } else if (rigidBodies[i].nSamples > 0 && !aircrafts[rigidBodies[i].id].connected) {
      fprintf(stderr, "#pragma message: Now tracking rigid id %d, aircraft id %d.\n",
              rigidBodies[i].id, aircrafts[rigidBodies[i].id].ac_id);
    }

    // Check if we still track the rigid
    if (rigidBodies[i].nSamples < 1) {
      continue;
    }

    // Update the last tracked
    aircrafts[rigidBodies[i].id].connected = TRUE;
    aircrafts[rigidBodies[i].id].lastSample = natnet_latency;

    // Defines to make easy use of paparazzi math
    struct EnuCoor_d pos, speed;
    struct EcefCoor_d ecef_pos;
    struct LlaCoor_d lla_pos;
    struct DoubleQuat orient;
    struct DoubleEulers orient_eulers;

    // Add the Optitrack angle to the x and y positions
    pos.x = cos(tracking_offset_angle) * rigidBodies[i].x - sin(tracking_offset_angle) * rigidBodies[i].y;
    pos.y = sin(tracking_offset_angle) * rigidBodies[i].x + cos(tracking_offset_angle) * rigidBodies[i].y;
    pos.z = rigidBodies[i].z;

    // Convert the position to ecef and lla based on the Optitrack LTP
    ecef_of_enu_point_d(&ecef_pos , &tracking_ltp , &pos);
    lla_of_ecef_d(&lla_pos, &ecef_pos);

    // Check if we have enough samples to estimate the velocity
    rigidBodies[i].nVelocityTransmit++;
    if (rigidBodies[i].nVelocitySamples >= min_velocity_samples) {
      // Calculate the derevative of the sum to get the correct velocity     (1 / freq_transmit) * (samples / total_samples)
      double sample_time = //((double)rigidBodies[i].nVelocitySamples / (double)rigidBodies[i].totalVelocitySamples) /
        ((double)rigidBodies[i].nVelocityTransmit / (double)freq_transmit);
      rigidBodies[i].vel_x = rigidBodies[i].vel_x / sample_time;
      rigidBodies[i].vel_y = rigidBodies[i].vel_y / sample_time;
      rigidBodies[i].vel_z = rigidBodies[i].vel_z / sample_time;

      // Add the Optitrack angle to the x and y velocities
      speed.x = cos(tracking_offset_angle) * rigidBodies[i].vel_x - sin(tracking_offset_angle) * rigidBodies[i].vel_y;
      speed.y = sin(tracking_offset_angle) * rigidBodies[i].vel_x + cos(tracking_offset_angle) * rigidBodies[i].vel_y;
      speed.z = rigidBodies[i].vel_z;

      // Conver the speed to ecef based on the Optitrack LTP
      ecef_of_enu_vect_d(&rigidBodies[i].ecef_vel , &tracking_ltp , &speed);
    }

    // Copy the quaternions and convert to euler angles for the heading
    orient.qi = rigidBodies[i].qw;
    orient.qx = rigidBodies[i].qx;
    orient.qy = rigidBodies[i].qy;
    orient.qz = rigidBodies[i].qz;
    double_eulers_of_quat(&orient_eulers, &orient);

    // Calculate the heading by adding the Natnet offset angle and normalizing it
    double heading = -orient_eulers.psi + 90.0 / 57.6 -
                     tracking_offset_angle; //the optitrack axes are 90 degrees rotated wrt ENU
    NormRadAngle(heading);

    printf_debug("[%d -> %d]Samples: %d\t%d\t\tTiming: %3.3f latency\n", rigidBodies[i].id,
                 aircrafts[rigidBodies[i].id].ac_id
                 , rigidBodies[i].nSamples, rigidBodies[i].nVelocitySamples, natnet_latency);
    printf_debug("    Heading: %f\t\tPosition: %f\t%f\t%f\t\tVelocity: %f\t%f\t%f\n", DegOfRad(heading),
                 rigidBodies[i].x, rigidBodies[i].y, rigidBodies[i].z,
                 rigidBodies[i].ecef_vel.x, rigidBodies[i].ecef_vel.y, rigidBodies[i].ecef_vel.z);


    /* Construct time of time of week (tow) */
    time_t now;
    time(&now);
    struct tm *ts = localtime(&now);
    
    uint32_t tow = (ts->tm_wday - 1)*(24*60*60*1000) + ts->tm_hour*(60*60*1000) + ts->tm_min*(60*1000) + ts->tm_sec*1000;

    // Transmit the REMOTE_GPS packet on the ivy bus (either small or big)
    if (small_packets) {
      /* The local position is an int32 and the 11 LSBs of the (signed) x and y axis are compressed into
       * a single integer. The z axis is considered unsigned and only the latter 10 LSBs are
       * used.
       */

      uint32_t pos_xyz = 0;
      // check if position within limits
      if (fabs(pos.x * 100.) < pow(2, 10)) {
        pos_xyz = (((uint32_t)(pos.x * 100.0)) & 0x7FF) << 21;                     // bits 31-21 x position in cm
      } else {
        fprintf(stderr, "Warning!! X position out of maximum range of small message (±%.2fm): %.2f", pow(2, 10) / 100, pos.x);
        pos_xyz = (((uint32_t)(pow(2, 10) * pos.x / fabs(pos.x))) & 0x7FF) << 21;  // bits 31-21 x position in cm
      }

      if (fabs(pos.y * 100.) < pow(2, 10)) {
        pos_xyz |= (((uint32_t)(pos.y * 100.0)) & 0x7FF) << 10;                    // bits 20-10 y position in cm
      } else {
        fprintf(stderr, "Warning!! Y position out of maximum range of small message (±%.2fm): %.2f", pow(2, 10) / 100, pos.y);
        pos_xyz |= (((uint32_t)(pow(2, 10) * pos.y / fabs(pos.y))) & 0x7FF) << 10; // bits 20-10 y position in cm
      }

      if (pos.z * 100. < pow(2, 10) && pos.z > 0.) {
        pos_xyz |= (((uint32_t)(fabs(pos.z) * 100.0)) & 0x3FF);                          // bits 9-0 z position in cm
      } else if (pos.z > 0.) {
        fprintf(stderr, "Warning!! Z position out of maximum range of small message (%.2fm): %.2f", pow(2, 10) / 100, pos.z);
        pos_xyz |= (((uint32_t)(pow(2, 10))) & 0x3FF);                             // bits 9-0 z position in cm
      }
      // printf("ENU Pos: %u (%.2f, %.2f, %.2f)\n", pos_xyz, pos.x, pos.y, pos.z);

      /* The speed is an int32 and the 11 LSBs of the x and y axis and 10 LSBs of z (all signed) are compressed into
       * a single integer.
       */
      uint32_t speed_xyz = 0;
      // check if speed within limits
      if (fabs(speed.x * 100) < pow(2, 10)) {
        speed_xyz = (((uint32_t)(speed.x * 100.0)) & 0x7FF) << 21;                       // bits 31-21 speed x in cm/s
      } else {
        fprintf(stderr, "Warning!! X Speed out of maximum range of small message (±%.2fm/s): %.2f", pow(2, 10) / 100, speed.x);
        speed_xyz = (((uint32_t)(pow(2, 10) * speed.x / fabs(speed.x))) & 0x7FF) << 21;  // bits 31-21 speed x in cm/s
      }

      if (fabs(speed.y * 100) < pow(2, 10)) {
        speed_xyz |= (((uint32_t)(speed.y * 100.0)) & 0x7FF) << 10;                      // bits 20-10 speed y in cm/s
      } else {
        fprintf(stderr, "Warning!! Y Speed out of maximum range of small message (±%.2fm/s): %.2f", pow(2, 10) / 100, speed.y);
        speed_xyz |= (((uint32_t)(pow(2, 10) * speed.y / fabs(speed.y))) & 0x7FF) << 10; // bits 20-10 speed y in cm/s
      }

      if (fabs(speed.z * 100) < pow(2, 9)) {
        speed_xyz |= (((uint32_t)(speed.z * 100.0)) & 0x3FF);                            // bits 9-0 speed z in cm/s
      } else {
        fprintf(stderr, "Warning!! Z Speed out of maximum range of small message (±%.2fm/s): %.2f", pow(2, 9) / 100, speed.z);
        speed_xyz |= (((uint32_t)(pow(2, 9) * speed.z / fabs(speed.z))) & 0x3FF);       // bits 9-0 speed z in cm/s
      }

      /* The gps_small msg should always be less than 20 bytes including the pprz header of 6 bytes
       * This is primarily due to the maximum packet size of the bluetooth msgs of 19 bytes
       * increases the probability that a complete message will be accepted
       */
      IvySendMsg("0 REMOTE_GPS_SMALL %d %d %d %d %d",
                (int16_t)(heading * 10000),       // int16_t heading in rad*1e4 (2 bytes)
                pos_xyz,                          // uint32 ENU X, Y and Z in CM (4 bytes)
                speed_xyz,                        // uint32 ENU velocity X, Y, Z in cm/s (4 bytes)
                tow,                              // uint32_t time of day
                aircrafts[rigidBodies[i].id].ac_id); // uint8 rigid body ID (1 byte)

    } else {
      IvySendMsg("0 REMOTE_GPS %d %d %d %d %d %d %d %d %d %d %d %d %d %d", aircrafts[rigidBodies[i].id].ac_id,
                 rigidBodies[i].nMarkers,                //uint8 Number of markers (sv_num)
                 (int)(ecef_pos.x * 100.0),              //int32 ECEF X in CM
                 (int)(ecef_pos.y * 100.0),              //int32 ECEF Y in CM
                 (int)(ecef_pos.z * 100.0),              //int32 ECEF Z in CM
                 (int)(DegOfRad(lla_pos.lat) * 10000000.0),        //int32 LLA latitude in deg*1e7
                 (int)(DegOfRad(lla_pos.lon) * 10000000.0),        //int32 LLA longitude in deg*1e7
                 (int)(lla_pos.alt * 1000.0),            //int32 LLA altitude in mm above elipsoid
                 (int)(rigidBodies[i].z * 1000.0),       //int32 HMSL height above mean sea level in mm
                 (int)(rigidBodies[i].ecef_vel.x * 100.0), //int32 ECEF velocity X in cm/s
                 (int)(rigidBodies[i].ecef_vel.y * 100.0), //int32 ECEF velocity Y in cm/s
                 (int)(rigidBodies[i].ecef_vel.z * 100.0), //int32 ECEF velocity Z in cm/s
                 tow,
                 (int)(heading * 10000000.0));           //int32 Course in rad*1e7
    }
    if (must_log) {
      if (log_exists == 0) {
        fp = fopen(nameOfLogfile, "w");
        log_exists = 1;
      }

      if (fp == NULL) {
        printf("I couldn't open file for writing.\n");
        exit(0);
      } else {
        struct timeval cur_time;
        gettimeofday(&cur_time, NULL);
        fprintf(fp, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", aircrafts[rigidBodies[i].id].ac_id,
                rigidBodies[i].nMarkers,                //uint8 Number of markers (sv_num)
                (int)(ecef_pos.x * 100.0),              //int32 ECEF X in CM
                (int)(ecef_pos.y * 100.0),              //int32 ECEF Y in CM
                (int)(ecef_pos.z * 100.0),              //int32 ECEF Z in CM
                (int)(DegOfRad(lla_pos.lat) * 1e7),     //int32 LLA latitude in deg*1e7
                (int)(DegOfRad(lla_pos.lon) * 1e7),     //int32 LLA longitude in deg*1e7
                (int)(lla_pos.alt*1000.0),              //int32 LLA altitude in mm above elipsoid
                (int)(rigidBodies[i].z * 1000.0),       //int32 HMSL height above mean sea level in mm
                (int)(rigidBodies[i].ecef_vel.x * 100.0), //int32 ECEF velocity X in cm/s
                (int)(rigidBodies[i].ecef_vel.y * 100.0), //int32 ECEF velocity Y in cm/s
                (int)(rigidBodies[i].ecef_vel.z * 100.0), //int32 ECEF velocity Z in cm/s
                (int)(heading * 10000000.0),            //int32 Course in rad*1e7
                (int)cur_time.tv_sec,
                (int)cur_time.tv_usec);
      }
    }





    // Reset the velocity differentiator if we calculated the velocity
    if (rigidBodies[i].nVelocitySamples >= min_velocity_samples) {
      rigidBodies[i].vel_x = 0;
      rigidBodies[i].vel_y = 0;
      rigidBodies[i].vel_z = 0;
      rigidBodies[i].nVelocitySamples = 0;
      rigidBodies[i].totalVelocitySamples = 0;
      rigidBodies[i].nVelocityTransmit = 0;
    }

    rigidBodies[i].nSamples = 0;
  }

  return TRUE;
}

/** The NatNet sampler periodic function */
static gboolean sample_data(GIOChannel *chan, GIOCondition cond, gpointer data)
{
  static unsigned char buffer_data[MAX_PACKETSIZE];
  static int bytes_data = 0;

  // Keep on reading until we have the whole packet
  bytes_data += udp_socket_recv(&natnet_data, buffer_data, MAX_PACKETSIZE);

  // Parse NatNet data
  if (bytes_data >= 2 && bytes_data >= buffer_data[1]) {
    natnet_parse(buffer_data);
    bytes_data = 0;
  }

  return TRUE;
}


/** Print the program help */
void print_help(char *filename)
{
  static const char *usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -h, --help                Display this help\n"
    "   -v, --verbose <level>     Verbosity level 0-2 (0)\n\n"

    "   -ac <rigid_id> <ac_id>    Use rigid ID for GPS of ac_id (multiple possible)\n\n"
    "   -log <name of file>         Log to a file\n\n"
    "   -multicast_addr <ip>      NatNet server multicast address (239.255.42.99)\n"
    "   -server <ip>              NatNet server IP address (255.255.255.255)\n"
    "   -version <id>             NatNet server version (2.5)\n"
    "   -data_port <port>         NatNet server data socket UDP port (1510)\n"
    "   -cmd_port <port>          NatNet server command socket UDP port (1511)\n\n"

    "   -ecef <x> <y> <z>         ECEF coordinates of the tracking system\n"
    "   -lla <lat> <lon> <alt>    Latitude, longitude and altitude of the tracking system\n"
    "   -offset_angle <degree>    Tracking system angle offset compared to the North in degrees\n\n"

    "   -tf <freq>                Transmit frequency to the ivy bus in hertz (60)\n"
    "   -vel_samples <samples>    Minimum amount of samples for the velocity differentiator (4)\n"
    "   -small                    Send small packets instead of bigger (FALSE)\n\n"

    "   -ivy_bus <address:port>   Ivy bus address and port (127.255.255.255:2010)\n";
  fprintf(stderr, usage, filename);
}

/** Check the amount of arguments */
void check_argcount(int argc, char **argv, int i, int expected)
{
  if (i + expected >= argc) {
    fprintf(stderr, "Option %s expected %d arguments\r\n\r\n", argv[i], expected);
    print_help(argv[0]);
    exit(0);
  }
}

/** Parse the options from the commandline */
static void parse_options(int argc, char **argv)
{
  int i, count_ac = 0;
  for (i = 1; i < argc; ++i) {

    // Print help
    if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
      print_help(argv[0]);
      exit(0);
    }
    // Set the verbosity level
    if (strcmp(argv[i], "--verbosity") == 0 || strcmp(argv[i], "-v") == 0) {
      check_argcount(argc, argv, i, 1);

      verbose = atoi(argv[++i]);
    }

    // Set an rigid body to ivy ac_id
    else if (strcmp(argv[i], "-ac") == 0) {
      check_argcount(argc, argv, i, 2);

      int rigid_id = atoi(argv[++i]);
      uint8_t ac_id = atoi(argv[++i]);

      if (rigid_id >= MAX_RIGIDBODIES) {
        fprintf(stderr, "Rigid body ID must be less then %d (MAX_RIGIDBODIES)\n\n", MAX_RIGIDBODIES);
        print_help(argv[0]);
        exit(EXIT_FAILURE);
      }
      aircrafts[rigid_id].ac_id = ac_id;
      count_ac++;
    }
    // See if we want to log to a file
    else if (strcmp(argv[i], "-log") == 0) {
      check_argcount(argc, argv, i, 1);

      nameOfLogfile = argv[++i];
      must_log = 1;
    }

    // Set the NatNet multicast address
    else if (strcmp(argv[i], "-multicast_addr") == 0) {
      check_argcount(argc, argv, i, 1);

      natnet_multicast_addr = argv[++i];
    }
    // Set the NatNet server ip address
    else if (strcmp(argv[i], "-server") == 0) {
      check_argcount(argc, argv, i, 1);

      natnet_addr = argv[++i];
    }
    // Set the NatNet server version
    else if (strcmp(argv[i], "-version") == 0) {
      check_argcount(argc, argv, i, 1);

      float version = atof(argv[++i]);
      natnet_major = (uint8_t)version;
      natnet_minor = (uint8_t)(version * 10.0) % 10;
    }
    // Set the NatNet server data port
    else if (strcmp(argv[i], "-data_port") == 0) {
      check_argcount(argc, argv, i, 1);

      natnet_data_port = atoi(argv[++i]);
    }
    // Set the NatNet server command port
    else if (strcmp(argv[i], "-cmd_port") == 0) {
      check_argcount(argc, argv, i, 1);

      natnet_cmd_port = atoi(argv[++i]);
    }

    // Set the Tracking system position in ECEF
    else if (strcmp(argv[i], "-ecef") == 0) {
      check_argcount(argc, argv, i, 3);

      struct EcefCoor_d tracking_ecef;
      tracking_ecef.x  = atof(argv[++i]);
      tracking_ecef.y  = atof(argv[++i]);
      tracking_ecef.z  = atof(argv[++i]);
      ltp_def_from_ecef_d(&tracking_ltp, &tracking_ecef);
    }
    // Set the tracking system position in LLA
    else if (strcmp(argv[i], "-lla") == 0) {
      check_argcount(argc, argv, i, 3);

      struct LlaCoor_d tracking_lla;
      tracking_lla.lat  = atof(argv[++i]);
      tracking_lla.lon  = atof(argv[++i]);
      tracking_lla.alt  = atof(argv[++i]);
      ltp_def_from_lla_d(&tracking_ltp, &tracking_lla);
    }
    // Set the tracking system offset angle in degrees
    else if (strcmp(argv[i], "-offset_angle") == 0) {
      check_argcount(argc, argv, i, 1);

      tracking_offset_angle = atof(argv[++i]);
    }

    // Set the transmit frequency
    else if (strcmp(argv[i], "-tf") == 0) {
      check_argcount(argc, argv, i, 1);

      freq_transmit = atoi(argv[++i]);
    }
    // Set the minimum amount of velocity samples for the differentiator
    else if (strcmp(argv[i], "-vel_samples") == 0) {
      check_argcount(argc, argv, i, 1);

      min_velocity_samples = atoi(argv[++i]);
    }
    // Set to use small packets
    else if (strcmp(argv[i], "-small") == 0) {
      small_packets = TRUE;
    }

    // Set the ivy bus
    else if (strcmp(argv[i], "-ivy_bus") == 0) {
      check_argcount(argc, argv, i, 1);

      ivy_bus = argv[++i];
    }

    // Unknown option
    else {
      fprintf(stderr, "Unknown option %s\r\n\r\n", argv[i]);
      print_help(argv[0]);
      exit(0);
    }
  }

  // Check if at least one aircraft is set
  if (count_ac < 1) {
    fprintf(stderr, "You must specify at least one aircraft (-ac <rigid_id> <ac_id>)\n\n");
    print_help(argv[0]);
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char **argv)
{
  // Set the default tracking system position and angle
  struct LlaCoor_d tracking_lla;
  tracking_lla.lat = RadOfDeg(51.9906340);
  tracking_lla.lon = RadOfDeg(4.3767889);
  tracking_lla.alt = 45.103;
  tracking_offset_angle = 33.0 / 57.6;
  ltp_def_from_lla_d(&tracking_ltp, &tracking_lla);

  // Parse the options from cmdline
  parse_options(argc, argv);
  printf_debug("Tracking system Latitude: %f Longitude: %f Offset to North: %f degrees\n", DegOfRad(tracking_ltp.lla.lat),
               DegOfRad(tracking_ltp.lla.lon), DegOfRad(tracking_offset_angle));

  // Create the network connections
  printf_debug("Starting NatNet listening (multicast address: %s, data port: %d, version: %d.%d)\n",
               natnet_multicast_addr, natnet_data_port, natnet_major, natnet_minor);
  udp_socket_create(&natnet_data, "", -1, natnet_data_port, 0); // Only receiving
  udp_socket_subscribe_multicast(&natnet_data, natnet_multicast_addr);
  udp_socket_set_recvbuf(&natnet_data, 0x100000); // 1MB

  printf_debug("Starting NatNet command socket (server address: %s, command port: %d)\n", natnet_addr, natnet_cmd_port);
  udp_socket_create(&natnet_cmd, natnet_addr, natnet_cmd_port, 0, 1);
  udp_socket_set_recvbuf(&natnet_cmd, 0x100000); // 1MB

  // Create the Ivy Client
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  IvyInit("natnet2ivy", "natnet2ivy READY", 0, 0, 0, 0);
  IvyStart(ivy_bus);

  // Create the main timers
  printf_debug("Starting transmitting and sampling timeouts (transmitting frequency: %dHz, minimum velocity samples: %d)\n",
               freq_transmit, min_velocity_samples);
  g_timeout_add(1000 / freq_transmit, timeout_transmit_callback, NULL);

  GIOChannel *sk = g_io_channel_unix_new(natnet_data.sockfd);
  g_io_add_watch(sk, G_IO_IN | G_IO_NVAL | G_IO_HUP,
                 sample_data, NULL);

  // Run the main loop
  g_main_loop_run(ml);

  return 0;
}
