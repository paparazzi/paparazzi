/*
 * Paparazzi mcu0 $Id: quad_xsens.c,v 1.2 2008/05/07 12:54:23 gautier Exp $
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

/** \file csc_xsens.c
 * \brief Parser for the Xsens protocol
 */

#include "mercury_xsens.h"
#include "booz/booz_imu.h"
#include "booz/booz_ahrs.h"
#include "booz/ahrs/booz_ahrs_aligner.h"
#include "booz/booz_imu.h"
#include "csc_booz2_ins.h"

#include <inttypes.h>

#include "led.h"

#include "downlink.h"
#include "messages.h"
#include "uart.h"
//#include "com_stats.h"
#include "math/pprz_algebra_float.h"
#include "string.h"

void parse_xsens_msg(uint8_t xsens_id, uint8_t c );

#define __Xsens2Link(dev, _x) dev##_x
#define _Xsens2Link(dev, _x)  __Xsens2Link(dev, _x)
#define Xsens2Link(_x) _Xsens2Link(XSENS2_LINK, _x)

#define Xsens2Buffer() Xsens2Link(ChAvailable())
#define ReadXsens2Buffer() { while (Xsens2Link(ChAvailable())&&!xsens_msg_received[1]) parse_xsens_msg(1, Xsens2Link(Getch())); }

#define Xsens2UartSend1(c) Xsens2Link(Transmit(c))
#define Xsens2UartInitParam(_a,_b,_c) Xsens2Link(InitParam(_a,_b,_c))
#define Xsens2UartRunning Xsens2Link(TxRunning)

#define Xsens2InitCheksum() { send_ck[1] = 0; }
#define Xsens2UpdateChecksum(c) { send_ck[1] += c; }

#define Xsens2Send1(c) { uint8_t i8=c; XSens2UartSend1(i8); Xsens2UpdateChecksum(i8); }
#define Xsens2Send1ByAddr(x) { Xsens2Send1(*x); }
#define Xsens2Send2ByAddr(x) { Xsens2Send1(*(x+1)); Xsens2Send1(*x); }
#define Xsens2Send4ByAddr(x) { Xsens2Send1(*(x+3)); Xsens2Send1(*(x+2)); Xsens2Send1(*(x+1)); Xsens2Send1(*x); }

#define Xsens2Header(msg_id, len) { \
  Xsens2UartSend1(XSENS_START); \
  Xsens2InitCheksum(); \
  Xsens2Send1(XSENS_BID); \
  Xsens2Send1(msg_id); \
  Xsens2Send1(len); \
}
#define Xsens2Trailer() { uint8_t i8=0x100-send_ck[1]; Xsens2UartSend1(i8); }

#define __Xsens1Link(dev, _x) dev##_x
#define _Xsens1Link(dev, _x)  __Xsens1Link(dev, _x)
#define Xsens1Link(_x) _Xsens1Link(XSENS1_LINK, _x)

#define Xsens1Buffer() Xsens1Link(ChAvailable())
#define ReadXsens1Buffer() { while (Xsens1Link(ChAvailable())&&!xsens_msg_received[0]) parse_xsens_msg(0, Xsens1Link(Getch())); }

#define Xsens1UartSend1(c) Xsens1Link(Transmit(c))
#define Xsens1UartInitParam(_a,_b,_c) Xsens1Link(InitParam(_a,_b,_c))
#define Xsens1UartRunning Xsens1Link(TxRunning)

#define Xsens1InitCheksum() { send_ck[0] = 0; }
#define Xsens1UpdateChecksum(c) { send_ck[0] += c; }

#define Xsens1Send1(c) { uint8_t i8=c; XSens1UartSend1(i8); Xsens1UpdateChecksum(i8); }
#define Xsens1Send1ByAddr(x) { Xsens1Send1(*x); }
#define Xsens1Send2ByAddr(x) { Xsens1Send1(*(x+1)); Xsens1Send1(*x); }
#define Xsens1Send4ByAddr(x) { Xsens1Send1(*(x+3)); Xsens1Send1(*(x+2)); Xsens1Send1(*(x+1)); Xsens1Send1(*x); }

#define Xsens1Header(msg_id, len) { \
  Xsens1UartSend1(XSENS_START); \
  Xsens1InitCheksum(); \
  Xsens1Send1(XSENS_BID); \
  Xsens1Send1(msg_id); \
  Xsens1Send1(len); \
}
#define Xsens1Trailer() { uint8_t i8=0x100-send_ck[0]; Xsens1UartSend1(i8); }


/** Includes macros generated from xsens_MTi.xml */ 
#include "xsens_protocol.h"

uint8_t xsens_mode[XSENS_COUNT]; // Receiver status 
volatile uint8_t xsens_msg_received[XSENS_COUNT];

float xsens_phi[XSENS_COUNT];
float xsens_theta[XSENS_COUNT];
float xsens_psi[XSENS_COUNT];

float xsens_r_a[XSENS_COUNT];
float xsens_r_b[XSENS_COUNT];
float xsens_r_c[XSENS_COUNT];
float xsens_r_d[XSENS_COUNT];
float xsens_r_e[XSENS_COUNT];
float xsens_r_f[XSENS_COUNT];
float xsens_r_g[XSENS_COUNT];
float xsens_r_h[XSENS_COUNT];
float xsens_r_i[XSENS_COUNT];

struct FloatRMat xsens_rmat[XSENS_COUNT];
struct FloatRMat xsens_rmat_neutral[XSENS_COUNT];
struct FloatRMat xsens_rmat_adj[XSENS_COUNT];

float xsens_accel_x[XSENS_COUNT];
float xsens_accel_y[XSENS_COUNT];
float xsens_accel_z[XSENS_COUNT];

float xsens_mag_x[XSENS_COUNT];
float xsens_mag_y[XSENS_COUNT];
float xsens_mag_z[XSENS_COUNT];

float xsens_gyro_x[XSENS_COUNT];
float xsens_gyro_y[XSENS_COUNT];
float xsens_gyro_z[XSENS_COUNT];




float xsens_mag_heading[XSENS_COUNT];

int xsens_setzero = 0;

#define XSENS_MSG_BUF 1
#define XSENS_MAX_PAYLOAD 254
uint8_t xsens_msg_buf[XSENS_COUNT][XSENS_MSG_BUF][XSENS_MAX_PAYLOAD];
static volatile uint8_t xsens_msg_buf_count[XSENS_COUNT]; // buffer count
static volatile uint8_t xsens_msg_buf_pi[XSENS_COUNT]; // produce index
static volatile uint8_t xsens_msg_buf_ci[XSENS_COUNT]; // consume index

#define XSENS_RAW_MODE

// See Page 25 MT Low-Level Comm Doc
#define XSENS_OUTPUT_CALIBRATED (1 << 1)
#define XSENS_OUTPUT_ORIENTATION (1 << 2)
#define XSENS_OUTPUT_AUXDATA (1 << 3)
#define XSENS_OUTPUT_STATUS (1 << 11)
#define XSENS_RAW_INERTIAL (1 << 14)
#ifdef XSENS_RAW_MODE
#define XSENS_DEFAULT_OUTPUT_MODE (XSENS_RAW_INERTIAL | XSENS_OUTPUT_STATUS)
#else
#define XSENS_DEFAULT_OUTPUT_MODE (XSENS_OUTPUT_ORIENTATION | XSENS_OUTPUT_STATUS | XSENS_OUTPUT_CALIBRATED)
#endif

// output settings : (Page 26 MT Low-Level Comm Doc)
// Sample Counter, rotation matrix, floating point output
#define XSENS_OUTPUT_AUX1 (1 << 10)
#define XSENS_OUTPUT_AUX2 (1 << 11)
#define XSENS_ORIENTATION (2 << 2)
#define XSENS_ACCELS (1 << 4)
#define XSENS_GYROS (1 << 5)
#define XSENS_MAGS (1 << 6)
#define XSENS_TIMESTAMP (1 << 0)
#ifdef XSENS_RAW_MODE
#define XSENS_DEFAULT_OUTPUT_SETTINGS (XSENS_ACCELS | XSENS_MAGS | XSENS_GYROS | XSENS_TIMESTAMP)
#else
#define XSENS_DEFAULT_OUTPUT_SETTINGS (XSENS_ORIENTATION | XSENS_ACCELS | XSENS_MAGS | XSENS_GYROS | XSENS_TIMESTAMP)
#endif

#define UNINIT        0
#define GOT_START     1
#define GOT_BID       2
#define GOT_MID       3
#define GOT_LEN       4
#define GOT_DATA      5
#define GOT_CHECKSUM  6

uint8_t xsens_errorcode[XSENS_COUNT];
uint8_t xsens_msg_status[XSENS_COUNT];
uint16_t xsens_time_stamp[XSENS_COUNT];
uint16_t xsens_output_mode[XSENS_COUNT];
uint32_t xsens_output_settings[XSENS_COUNT];

static uint8_t msg_id[XSENS_COUNT][XSENS_MSG_BUF];
static uint8_t xsens_len[XSENS_COUNT][XSENS_MSG_BUF];
static uint8_t xsens_msg_idx[XSENS_COUNT];
static uint8_t ck[XSENS_COUNT];
static uint8_t send_ck[XSENS_COUNT];

void booz_imu_impl_init( void  )
{

}

void xsens_init( void )
{
  for (int i = 0; i < XSENS_COUNT; i++) {
    send_ck[i] = 0;
    xsens_msg_status[i] = 0;
    xsens_time_stamp[i] = 0;
    xsens_output_mode[i] = XSENS_DEFAULT_OUTPUT_MODE;
    xsens_output_settings[i] = XSENS_DEFAULT_OUTPUT_SETTINGS;
    xsens_msg_buf_count[i] = 0;
    xsens_msg_buf_pi[i] = 0;
    xsens_msg_buf_ci[i] = 0;
    FLOAT_RMAT_ZERO(xsens_rmat_neutral[i]);
  }
  booz_imu.accel_neutral.x = IMU_ACCEL_X_NEUTRAL;
  booz_imu.accel_neutral.y = IMU_ACCEL_Y_NEUTRAL;
  booz_imu.accel_neutral.z = IMU_ACCEL_Z_NEUTRAL;

  booz_imu.gyro_neutral.p = IMU_GYRO_P_NEUTRAL;
  booz_imu.gyro_neutral.q = IMU_GYRO_Q_NEUTRAL;
  booz_imu.gyro_neutral.r = IMU_GYRO_R_NEUTRAL;

  booz_imu.mag_neutral.x = IMU_MAG_X_NEUTRAL;
  booz_imu.mag_neutral.y = IMU_MAG_Y_NEUTRAL;
  booz_imu.mag_neutral.z = IMU_MAG_Z_NEUTRAL;
  // Also TODO: set scenario to aerospace
  // set magnetic declination angle
  // Probably quicker to just set everything once via MT Manager software
  // instead of setting via paparazzi (MT-G should store setting in flash)
  //XSENS_GoToConfig();
  //XSENS_SetOutputMode(xsens2_output_mode);
  //XSENS_SetOutputSettings(xsens2_output_settings);
  //XSENS_GoToMeasurment();
}

void xsens_periodic_task ( void )
{
  for (int i = 0; i < XSENS_COUNT; i++)
  {
    if (xsens_msg_buf_count[i] > 0) {
      xsens_parse_msg(i);
      xsens_msg_buf_count[i]--;
      xsens_msg_buf_ci[i] = (xsens_msg_buf_ci[i] + 1) % XSENS_MSG_BUF;
    }
    if (xsens_setzero) {
      memcpy(&xsens_rmat_neutral[i], &xsens_rmat[i], sizeof(struct FloatRMat));
    }
  }
  xsens_setzero = 0;
}

void xsens_event_task( void ) 
{
  while (Xsens1Link(ChAvailable()) && !xsens_msg_received[0]) {
    parse_xsens_msg(0, Xsens1Link(Getch()));
  }

/*
  while (Xsens2Link(ChAvailable()) && !xsens_msg_received[1])
    parse_xsens_msg(1, Xsens2Link(Getch()));
*/

  for (int i = 0; i < XSENS_COUNT; i++) {
    if (xsens_msg_received[i]) {

      // first make room
      while (xsens_msg_buf_count[i] >= XSENS_MSG_BUF) {
        // throwing away old stuff
        xsens_msg_buf_ci[i] = (xsens_msg_buf_ci[i] + 1 ) % XSENS_MSG_BUF;
        xsens_msg_buf_count[i]--;
      }

      xsens_msg_buf_pi[i] = (xsens_msg_buf_pi[i] + 1 ) % XSENS_MSG_BUF;
      xsens_msg_buf_count[i]++;

      //xsens_parse_msg(i);
      xsens_msg_received[i] = FALSE;
    }
  }
}

// Called after receipt of valid message to 
void xsens_parse_msg( uint8_t xsens_id ) {
  uint8_t buf_slot = xsens_msg_buf_ci[xsens_id];

  if (msg_id[xsens_id][buf_slot] == XSENS_ReqOutputModeAck_ID) {
    xsens_output_mode[xsens_id] = XSENS_ReqOutputModeAck_mode(xsens_msg_buf[xsens_id]);
  }
  else if (msg_id[xsens_id][buf_slot] == XSENS_Error_ID) {
    xsens_errorcode[xsens_id] = XSENS_Error_errorcode(xsens_msg_buf[xsens_id]);
  }
  else if (msg_id[xsens_id][buf_slot] == XSENS_MTData_ID) {
    uint8_t offset = 0;
    // test RAW modes else calibrated modes 
      if (XSENS_MASK_RAWInertial(xsens_output_mode[xsens_id])){// || (XSENS_MASK_RAWGPS(xsens2_output_mode))) {
	booz_imu.accel_unscaled.x = XSENS_DATA_RAWInertial_accZ(xsens_msg_buf[xsens_id][buf_slot],offset);
	booz_imu.accel_unscaled.y = XSENS_DATA_RAWInertial_accY(xsens_msg_buf[xsens_id][buf_slot],offset);
	booz_imu.accel_unscaled.z = XSENS_DATA_RAWInertial_accX(xsens_msg_buf[xsens_id][buf_slot],offset);
	booz_imu.gyro_unscaled.p = XSENS_DATA_RAWInertial_gyrZ(xsens_msg_buf[xsens_id][buf_slot],offset);
	booz_imu.gyro_unscaled.q = XSENS_DATA_RAWInertial_gyrY(xsens_msg_buf[xsens_id][buf_slot],offset);
	booz_imu.gyro_unscaled.r = XSENS_DATA_RAWInertial_gyrX(xsens_msg_buf[xsens_id][buf_slot],offset);
        booz_imu.mag_unscaled.x  = XSENS_DATA_RAWInertial_magZ(xsens_msg_buf[xsens_id][buf_slot],offset);
	booz_imu.mag_unscaled.y  = XSENS_DATA_RAWInertial_magY(xsens_msg_buf[xsens_id][buf_slot],offset);
	booz_imu.mag_unscaled.z  = XSENS_DATA_RAWInertial_magX(xsens_msg_buf[xsens_id][buf_slot],offset);
	BoozImuScaleGyro(booz_imu);
	BoozImuScaleAccel(booz_imu);
	BoozImuScaleMag(booz_imu);
	
	// Copied from booz2_main -- 5143134f060fcc57ce657e17d8b7fc2e72119fd7
	// mmt 6/15/09
	if (booz_ahrs.status == BOOZ_AHRS_UNINIT) {
	  // 150
	  booz_ahrs_aligner_run();
	  if (booz_ahrs_aligner.status == BOOZ_AHRS_ALIGNER_LOCKED)
	    booz_ahrs_align();
	}
	else {
	  booz_ahrs_propagate();
	  booz_ins_propagate();
	}
      }
      if (XSENS_MASK_Temp(xsens_output_mode[xsens_id])) {
        offset += XSENS_DATA_Temp_LENGTH;
      }
      if (XSENS_MASK_Calibrated(xsens_output_mode[xsens_id])) {
        if (XSENS_MASK_AccOut(xsens_output_settings[xsens_id])) {
          xsens_accel_x[xsens_id] = XSENS_DATA_Calibrated_accX(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_accel_y[xsens_id] = XSENS_DATA_Calibrated_accY(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_accel_z[xsens_id] = XSENS_DATA_Calibrated_accZ(xsens_msg_buf[xsens_id][buf_slot],offset);
        }
        if (XSENS_MASK_GyrOut(xsens_output_settings[xsens_id])) {
          xsens_gyro_x[xsens_id] = XSENS_DATA_Calibrated_gyrX(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_gyro_y[xsens_id] = XSENS_DATA_Calibrated_gyrY(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_gyro_z[xsens_id] = XSENS_DATA_Calibrated_gyrZ(xsens_msg_buf[xsens_id][buf_slot],offset);
        }
        if (XSENS_MASK_MagOut(xsens_output_settings[xsens_id])) {
          xsens_mag_x[xsens_id] = XSENS_DATA_Calibrated_magX(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_mag_y[xsens_id] = XSENS_DATA_Calibrated_magY(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_mag_z[xsens_id] = XSENS_DATA_Calibrated_magZ(xsens_msg_buf[xsens_id][buf_slot],offset);
          float pitch = xsens_phi[xsens_id];
          float roll = -xsens_theta[xsens_id];
          float tilt_comp_x = xsens_mag_x[xsens_id] * cos(pitch)
                            + xsens_mag_y[xsens_id] * sin(roll) * sin(pitch)
                            - xsens_mag_z[xsens_id] * cos(roll) * sin(pitch);
          float tilt_comp_y = xsens_mag_y[xsens_id] * cos(roll) 
                            + xsens_mag_z[xsens_id] * sin(roll);
          xsens_mag_heading[xsens_id] = -atan2( tilt_comp_y, tilt_comp_x);
        }
        offset += XSENS_DATA_Calibrated_LENGTH;
      }
      if (XSENS_MASK_Orientation(xsens_output_mode[xsens_id])) {
        if (XSENS_MASK_OrientationMode(xsens_output_settings[xsens_id]) == 0x0) {
          offset += XSENS_DATA_Quaternion_LENGTH;
        }
        if (XSENS_MASK_OrientationMode(xsens_output_settings[xsens_id]) == 0x1) {
          xsens_phi[xsens_id] = XSENS_DATA_Euler_roll(xsens_msg_buf[xsens_id][buf_slot],offset)  * M_PI/180;
          xsens_theta[xsens_id] =XSENS_DATA_Euler_pitch(xsens_msg_buf[xsens_id][buf_slot],offset) * M_PI/180;
          xsens_psi[xsens_id] =  XSENS_DATA_Euler_yaw(xsens_msg_buf[xsens_id][buf_slot],offset)   * M_PI/180;
          offset += XSENS_DATA_Euler_LENGTH;
        }
        if (XSENS_MASK_OrientationMode(xsens_output_settings[xsens_id]) == 0x2) {
          xsens_rmat[xsens_id].m[0] = XSENS_DATA_Matrix_a(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_rmat[xsens_id].m[1] = XSENS_DATA_Matrix_b(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_rmat[xsens_id].m[2] = XSENS_DATA_Matrix_c(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_rmat[xsens_id].m[3] = XSENS_DATA_Matrix_d(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_rmat[xsens_id].m[4] = XSENS_DATA_Matrix_e(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_rmat[xsens_id].m[5] = XSENS_DATA_Matrix_f(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_rmat[xsens_id].m[6] = XSENS_DATA_Matrix_g(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_rmat[xsens_id].m[7] = XSENS_DATA_Matrix_h(xsens_msg_buf[xsens_id][buf_slot],offset);
          xsens_rmat[xsens_id].m[8] = XSENS_DATA_Matrix_i(xsens_msg_buf[xsens_id][buf_slot],offset);
	  
	  FLOAT_RMAT_COMP_INV(xsens_rmat_adj[xsens_id], xsens_rmat_neutral[xsens_id], xsens_rmat[xsens_id]);

          // Calculate roll, pitch, yaw from rotation matrix ( p 31-33 MTi-G USer Man and Tech Doc)
          xsens_phi[xsens_id] = -atan2 (xsens_rmat_adj[xsens_id].m[7], xsens_rmat_adj[xsens_id].m[8]);
          xsens_theta[xsens_id] = asin (xsens_rmat_adj[xsens_id].m[6]);
          xsens_psi[xsens_id] = atan2 (xsens_rmat_adj[xsens_id].m[3], xsens_rmat_adj[xsens_id].m[0]);

          offset += XSENS_DATA_Matrix_LENGTH;
        }
      }
      if (XSENS_MASK_Status(xsens_output_mode[xsens_id])) {
        xsens_msg_status[xsens_id] = XSENS_DATA_Status_status(xsens_msg_buf[xsens_id][buf_slot],offset);
        xsens_mode[xsens_id] = xsens_msg_status[xsens_id];
        offset += XSENS_DATA_Status_LENGTH;
      }
      if (XSENS_MASK_TimeStamp(xsens_output_settings[xsens_id])) {
        uint16_t ts = XSENS_DATA_TimeStamp_ts(xsens_msg_buf[xsens_id][buf_slot],offset);
        if (xsens_time_stamp[xsens_id] + 1 != ts)
          //xsens_err_count[xsens_id]++;
        xsens_time_stamp[xsens_id] = ts;
        offset += XSENS_DATA_TimeStamp_LENGTH;
      }
  }
}


// Simple state machine parser for XSENS messages (MT Low-Level Comm Doc)
// Passed serial bytes one per call and parses stream into message id, data length, and data buffer
// for use at higher level
void parse_xsens_msg( uint8_t xsens_id, uint8_t c ) {
  static uint8_t xsens_status[XSENS_COUNT];
  uint8_t buf_slot = xsens_msg_buf_pi[xsens_id]; // produce index

  // keep track of checksum byte
  ck[xsens_id] += c;
  switch (xsens_status[xsens_id]) {
  case UNINIT:
    // Look for xsens start byte 
    if (c != XSENS_START)
      goto error;
    xsens_status[xsens_id]++; 
    ck[xsens_id] = 0; // Reset checksum
    break;
  case GOT_START:
    // Look for xsens Bus ID
    if (c != XSENS_BID)
      goto error;
    xsens_status[xsens_id]++;
    break;
  case GOT_BID:
    // Save message ID
    msg_id[xsens_id][buf_slot] = c;
    xsens_status[xsens_id]++;
    break;
  case GOT_MID:
    // Save message length
    xsens_len[xsens_id][buf_slot] = c;
    // check for valid message length 
    if (xsens_len[xsens_id][buf_slot] > XSENS_MAX_PAYLOAD)
      goto error;
    xsens_msg_idx[xsens_id] = 0; // Reset buffer index
    xsens_status[xsens_id]++;
    break;
  case GOT_LEN:
    // Read byte into data buffer
    xsens_msg_buf[xsens_id][buf_slot][xsens_msg_idx[xsens_id]] = c;
    xsens_msg_idx[xsens_id]++;
    // Terminate reading at end of data field
    if (xsens_msg_idx[xsens_id] >= xsens_len[xsens_id][buf_slot])
      xsens_status[xsens_id]++;
    break;
  case GOT_DATA:
    // Check for valid checksum
    if (ck[xsens_id] != 0) {
      //xsens_err_count[xsens_id]++;
      goto error;
    }
    // Notification for valid message received
    xsens_msg_received[xsens_id] = TRUE;
    //xsens_recv_count[xsens_id]++;
    goto restart;
    break;
  }
  return;
 error:  
 restart:
  // Start over (Reset parser state)
  xsens_status[xsens_id] = UNINIT;
  return;
}
