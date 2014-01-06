/*
 * Copyright (C) 2013 Michal Podhradsky
 * Utah State University, http://aggieair.usu.edu/
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
 * @file ahrs_gx3.c
 *
 * Driver for Microstrain GX3 IMU/AHRS subsystem
 *
 * Takes care of configuration of the IMU, communication and parsing
 * the received packets. See GX3 datasheet for configuration options.
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include "subsystems/ahrs/ahrs_gx3.h"

#ifdef AHRS_UPDATE_FW_ESTIMATOR
// remotely settable
#ifndef INS_ROLL_NEUTRAL_DEFAULT
#define INS_ROLL_NEUTRAL_DEFAULT 0
#endif
#ifndef INS_PITCH_NEUTRAL_DEFAULT
#define INS_PITCH_NEUTRAL_DEFAULT 0
#endif
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
#endif

#define GX3_CHKSM(_ubx_payload) (uint16_t)((uint16_t)(*((uint8_t*)_ubx_payload+66+1))|(uint16_t)(*((uint8_t*)_ubx_payload+66+0))<<8)

/*
 * Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
 * Positive pitch : nose up
 * Positive roll : right wing down
 * Positive yaw : clockwise
 */
struct AhrsFloatQuat ahrs_impl;
struct AhrsAligner ahrs_aligner;

static inline bool_t gx3_verify_chk(volatile uint8_t *buff_add);
static inline float bef(volatile uint8_t *c);

/* Big Endian to Float */
static inline float bef(volatile uint8_t *c) {
  float f;
  int8_t * p;
  p = ((int8_t *)&f)+3;
  *p-- = *c++;
  *p-- = *c++;
  *p-- = *c++;
  *p = *c;
  return f;
}

static inline bool_t gx3_verify_chk(volatile uint8_t *buff_add) {
  uint16_t i,chk_calc;
  chk_calc = 0;
  for (i=0;i<GX3_MSG_LEN-2;i++) {
    chk_calc += (uint8_t)*buff_add++;
  }
  return (chk_calc == ( (((uint16_t)*buff_add)<<8) + (uint8_t)*(buff_add+1) ));
}

void ahrs_align(void) {
  ahrs_impl.gx3_status = GX3Uninit;

  //make the gyros zero, takes 10s (specified in Byte 4 and 5)
  uart_transmit(&GX3_PORT, 0xcd);
  uart_transmit(&GX3_PORT, 0xc1);
  uart_transmit(&GX3_PORT, 0x29);
  uart_transmit(&GX3_PORT, 0x27);
  uart_transmit(&GX3_PORT, 0x10);

  ahrs_impl.gx3_status = GX3Running;
}

#if DOWNLINK
#include "subsystems/datalink/telemetry.h"

static send_gx3(void) {
  DOWNLINK_SEND_GX3_INFO(DefaultChannel, DefaultDevice,
      &ahrs_impl.gx3_freq,
      &ahrs_impl.gx3_packet.chksm_error,
      &ahrs_impl.gx3_packet.hdr_error,
      &ahrs_impl.gx3_chksm);
}
#endif

/*
 * GX3 can be set up during the startup, or it can be configured to
 * start sending data automatically after power up.
 */
void imu_impl_init(void) {
  // Initialize variables
  ahrs_impl.gx3_status = GX3Uninit;

  // Initialize packet
  ahrs_impl.gx3_packet.status = GX3PacketWaiting;
  ahrs_impl.gx3_packet.msg_idx = 0;
  ahrs_impl.gx3_packet.msg_available = FALSE;
  ahrs_impl.gx3_packet.chksm_error = 0;
  ahrs_impl.gx3_packet.hdr_error = 0;

  // It is necessary to wait for GX3 to power up for proper initialization
  for (uint32_t startup_counter=0; startup_counter<IMU_GX3_LONG_DELAY*2; startup_counter++){
    __asm("nop");
  }

#ifdef GX3_INITIALIZE_DURING_STARTUP
#pragma message "GX3 initializing"
  /*
  // FOR NON-CONTINUOUS MODE UNCOMMENT THIS
  //4 byte command for non-Continous Mode so we can set the other settings
  uart_transmit(&GX3_PORT, 0xc4);
  uart_transmit(&GX3_PORT, 0xc1);
  uart_transmit(&GX3_PORT, 0x29);
  uart_transmit(&GX3_PORT, 0x00); // stop
  */

  //Sampling Settings (0xDB)
  uart_transmit(&GX3_PORT, 0xdb); //set update speed
  uart_transmit(&GX3_PORT, 0xa8);
  uart_transmit(&GX3_PORT, 0xb9);
  //set rate of IMU link, is 1000/IMU_DIV
#define IMU_DIV1 0
#define IMU_DIV2 2
#define ACC_FILT_DIV 2
#define MAG_FILT_DIV 30
#ifdef GX3_SAVE_SETTINGS
  uart_transmit(&GX3_PORT, 0x02);//set params and save them in non-volatile memory
#else
  uart_transmit(&GX3_PORT, 0x02); //set and don't save
#endif
  uart_transmit(&GX3_PORT, IMU_DIV1);
  uart_transmit(&GX3_PORT, IMU_DIV2);
  uart_transmit(&GX3_PORT, 0b00000000);  //set options byte 8 - GOOD
  uart_transmit(&GX3_PORT, 0b00000011);  //set options byte 7 - GOOD
  //0 - calculate orientation, 1 - enable coning & sculling, 2-3 reserved, 4 - no little endian data,
  // 5 - no NaN supressed, 6 - disable finite size correction, 7 - reserved,
  // 8  - enable magnetometer, 9 - reserved, 10 - enable magnetic north compensation, 11 - enable gravity compensation
  // 12 - no quaternion calculation, 13-15 reserved
  uart_transmit(&GX3_PORT, ACC_FILT_DIV);
  uart_transmit(&GX3_PORT, MAG_FILT_DIV); //mag window filter size == 33hz
  uart_transmit(&GX3_PORT, 0x00);
  uart_transmit(&GX3_PORT, 10); // Up Compensation in secs, def=10s
  uart_transmit(&GX3_PORT, 0x00);
  uart_transmit(&GX3_PORT, 10); // North Compensation in secs
  uart_transmit(&GX3_PORT, 0x00); //power setting = 0, high power/bw
  uart_transmit(&GX3_PORT, 0x00); //rest of the bytes are 0
  uart_transmit(&GX3_PORT, 0x00);
  uart_transmit(&GX3_PORT, 0x00);
  uart_transmit(&GX3_PORT, 0x00);
  uart_transmit(&GX3_PORT, 0x00);

  // OPTIONAL: realign up and north
  /*
    uart_transmit(&GX3_PORT, 0xdd);
    uart_transmit(&GX3_PORT, 0x54);
    uart_transmit(&GX3_PORT, 0x4c);
    uart_transmit(&GX3_PORT, 3);
    uart_transmit(&GX3_PORT, 10);
    uart_transmit(&GX3_PORT, 10);
    uart_transmit(&GX3_PORT, 0x00);
    uart_transmit(&GX3_PORT, 0x00);
    uart_transmit(&GX3_PORT, 0x00);
    uart_transmit(&GX3_PORT, 0x00);
  */

  //Another wait loop for proper GX3 init
  for (uint32_t startup_counter=0; startup_counter<IMU_GX3_LONG_DELAY; startup_counter++){
    __asm("nop");
  }

#ifdef GX3_SET_WAKEUP_MODE
  //Mode Preset (0xD5)
  uart_transmit(&GX3_PORT, 0xD5);
  uart_transmit(&GX3_PORT, 0xBA);
  uart_transmit(&GX3_PORT, 0x89);
  uart_transmit(&GX3_PORT, 0x02); // wake up in continuous mode

  //Continuous preset (0xD6)
  uart_transmit(&GX3_PORT, 0xD6);
  uart_transmit(&GX3_PORT, 0xC6);
  uart_transmit(&GX3_PORT, 0x6B);
  uart_transmit(&GX3_PORT, 0xc8); // accel, gyro, R
#endif

  //4 byte command for Continous Mode
  uart_transmit(&GX3_PORT, 0xc4);
  uart_transmit(&GX3_PORT, 0xc1);
  uart_transmit(&GX3_PORT, 0x29);
  uart_transmit(&GX3_PORT, 0xc8); // accel,gyro,R

  // Reset gyros to zero
  ahrs_align();
#endif
  ahrs.status = AHRS_RUNNING;

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, "GX3_INFO", send_gx3);
#endif
}


void imu_periodic(void) {
  /* IF IN NON-CONTINUOUS MODE, REQUEST DATA NOW
     uart_transmit(&GX3_PORT, 0xc8); // accel,gyro,R
  */
}


void gx3_packet_read_message(void) {
  ahrs_impl.gx3_accel.x     = bef(&ahrs_impl.gx3_packet.msg_buf[1]);
  ahrs_impl.gx3_accel.y     = bef(&ahrs_impl.gx3_packet.msg_buf[5]);
  ahrs_impl.gx3_accel.z     = bef(&ahrs_impl.gx3_packet.msg_buf[9]);
  ahrs_impl.gx3_rate.p      = bef(&ahrs_impl.gx3_packet.msg_buf[13]);
  ahrs_impl.gx3_rate.q      = bef(&ahrs_impl.gx3_packet.msg_buf[17]);
  ahrs_impl.gx3_rate.r      = bef(&ahrs_impl.gx3_packet.msg_buf[21]);
  ahrs_impl.gx3_rmat.m[0]   = bef(&ahrs_impl.gx3_packet.msg_buf[25]);
  ahrs_impl.gx3_rmat.m[1]   = bef(&ahrs_impl.gx3_packet.msg_buf[29]);
  ahrs_impl.gx3_rmat.m[2]   = bef(&ahrs_impl.gx3_packet.msg_buf[33]);
  ahrs_impl.gx3_rmat.m[3]   = bef(&ahrs_impl.gx3_packet.msg_buf[37]);
  ahrs_impl.gx3_rmat.m[4]   = bef(&ahrs_impl.gx3_packet.msg_buf[41]);
  ahrs_impl.gx3_rmat.m[5]   = bef(&ahrs_impl.gx3_packet.msg_buf[45]);
  ahrs_impl.gx3_rmat.m[6]   = bef(&ahrs_impl.gx3_packet.msg_buf[49]);
  ahrs_impl.gx3_rmat.m[7]   = bef(&ahrs_impl.gx3_packet.msg_buf[53]);
  ahrs_impl.gx3_rmat.m[8]   = bef(&ahrs_impl.gx3_packet.msg_buf[57]);
  ahrs_impl.gx3_time    = (uint32_t)(ahrs_impl.gx3_packet.msg_buf[61] << 24 |
                                     ahrs_impl.gx3_packet.msg_buf[62] << 16 | ahrs_impl.gx3_packet.msg_buf[63] << 8 | ahrs_impl.gx3_packet.msg_buf[64]);
  ahrs_impl.gx3_chksm   = GX3_CHKSM(ahrs_impl.gx3_packet.msg_buf);

  ahrs_impl.gx3_freq = 62500.0 / (float)(ahrs_impl.gx3_time - ahrs_impl.gx3_ltime);
  ahrs_impl.gx3_ltime = ahrs_impl.gx3_time;

  // Acceleration
  VECT3_SMUL(ahrs_impl.gx3_accel, ahrs_impl.gx3_accel, 9.80665); // Convert g into m/s2
  ACCELS_BFP_OF_REAL(imu.accel, ahrs_impl.gx3_accel); // for backwards compatibility with fixed point interface
  imuf.accel = ahrs_impl.gx3_accel;

  // Rates
  struct FloatRates body_rate;
  imuf.gyro = ahrs_impl.gx3_rate;
  /* compute body rates */
  FLOAT_RMAT_TRANSP_RATEMULT(body_rate, imuf.body_to_imu_rmat, imuf.gyro);
  /* Set state */
  stateSetBodyRates_f(&body_rate);

  // Attitude
  struct FloatRMat ltp_to_body_rmat;
  FLOAT_RMAT_COMP(ltp_to_body_rmat, ahrs_impl.gx3_rmat, imuf.body_to_imu_rmat);
#ifdef AHRS_UPDATE_FW_ESTIMATOR // fixedwing
  struct FloatEulers ltp_to_body_eulers;
  FLOAT_EULERS_OF_RMAT(ltp_to_body_eulers, ltp_to_body_rmat);
  ltp_to_body_eulers.phi -= ins_roll_neutral;
  ltp_to_body_eulers.theta -= ins_pitch_neutral;
#if AHRS_USE_GPS_HEADING && USE_GPS
  float course_f = (float)DegOfRad(gps.course / 1e7);
  if (course_f > 180.0) {
    course_f -= 360.0;
  }
  ltp_to_body_eulers.psi = (float)RadOfDeg(course_f);
#endif
  stateSetNedToBodyEulers_f(&ltp_to_body_eulers);
#else
#ifdef IMU_MAG_OFFSET //rotorcraft
  struct FloatEulers ltp_to_body_eulers;
  FLOAT_EULERS_OF_RMAT(ltp_to_body_eulers, ltp_to_body_rmat);
  ltp_to_body_eulers.psi -= ahrs_impl.mag_offset;
  stateSetNedToBodyEulers_f(&ltp_to_body_eulers);
#else
  stateSetNedToBodyRMat_f(&ltp_to_body_rmat);
#endif
#endif
}


/* GX3 Packet Collection */
void gx3_packet_parse( uint8_t c ) {
  switch (ahrs_impl.gx3_packet.status) {
    case GX3PacketWaiting:
      ahrs_impl.gx3_packet.msg_idx = 0;
      if (c == GX3_HEADER) {
        ahrs_impl.gx3_packet.status++;
        ahrs_impl.gx3_packet.msg_buf[ahrs_impl.gx3_packet.msg_idx] = c;
        ahrs_impl.gx3_packet.msg_idx++;
      } else {
        ahrs_impl.gx3_packet.hdr_error++;
      }
      break;
    case GX3PacketReading:
      ahrs_impl.gx3_packet.msg_buf[ahrs_impl.gx3_packet.msg_idx] =  c;
      ahrs_impl.gx3_packet.msg_idx++;
      if (ahrs_impl.gx3_packet.msg_idx == GX3_MSG_LEN) {
        if (gx3_verify_chk(ahrs_impl.gx3_packet.msg_buf)) {
          ahrs_impl.gx3_packet.msg_available = TRUE;
        } else {
          ahrs_impl.gx3_packet.msg_available = FALSE;
          ahrs_impl.gx3_packet.chksm_error++;
        }
        ahrs_impl.gx3_packet.status = 0;
      }
      break;
    default:
      ahrs_impl.gx3_packet.status = 0;
      ahrs_impl.gx3_packet.msg_idx = 0;
      break;
  }
}

void ahrs_init(void) {
  ahrs.status = AHRS_UNINIT;
  /* set ltp_to_imu so that body is zero */
  QUAT_COPY(ahrs_impl.ltp_to_imu_quat, imuf.body_to_imu_quat);
#ifdef IMU_MAG_OFFSET
  ahrs_impl.mag_offset = IMU_MAG_OFFSET;
#else
  ahrs_impl.mag_offset = 0.0;
#endif
  ahrs_aligner.status = AHRS_ALIGNER_LOCKED;
}

void ahrs_aligner_run(void) {
#ifdef AHRS_ALIGNER_LED
  LED_ON(AHRS_ALIGNER_LED);
#endif
  ahrs.status = AHRS_RUNNING;
}


void ahrs_aligner_init(void) {
}

void ahrs_propagate(void) {
}

void ahrs_update_accel(void) {
}

void ahrs_update_mag(void) {
}

void ahrs_update_gps(void) {
}
