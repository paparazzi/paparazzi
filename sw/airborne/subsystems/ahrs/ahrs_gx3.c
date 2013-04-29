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
#include "mcu_periph/sys_time.h"

#define GX3_TIME(_ubx_payload) (uint32_t)((uint32_t)(*((uint8_t*)_ubx_payload+62+3))|(uint32_t)(*((uint8_t*)_ubx_payload+62+2))<<8|(uint32_t)(*((uint8_t*)_ubx_payload+62+1))<<16|(uint32_t)(*((uint8_t*)_ubx_payload+62+0))<<24)
#define GX3_CHKSM(_ubx_payload) (uint16_t)((uint16_t)(*((uint8_t*)_ubx_payload+66+1))|(uint16_t)(*((uint8_t*)_ubx_payload+66+0))<<8)

/*
 * Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
 * Positive pitch : nose up
 * Positive roll : right wing down
 * Positive yaw : clockwise
 */
struct GX3_packet GX3_packet;
enum GX3Status GX3_status;
uint32_t GX3_time;
uint32_t GX3_ltime;
uint16_t GX3_chksm;
uint16_t GX3_calcsm;
float GX3_freq;

struct FloatVect3 GX3_accel;
struct FloatRates GX3_rate;
struct FloatRMat  GX3_rmat;
struct FloatQuat GX3_quat;
struct FloatEulers GX3_euler;

struct AhrsFloatQuat ahrs_impl;
struct AhrsAligner ahrs_aligner;

static inline bool_t GX3_verify_chk(volatile uint8_t *buff_add);
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

static inline bool_t GX3_verify_chk(volatile uint8_t *buff_add) {
  uint16_t i,chk_calc;
  chk_calc = 0;
  for (i=0;i<GX3_MSG_LEN-2;i++) {
    chk_calc += (uint8_t)*buff_add++;
  }
  return (chk_calc == ( (((uint16_t)*buff_add)<<8) + (uint8_t)*(buff_add+1) ));
}

void ahrs_align(void) {
  GX3_status = GX3Uninit;

  //make the gyros zero, takes 10s (specified in Byte 4 and 5)
  GX3Link(Transmit(0xcd));
  GX3Link(Transmit(0xc1));
  GX3Link(Transmit(0x29));
  GX3Link(Transmit(0x27));
  GX3Link(Transmit(0x10));

  GX3_status = GX3Running;
}


void imu_impl_init(void) {
  // Initialize variables
  GX3_status = GX3Uninit;

  // Initialize packet
  GX3_packet.status = GX3PacketWaiting;
  GX3_packet.msg_idx = 0;
  GX3_packet.msg_available = FALSE;
  GX3_packet.chksm_error = 0;
  GX3_packet.hdr_error = 0;

  // It is necessary to wait for GX3 to power up for proper initialization
  for (uint32_t startup_counter=0; startup_counter<IMU_GX3_LONG_DELAY*2; startup_counter++){
    __asm("nop");
  }

  /*
  // FOR NON-CONTINUOUS MODE UNCOMMENT THIS
  //4 byte command for non-Continous Mode so we can set the other settings
  GX3Link(Transmit(0xc4));
  GX3Link(Transmit(0xc1));
  GX3Link(Transmit(0x29));
  GX3Link(Transmit(0x00)); // stop
  */

  //Sampling Settings (0xDB)
  GX3Link(Transmit(0xdb)); //set update speed
  GX3Link(Transmit(0xa8));
  GX3Link(Transmit(0xb9));
  //set rate of IMU link, is 1000/IMU_DIV
#define IMU_DIV1 0
#define IMU_DIV2 2
#define ACC_FILT_DIV 2
#define MAG_FILT_DIV 30
  GX3Link(Transmit(0x01));//set params, don't store
  GX3Link(Transmit(IMU_DIV1));
  GX3Link(Transmit(IMU_DIV2));
  GX3Link(Transmit(0b00000000));  //set options byte 8 - GOOD
  GX3Link(Transmit(0b00000011));  //set options byte 7 - GOOD
  //0 - calculate orientation, 1 - enable coning & sculling, 2-3 reserved, 4 - no little endian data,
  // 5 - no NaN supressed, 6 - disable finite size correction, 7 - reserved,
  // 8 - enable magnetometer, 9 - reserved, 10 - enable magnetic north compensation, 11 - enable gravity compensation
  // 12 - no quaternion calculation, 13-15 reserved
  GX3Link(Transmit(ACC_FILT_DIV));
  GX3Link(Transmit(MAG_FILT_DIV)); //mag window filter size == 33hz
  GX3Link(Transmit(0x00));
  GX3Link(Transmit(10)); // Up Compensation in secs, def=10s
  GX3Link(Transmit(0x00));
  GX3Link(Transmit(10)); // North Compensation in secs
  GX3Link(Transmit(0x00)); //power setting = 0, high power/bw
  GX3Link(Transmit(0x00)); //rest of the bytes are 0
  GX3Link(Transmit(0x00));
  GX3Link(Transmit(0x00));
  GX3Link(Transmit(0x00));
  GX3Link(Transmit(0x00));

  // OPTIONAL: realign up and north
  /*
    GX3Link(Transmit(0xdd));
    GX3Link(Transmit(0x54));
    GX3Link(Transmit(0x4c));
    GX3Link(Transmit(3));
    GX3Link(Transmit(10));
    GX3Link(Transmit(10));
    GX3Link(Transmit(0x00));
    GX3Link(Transmit(0x00));
    GX3Link(Transmit(0x00));
    GX3Link(Transmit(0x00));
  */

  // Another wait loop for proper GX3 init
  for (uint32_t startup_counter=0; startup_counter<IMU_GX3_LONG_DELAY; startup_counter++){
    __asm("nop");
  }

  //4 byte command for Continous Mode
  GX3Link(Transmit(0xc4));
  GX3Link(Transmit(0xc1));
  GX3Link(Transmit(0x29));
  GX3Link(Transmit(0xc8)); // accel,gyro,R

  // Reset gyros to zerp
  ahrs_align();
}


void imu_periodic(void) {
  /* IF IN NON-CONTINUOUS MODE, REQUEST DATA NOW
     GX3Link(Transmit(0xc8)); // accel,gyro,R
  */
}


void GX3_packet_read_message(void) {
  GX3_accel.x     = bef(&GX3_packet.msg_buf[1]);
  GX3_accel.y     = bef(&GX3_packet.msg_buf[5]);
  GX3_accel.z     = bef(&GX3_packet.msg_buf[9]);
  GX3_rate.p      = bef(&GX3_packet.msg_buf[13]);
  GX3_rate.q      = bef(&GX3_packet.msg_buf[17]);
  GX3_rate.r      = bef(&GX3_packet.msg_buf[21]);
  GX3_rmat.m[0]   = bef(&GX3_packet.msg_buf[25]);
  GX3_rmat.m[1]   = bef(&GX3_packet.msg_buf[29]);
  GX3_rmat.m[2]   = bef(&GX3_packet.msg_buf[33]);
  GX3_rmat.m[3]   = bef(&GX3_packet.msg_buf[37]);
  GX3_rmat.m[4]   = bef(&GX3_packet.msg_buf[41]);
  GX3_rmat.m[5]   = bef(&GX3_packet.msg_buf[45]);
  GX3_rmat.m[6]   = bef(&GX3_packet.msg_buf[49]);
  GX3_rmat.m[7]   = bef(&GX3_packet.msg_buf[53]);
  GX3_rmat.m[8]   = bef(&GX3_packet.msg_buf[57]);
  GX3_time  = GX3_TIME(GX3_packet.msg_buf);
  GX3_chksm	= GX3_CHKSM(GX3_packet.msg_buf);
  GX3_calcsm = 0;

  GX3_freq = ((GX3_time - GX3_ltime))/16000000.0;
  GX3_freq = 1.0/GX3_freq;
  GX3_ltime = GX3_time;

  // Acceleration
  VECT3_SMUL(GX3_accel, GX3_accel, 9.80665); // Convert g into m/s2
  ACCELS_BFP_OF_REAL(imu.accel, GX3_accel);
  imuf.accel = GX3_accel;

  // Rates
  struct FloatRates body_rate;
  ahrs_impl.imu_rate = GX3_rate;
  /* compute body rates */
  FLOAT_RMAT_TRANSP_RATEMULT(body_rate, imuf.body_to_imu_rmat, ahrs_impl.imu_rate);
  /* Set state */
  stateSetBodyRates_f(&body_rate);

  // Quaternions from rotation matrix
  FLOAT_QUAT_OF_RMAT(GX3_quat, GX3_rmat);
  ahrs_impl.ltp_to_imu_quat = GX3_quat;
  /* Compute LTP to BODY quaternion */
  struct FloatQuat ltp_to_body_quat;
  FLOAT_QUAT_COMP_INV(ltp_to_body_quat, ahrs_impl.ltp_to_imu_quat, imuf.body_to_imu_quat);
  stateSetNedToBodyQuat_f(&ltp_to_body_quat);

  // TODO: compensate for magnetic offset
}


/* GX3 Packet Collection */
void GX3_packet_parse( uint8_t c ) {
  switch (GX3_packet.status) {
    case GX3PacketWaiting:
      GX3_packet.msg_idx = 0;
      if (c == GX3_HEADER) {
        GX3_packet.status++;
        GX3_packet.msg_buf[GX3_packet.msg_idx] = c;
        GX3_packet.msg_idx++;
      } else {
        GX3_packet.hdr_error++;
      }
      break;
    case GX3PacketReading:
      GX3_packet.msg_buf[GX3_packet.msg_idx] =  c;
      GX3_packet.msg_idx++;
      if (GX3_packet.msg_idx == GX3_MSG_LEN) {
        if (GX3_verify_chk(GX3_packet.msg_buf)) {
          GX3_packet.msg_available = TRUE;
        } else {
          GX3_packet.msg_available = FALSE;
          GX3_packet.chksm_error++;
        }
        GX3_packet.status = 0;
      }
      break;
    default:
      GX3_packet.status = 0;
      GX3_packet.msg_idx = 0;
      break;
  }
}

void ahrs_init(void) {
  ahrs.status = AHRS_UNINIT;

  /* set ltp_to_imu so that body is zero */
  QUAT_COPY(ahrs_impl.ltp_to_imu_quat, imuf.body_to_imu_quat);
  FLOAT_RATES_ZERO(ahrs_impl.imu_rate);

#ifdef IMU_MAG_OFFSET
  ahrs_impl.mag_offset = IMU_MAG_OFFSET;
#else
  ahrs_impl.mag_offset = 0.;
#endif

  ahrs_aligner.status = AHRS_ALIGNER_LOCKED;
}

void ahrs_aligner_run(void) {
#ifdef AHRS_ALIGNER_LED
  LED_TOGGLE(AHRS_ALIGNER_LED);
#endif

  if (GX3_freq > GX3_MIN_FREQ) {
    ahrs.status = AHRS_RUNNING;
#ifdef AHRS_ALIGNER_LED
    LED_ON(AHRS_ALIGNER_LED);
#endif
  }
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
