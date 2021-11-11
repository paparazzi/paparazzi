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

#include "modules/ahrs/ahrs_gx3.h"

// for ahrs_register_impl
#include "modules/ahrs/ahrs.h"
#include "modules/core/abi.h"

#define GX3_CHKSM(_ubx_payload) (uint16_t)((uint16_t)(*((uint8_t*)_ubx_payload+66+1))|(uint16_t)(*((uint8_t*)_ubx_payload+66+0))<<8)

/**
 * Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
 * Positive pitch : nose up
 * Positive roll : right wing down
 * Positive yaw : clockwise
 */
struct AhrsGX3 ahrs_gx3;

static inline bool gx3_verify_chk(volatile uint8_t *buff_add);
static inline float bef(volatile uint8_t *c);

/* Big Endian to Float */
static inline float bef(volatile uint8_t *c)
{
  float f;
  int8_t *p;
  p = ((int8_t *)&f) + 3;
  *p-- = *c++;
  *p-- = *c++;
  *p-- = *c++;
  *p = *c;
  return f;
}

static inline bool gx3_verify_chk(volatile uint8_t *buff_add)
{
  uint16_t i, chk_calc;
  chk_calc = 0;
  for (i = 0; i < GX3_MSG_LEN - 2; i++) {
    chk_calc += (uint8_t) * buff_add++;
  }
  return (chk_calc == ((((uint16_t) * buff_add) << 8) + (uint8_t) * (buff_add + 1)));
}

void ahrs_gx3_align(void)
{
  ahrs_gx3.is_aligned = false;

  //make the gyros zero, takes 10s (specified in Byte 4 and 5)
  uart_put_byte(&GX3_PORT, 0, 0xcd);
  uart_put_byte(&GX3_PORT, 0, 0xc1);
  uart_put_byte(&GX3_PORT, 0, 0x29);
  uart_put_byte(&GX3_PORT, 0, 0x27);
  uart_put_byte(&GX3_PORT, 0, 0x10);

  ahrs_gx3.is_aligned = true;
}

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_gx3(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GX3_INFO(trans, dev, AC_ID,
                         &ahrs_gx3.freq,
                         &ahrs_gx3.packet.chksm_error,
                         &ahrs_gx3.packet.hdr_error,
                         &ahrs_gx3.chksm);
}
#endif

/*
 * GX3 can be set up during the startup, or it can be configured to
 * start sending data automatically after power up.
 */
void imu_gx3_init(void)
{
  // Initialize variables
  ahrs_gx3.is_aligned = false;

  // Initialize packet
  ahrs_gx3.packet.status = GX3PacketWaiting;
  ahrs_gx3.packet.msg_idx = 0;
  ahrs_gx3.packet.msg_available = false;
  ahrs_gx3.packet.chksm_error = 0;
  ahrs_gx3.packet.hdr_error = 0;

  // It is necessary to wait for GX3 to power up for proper initialization
  for (uint32_t startup_counter = 0; startup_counter < IMU_GX3_LONG_DELAY * 2; startup_counter++) {
    __asm("nop");
  }

#ifdef GX3_INITIALIZE_DURING_STARTUP
#pragma message "GX3 initializing"
  /*
  // FOR NON-CONTINUOUS MODE UNCOMMENT THIS
  //4 byte command for non-Continous Mode so we can set the other settings
  uart_put_byte(&GX3_PORT, 0, 0xc4);
  uart_put_byte(&GX3_PORT, 0, 0xc1);
  uart_put_byte(&GX3_PORT, 0, 0x29);
  uart_put_byte(&GX3_PORT, 0, 0x00); // stop
  */

  //Sampling Settings (0xDB)
  uart_put_byte(&GX3_PORT, 0, 0xdb); //set update speed
  uart_put_byte(&GX3_PORT, 0, 0xa8);
  uart_put_byte(&GX3_PORT, 0, 0xb9);
  //set rate of IMU link, is 1000/IMU_DIV
#define IMU_DIV1 0
#define IMU_DIV2 2
#define ACC_FILT_DIV 2
#define MAG_FILT_DIV 30
#ifdef GX3_SAVE_SETTINGS
  uart_put_byte(&GX3_PORT, 0, 0x02);//set params and save them in non-volatile memory
#else
  uart_put_byte(&GX3_PORT, 0, 0x02); //set and don't save
#endif
  uart_put_byte(&GX3_PORT, 0, IMU_DIV1);
  uart_put_byte(&GX3_PORT, 0, IMU_DIV2);
  uart_put_byte(&GX3_PORT, 0, 0b00000000);  //set options byte 8 - GOOD
  uart_put_byte(&GX3_PORT, 0, 0b00000011);  //set options byte 7 - GOOD
  //0 - calculate orientation, 1 - enable coning & sculling, 2-3 reserved, 4 - no little endian data,
  // 5 - no NaN supressed, 6 - disable finite size correction, 7 - reserved,
  // 8  - enable magnetometer, 9 - reserved, 10 - enable magnetic north compensation, 11 - enable gravity compensation
  // 12 - no quaternion calculation, 13-15 reserved
  uart_put_byte(&GX3_PORT, 0, ACC_FILT_DIV);
  uart_put_byte(&GX3_PORT, 0, MAG_FILT_DIV); //mag window filter size == 33hz
  uart_put_byte(&GX3_PORT, 0, 0x00);
  uart_put_byte(&GX3_PORT, 0, 10); // Up Compensation in secs, def=10s
  uart_put_byte(&GX3_PORT, 0, 0x00);
  uart_put_byte(&GX3_PORT, 0, 10); // North Compensation in secs
  uart_put_byte(&GX3_PORT, 0, 0x00); //power setting = 0, high power/bw
  uart_put_byte(&GX3_PORT, 0, 0x00); //rest of the bytes are 0
  uart_put_byte(&GX3_PORT, 0, 0x00);
  uart_put_byte(&GX3_PORT, 0, 0x00);
  uart_put_byte(&GX3_PORT, 0, 0x00);
  uart_put_byte(&GX3_PORT, 0, 0x00);

  // OPTIONAL: realign up and north
  /*
    uart_put_byte(&GX3_PORT, 0, 0xdd);
    uart_put_byte(&GX3_PORT, 0, 0x54);
    uart_put_byte(&GX3_PORT, 0, 0x4c);
    uart_put_byte(&GX3_PORT, 0, 3);
    uart_put_byte(&GX3_PORT, 0, 10);
    uart_put_byte(&GX3_PORT, 0, 10);
    uart_put_byte(&GX3_PORT, 0, 0x00);
    uart_put_byte(&GX3_PORT, 0, 0x00);
    uart_put_byte(&GX3_PORT, 0, 0x00);
    uart_put_byte(&GX3_PORT, 0, 0x00);
  */

  //Another wait loop for proper GX3 init
  for (uint32_t startup_counter = 0; startup_counter < IMU_GX3_LONG_DELAY; startup_counter++) {
    __asm("nop");
  }

#ifdef GX3_SET_WAKEUP_MODE
  //Mode Preset (0xD5)
  uart_put_byte(&GX3_PORT, 0, 0xD5);
  uart_put_byte(&GX3_PORT, 0, 0xBA);
  uart_put_byte(&GX3_PORT, 0, 0x89);
  uart_put_byte(&GX3_PORT, 0, 0x02); // wake up in continuous mode

  //Continuous preset (0xD6)
  uart_put_byte(&GX3_PORT, 0, 0xD6);
  uart_put_byte(&GX3_PORT, 0, 0xC6);
  uart_put_byte(&GX3_PORT, 0, 0x6B);
  uart_put_byte(&GX3_PORT, 0, 0xc8); // accel, gyro, R
#endif

  //4 byte command for Continous Mode
  uart_put_byte(&GX3_PORT, 0, 0xc4);
  uart_put_byte(&GX3_PORT, 0, 0xc1);
  uart_put_byte(&GX3_PORT, 0, 0x29);
  uart_put_byte(&GX3_PORT, 0, 0xc8); // accel,gyro, R

  // Reset gyros to zero
  ahrs_gx3_align();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GX3_INFO, send_gx3);
#endif
}


void imu_gx3_periodic(void)
{
  /* IF IN NON-CONTINUOUS MODE, REQUEST DATA NOW
     uart_put_byte(&GX3_PORT, 0, 0xc8); // accel,gyro, R
  */
}


void gx3_packet_read_message(void)
{
  ahrs_gx3.accel.x     = bef(&ahrs_gx3.packet.msg_buf[1]);
  ahrs_gx3.accel.y     = bef(&ahrs_gx3.packet.msg_buf[5]);
  ahrs_gx3.accel.z     = bef(&ahrs_gx3.packet.msg_buf[9]);
  ahrs_gx3.rate.p      = bef(&ahrs_gx3.packet.msg_buf[13]);
  ahrs_gx3.rate.q      = bef(&ahrs_gx3.packet.msg_buf[17]);
  ahrs_gx3.rate.r      = bef(&ahrs_gx3.packet.msg_buf[21]);
  ahrs_gx3.rmat.m[0]   = bef(&ahrs_gx3.packet.msg_buf[25]);
  ahrs_gx3.rmat.m[1]   = bef(&ahrs_gx3.packet.msg_buf[29]);
  ahrs_gx3.rmat.m[2]   = bef(&ahrs_gx3.packet.msg_buf[33]);
  ahrs_gx3.rmat.m[3]   = bef(&ahrs_gx3.packet.msg_buf[37]);
  ahrs_gx3.rmat.m[4]   = bef(&ahrs_gx3.packet.msg_buf[41]);
  ahrs_gx3.rmat.m[5]   = bef(&ahrs_gx3.packet.msg_buf[45]);
  ahrs_gx3.rmat.m[6]   = bef(&ahrs_gx3.packet.msg_buf[49]);
  ahrs_gx3.rmat.m[7]   = bef(&ahrs_gx3.packet.msg_buf[53]);
  ahrs_gx3.rmat.m[8]   = bef(&ahrs_gx3.packet.msg_buf[57]);
  ahrs_gx3.time    = (uint32_t)(ahrs_gx3.packet.msg_buf[61] << 24 |
                                ahrs_gx3.packet.msg_buf[62] << 16 |
                                ahrs_gx3.packet.msg_buf[63] << 8 |
                                ahrs_gx3.packet.msg_buf[64]);
  ahrs_gx3.chksm = GX3_CHKSM(ahrs_gx3.packet.msg_buf);

  ahrs_gx3.freq = 62500.0 / (float)(ahrs_gx3.time - ahrs_gx3.ltime);
  ahrs_gx3.ltime = ahrs_gx3.time;

  // Acceleration
  VECT3_SMUL(ahrs_gx3.accel, ahrs_gx3.accel, 9.80665); // Convert g into m/s2
  // for compatibility with fixed point interface
  ACCELS_BFP_OF_REAL(imu.accel, ahrs_gx3.accel);

  // Rates
  // for compatibility with fixed point interface
  RATES_BFP_OF_REAL(imu.gyro, ahrs_gx3.rate);
  struct FloatRates body_rate;
  /* compute body rates */
  struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&imu.body_to_imu);
  float_rmat_ratemult(&body_rate, body_to_imu_rmat, &ahrs_gx3.rate);
  /* Set state */
  stateSetBodyRates_f(&body_rate);

  // Attitude
  struct FloatRMat ltp_to_body_rmat;
  float_rmat_comp(&ltp_to_body_rmat, &ahrs_gx3.rmat, body_to_imu_rmat);

#if AHRS_USE_GPS_HEADING && USE_GPS
  struct FloatEulers ltp_to_body_eulers;
  float_eulers_of_rmat(&ltp_to_body_eulers, &ltp_to_body_rmat);
  float course_f = (float)DegOfRad(gps.course / 1e7);
  if (course_f > 180.0) {
    course_f -= 360.0;
  }
  ltp_to_body_eulers.psi = (float)RadOfDeg(course_f);
  stateSetNedToBodyEulers_f(&ltp_to_body_eulers);
#else // !AHRS_USE_GPS_HEADING
#ifdef IMU_MAG_OFFSET
  struct FloatEulers ltp_to_body_eulers;
  float_eulers_of_rmat(&ltp_to_body_eulers, &ltp_to_body_rmat);
  ltp_to_body_eulers.psi -= ahrs_gx3.mag_offset;
  stateSetNedToBodyEulers_f(&ltp_to_body_eulers);
#else
  stateSetNedToBodyRMat_f(&ltp_to_body_rmat);
#endif // IMU_MAG_OFFSET
#endif // !AHRS_USE_GPS_HEADING
}


/* GX3 Packet Collection */
void gx3_packet_parse(uint8_t c)
{
  switch (ahrs_gx3.packet.status) {
    case GX3PacketWaiting:
      ahrs_gx3.packet.msg_idx = 0;
      if (c == GX3_HEADER) {
        ahrs_gx3.packet.status++;
        ahrs_gx3.packet.msg_buf[ahrs_gx3.packet.msg_idx] = c;
        ahrs_gx3.packet.msg_idx++;
      } else {
        ahrs_gx3.packet.hdr_error++;
      }
      break;
    case GX3PacketReading:
      ahrs_gx3.packet.msg_buf[ahrs_gx3.packet.msg_idx] =  c;
      ahrs_gx3.packet.msg_idx++;
      if (ahrs_gx3.packet.msg_idx == GX3_MSG_LEN) {
        if (gx3_verify_chk(ahrs_gx3.packet.msg_buf)) {
          ahrs_gx3.packet.msg_available = true;
        } else {
          ahrs_gx3.packet.msg_available = false;
          ahrs_gx3.packet.chksm_error++;
        }
        ahrs_gx3.packet.status = 0;
      }
      break;
    default:
      ahrs_gx3.packet.status = 0;
      ahrs_gx3.packet.msg_idx = 0;
      break;
  }
}

void ahrs_gx3_init(void)
{
  /* set ltp_to_imu so that body is zero */
  struct FloatQuat *body_to_imu_quat = orientationGetQuat_f(&imu.body_to_imu);
  QUAT_COPY(ahrs_gx3.ltp_to_imu_quat, *body_to_imu_quat);
#ifdef IMU_MAG_OFFSET
  ahrs_gx3.mag_offset = IMU_MAG_OFFSET;
#else
  ahrs_gx3.mag_offset = 0.0;
#endif
  ahrs_gx3.is_aligned = false;
}

void ahrs_gx3_register(void)
{
  ahrs_gx3_init();
  /// @todo: provide enable function
  ahrs_register_impl(NULL);
}


/* no scaling */
void imu_scale_gyro(struct Imu *_imu __attribute__((unused))) {}
void imu_scale_accel(struct Imu *_imu __attribute__((unused))) {}
void imu_scale_mag(struct Imu *_imu __attribute__((unused))) {}

void ahrs_gx3_publish_imu(void)
{
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgIMU_GYRO_INT32(IMU_GX3_ID, now_ts, &imu.gyro);
  AbiSendMsgIMU_ACCEL_INT32(IMU_GX3_ID, now_ts, &imu.accel);
  AbiSendMsgIMU_MAG_INT32(IMU_GX3_ID, now_ts, &imu.mag);
}

static inline void ReadGX3Buffer(void)
{
  while (uart_char_available(&GX3_PORT) && !ahrs_gx3.packet.msg_available) {
    gx3_packet_parse(uart_getch(&GX3_PORT));
  }
}

void imu_gx3_event(void)
{
  if (uart_char_available(&GX3_PORT)) {
    ReadGX3Buffer();
  }
  if (ahrs_gx3.packet.msg_available) {
    gx3_packet_read_message();
    ahrs_gx3_publish_imu();
    ahrs_gx3.packet.msg_available = false;
  }
}
