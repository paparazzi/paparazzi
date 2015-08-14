/*
 * Copyright (C) 2014 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
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
 * @file ins_vectornav.c
 *
 * Vectornav VN-200 INS subsystem
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include "subsystems/ins/ins_vectornav.h"

#include "subsystems/abi.h"

#include "subsystems/imu.h"
#include "subsystems/gps.h"

#include "generated/airframe.h"

#include "math/pprz_geodetic_int.h"
#include "math/pprz_isa.h"

#include "generated/flight_plan.h"

#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE")
#endif

static inline void ins_vectornav_set_pacc(void);
static inline void ins_vectornav_set_sacc(void);
static inline void ins_vectornav_check_status(void);

struct InsInt ins_impl;

uint16_t calc_chk;
uint16_t rec_chk;
uint16_t counter;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
      &ins_impl.ltp_pos.x, &ins_impl.ltp_pos.y, &ins_impl.ltp_pos.z,
      &ins_impl.ltp_speed.x, &ins_impl.ltp_speed.y, &ins_impl.ltp_speed.z,
      &ins_impl.ltp_accel.x, &ins_impl.ltp_accel.y, &ins_impl.ltp_accel.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
      &ins_impl.baro_z, &ins_impl.ltp_pos.z, &ins_impl.ltp_speed.z, &ins_impl.ltp_accel.z);
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  if (ins_impl.ltp_initialized) {
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
      &ins_impl.ltp_def.ecef.x, &ins_impl.ltp_def.ecef.y, &ins_impl.ltp_def.ecef.z,
      &ins_impl.ltp_def.lla.lat, &ins_impl.ltp_def.lla.lon, &ins_impl.ltp_def.lla.alt,
      &ins_impl.ltp_def.hmsl, &ins_impl.qfe);
  }
}

static void send_vn_info(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_VECTORNAV_INFO(trans, dev, AC_ID,
      &ins_impl.timestamp,
      &ins_impl.vn_packet.chksm_error,
      &ins_impl.vn_packet.hdr_error,
      &counter,
      &ins_impl.mode,
      &ins_impl.err,
      &ins_impl.YprU.phi,
      &ins_impl.YprU.theta,
      &ins_impl.YprU.psi);
}

static void send_vn_msg(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t vnstatus = (uint16_t)ins_impl.mode;
  struct FloatEulers* attitude;
  attitude = stateGetNedToBodyEulers_f();
  struct NedCoor_f* ned_speed;
  ned_speed = stateGetSpeedNed_f();
  pprz_msg_send_VECTORNAV_MSG(trans, dev, AC_ID,
      &ins_impl.timestamp,
      &attitude->phi,
      &attitude->theta,
      &attitude->psi,
      &imuf.gyro.p,
      &imuf.gyro.q,
      &imuf.gyro.r,
      &ins_impl.pos_lla[0],
      &ins_impl.pos_lla[1],
      &ins_impl.pos_lla[2],
      &ned_speed->x,
      &ned_speed->y,
      &ned_speed->z,
      &imuf.accel.x,
      &imuf.accel.y,
      &imuf.accel.z,
      &gps.num_sv,
      &gps.fix,
      &ins_impl.posU[0],
      &ins_impl.posU[1],
      &ins_impl.posU[2],
      &ins_impl.velU,
      &ins_impl.ltp_accel_f.x,
      &ins_impl.ltp_accel_f.y,
      &ins_impl.ltp_accel_f.z,
      &ins_impl.YprU.phi,
      &ins_impl.YprU.theta,
      &ins_impl.YprU.psi,
      &vnstatus
      );
}
#endif

void ins_vectornav_register(void)
{
  ins_register_impl(ins_vectornav_init);
}


/** INS initialization. Called at startup.
 *  Needs to be implemented by each INS algorithm.
 */
void ins_vectornav_init(void) {
  // Initialize variables
  ins_impl.vn_status = VNNotTracking;
  ins_impl.vn_time = get_sys_time_float();

  // Initialize packet
  ins_impl.vn_packet.status = VNMsgSync;
  ins_impl.vn_packet.msg_idx = 0;
  ins_impl.vn_packet.msg_available = FALSE;
  ins_impl.vn_packet.chksm_error = 0;
  ins_impl.vn_packet.hdr_error = 0;
  ins_impl.vn_packet.overrun_error = 0;
  ins_impl.vn_packet.noise_error = 0;
  ins_impl.vn_packet.framing_error = 0;

#if USE_INS_NAV_INIT
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_impl.ltp_def, &ecef_nav0);
  ins_impl.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_impl.ltp_def);

  ins_impl.ltp_initialized = TRUE;
#else
  ins_gp.ltp_initialized  = FALSE;
#endif

  /*
#if USE_INS_NAV_INIT
  ins_init_origin_from_flightplan();
  ins_impl.ltp_initialized = TRUE;
#else
  TODO("Warning: USE_INS_NAV_INIT set to FALSE, untested behavior!");
  ins_impl.ltp_initialized  = FALSE;
#endif
  */

  INT32_VECT3_ZERO(ins_impl.ltp_pos);
  INT32_VECT3_ZERO(ins_impl.ltp_speed);
  INT32_VECT3_ZERO(ins_impl.ltp_accel);

  FLOAT_VECT3_ZERO(ins_impl.vel_ned);
  FLOAT_VECT3_ZERO(ins_impl.lin_accel);
  FLOAT_VECT3_ZERO(ins_impl.vel_body);

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, "INS", send_ins);
  register_periodic_telemetry(DefaultPeriodic, "INS_Z", send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, "INS_REF", send_ins_ref);
  register_periodic_telemetry(DefaultPeriodic, "VECTORNAV_INFO", send_vn_info);
  register_periodic_telemetry(DefaultPeriodic, "VECTORNAV_MSG", send_vn_msg);
#endif
}

/**
 * Convert yaw, pitch, and roll data from VectorNav
 * to correct attitude
 * yaw(0), pitch(1), roll(2) -> phi, theta, psi
 * [deg] -> rad
 */
static inline void ins_vectornav_yawPitchRoll_to_attitude(struct FloatEulers* vn_attitude) {
  static struct FloatEulers att_rad;
  att_rad.phi = (vn_attitude->psi)*DEG_TO_RAD;
  att_rad.theta = (vn_attitude->theta)*DEG_TO_RAD;
  att_rad.psi = (vn_attitude->phi)*DEG_TO_RAD;

  vn_attitude->phi = att_rad.phi;
  vn_attitude->theta = att_rad.theta;
  vn_attitude->psi = att_rad.psi;
}

/** Propagation. Usually integrates the gyro rates to angles.
 *  Reads the global #imu data struct.
 *  Needs to be implemented by each INS algorithm.
 */
void ins_vectornav_propagate() {
  // Acceleration [m/s^2]
  ACCELS_BFP_OF_REAL(imu.accel, imuf.accel); // for backwards compatibility with fixed point interface

  // Rates [rad/s]
  static struct FloatRates body_rate;
  RATES_BFP_OF_REAL(imu.gyro, imuf.gyro ); // for backwards compatibility with fixed point interface
  FLOAT_RMAT_RATEMULT(body_rate, imuf.body_to_imu_rmat, imuf.gyro); // compute body rates
  stateSetBodyRates_f(&body_rate);   // Set state [rad/s]

  // Attitude [deg]
  ins_vectornav_yawPitchRoll_to_attitude(&ins_impl.attitude); // convert to correct units and axis [rad]
  static struct FloatQuat imu_quat; // convert from euler to quat
  FLOAT_QUAT_OF_EULERS(imu_quat, ins_impl.attitude);
  static struct FloatRMat imu_rmat; // convert from quat to rmat
  FLOAT_RMAT_OF_QUAT(imu_rmat, imu_quat);
  static struct FloatRMat ltp_to_body_rmat; // rotate to body frame
  FLOAT_RMAT_COMP(ltp_to_body_rmat, imu_rmat, imuf.body_to_imu_rmat);
  stateSetNedToBodyRMat_f(&ltp_to_body_rmat); // set body states [rad]

  // NED (LTP) velocity [m/s]
  // North east down (NED), also known as local tangent plane (LTP),
  // is a geographical coordinate system for representing state vectors that is commonly used in aviation.
  // It consists of three numbers: one represents the position along the northern axis,
  // one along the eastern axis, and one represents vertical position. Down is chosen as opposed to
  // up in order to comply with the right-hand rule.
  // The origin of this coordinate system is usually chosen to be the aircraft's center of gravity.
  // x = North
  // y = East
  // z = Down
  stateSetSpeedNed_f(&ins_impl.vel_ned); // set state

  // NED (LTP) acceleration [m/s^2]
  static struct FloatVect3 accel_meas_ltp;// first we need to rotate linear acceleration from imu-frame to body-frame
  float_rmat_transp_vmult(&accel_meas_ltp, &(imuf.body_to_imu_rmat), &(ins_impl.lin_accel));
  static struct NedCoor_f ltp_accel; // assign to NedCoord_f struct
  VECT3_ASSIGN(ltp_accel, accel_meas_ltp.x, accel_meas_ltp.y, accel_meas_ltp.z);
  stateSetAccelNed_f(&ltp_accel); // then set the states
  ins_impl.ltp_accel_f = ltp_accel;

  // LLA position [rad, rad, m]
  //static struct LlaCoor_f lla_pos; // convert from deg to rad, and from double to float
  ins_impl.lla_pos.lat = ((float)ins_impl.pos_lla[0])*DEG_TO_RAD; // ins_impl.pos_lla[0] = lat
  ins_impl.lla_pos.lon = ((float)ins_impl.pos_lla[1])*DEG_TO_RAD; // ins_impl.pos_lla[1] = lon
  ins_impl.lla_pos.alt = ((float)ins_impl.pos_lla[2]); // ins_impl.pos_lla[2] = alt
  /*
   * FIXME: function stateSetPositionLla_f doesn't work properly - when used, the calculated fixed point position
   * is significantly off (by orders of magnitude) - can be shown on GCS, the AC position is estimated in middle of the ocen
   * try with sample data from Vector Nav
   * As far as I can tell, the float conversion from LLA to ECEF works fine (double check the numerical precision with matlab), so probably
   * going from ECEF_f to ECEF_i/NED_i makes the problem
   */
  //stateSetPositionLla_f(&ins_impl.lla_pos); // then set the states

  LLA_BFP_OF_REAL(gps.lla_pos, ins_impl.lla_pos);
  stateSetPositionLla_i(&gps.lla_pos);

  // fill in GPS message variables (ECEF is needed for correct display of AC in GCS map)
  gps.ecef_pos.x = stateGetPositionEcef_i()->x;
  gps.ecef_pos.y = stateGetPositionEcef_i()->y;
  gps.ecef_pos.z = stateGetPositionEcef_i()->z;

  /*
   * FIXME: ECEF velocity included just for order - it actually shows too high numbers, probably calculation error?
   */
  gps.ecef_vel.x = stateGetSpeedEcef_i()->x;
  gps.ecef_vel.y = stateGetSpeedEcef_i()->y;
  gps.ecef_vel.z = stateGetSpeedEcef_i()->z;

  // hack for GPS hmsl
  gps.hmsl = (uint32_t)(gps.lla_pos.alt);

  // set position uncertainty
  ins_vectornav_set_pacc();

  // set velocity uncertainty
  ins_vectornav_set_sacc();

  // check GPS status
  if (gps.fix == GPS_FIX_3D) {
    gps.last_3dfix_time = sys_time.nb_sec;
    gps.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps.last_msg_time = sys_time.nb_sec;
    gps.last_msg_ticks = sys_time.nb_sec_rem;
  }

  // read INS status
  ins_vectornav_check_status();

  // update internal states for telemetry purposes
        ins_impl.ltp_pos = *stateGetPositionNed_i();
        ins_impl.ltp_speed = *stateGetSpeedNed_i();
        ins_impl.ltp_accel = *stateGetAccelNed_i();

  // send ABI messages
   // current timestamp
   uint32_t now_ts = get_sys_time_usec();
   AbiSendMsgGPS(GPS_UBX_ID, now_ts, &gps);
   AbiSendMsgIMU_GYRO_INT32(IMU_ASPIRIN_ID, now_ts, &imu.gyro);
   AbiSendMsgIMU_ACCEL_INT32(IMU_ASPIRIN_ID, now_ts, &imu.accel);
   AbiSendMsgIMU_MAG_INT32(IMU_ASPIRIN_ID, now_ts, &imu.mag);
}

/**
 * Check INS status
 */
static inline void ins_vectornav_check_status(void) {
  ins_impl.mode = (uint8_t)(ins_impl.ins_status & 0x03);
  ins_impl.err = (uint8_t)((ins_impl.ins_status >> 3) & 0x0F);
}

/**
 * Set speed (velocity) uncertainty (NED)
 * speed accuracy in cm/s
 */
static inline void ins_vectornav_set_sacc(void){
  gps.sacc = (uint32_t)(ins_impl.velU*100);
}

/**
 * Find maximum uncertainty (NED)
 * position accuracy in cm
 */
static inline void ins_vectornav_set_pacc(void){
  float pacc = ins_impl.posU[0]; // in meters
  if (ins_impl.posU[1] > pacc) {
    pacc = ins_impl.posU[1];
  }
  if (ins_impl.posU[2] > pacc) {
    pacc = ins_impl.posU[2];
  }

  gps.pacc = (uint32_t)(pacc*100);
}


/**
 *  initialize the local origin (ltp_def) from flight plan position
 */
void ins_init_origin_from_flightplan(void) {
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = INT32_RAD_OF_DEG(NAV_LAT0);
  llh_nav0.lon = INT32_RAD_OF_DEG(NAV_LON0);
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0; //+ NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_impl.ltp_def, &ecef_nav0);
  ins_impl.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_impl.ltp_def);
  stateSetPositionEcef_i(&ecef_nav0);
}


/**
 * Read received packet
 */
void ins_vectornav_read_message(void) {
  ins_impl.vn_time = get_sys_time_float();

  uint16_t idx = VN_HEADER_SIZE;

  // Timestamp [nanoseconds] since startup
  static uint64_t nanostamp = 0;
  memcpy(&nanostamp, &ins_impl.vn_packet.msg_buf[idx], sizeof(uint64_t));
  idx += sizeof(uint64_t);

  // Timestamp [s]
  ins_impl.timestamp = ((float)nanostamp/1000000000); // [nanoseconds to seconds]

  //Attitude, float, [degrees], yaw, pitch, roll, NED frame
  memcpy(&ins_impl.attitude, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  // Rates (imu frame), float, [rad/s]
  memcpy(&imuf.gyro, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  //Pos LLA, double,[deg, deg, m]
  //The estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectfully.
  memcpy(&ins_impl.pos_lla, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(double));
  idx += 3*sizeof(double);

  //VelNed, float [m/s]
  //The estimated velocity in the North East Down (NED) frame, given in m/s.
  memcpy(&ins_impl.vel_ned, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  // Accel (imu-frame), float, [m/s^-2]
  memcpy(&imuf.accel, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  // tow (in nanoseconds), uint64
  static uint64_t tow = 0;
  memcpy(&tow, &ins_impl.vn_packet.msg_buf[idx], sizeof(uint64_t));
  idx += sizeof(uint64_t);
  tow = tow / 1000000; // nanoseconds to miliseconds
  gps.tow = (uint32_t) tow;

  //num sats, uint8
  gps.num_sv = ins_impl.vn_packet.msg_buf[idx];
  idx++;

  //gps fix, uint8
  gps.fix = ins_impl.vn_packet.msg_buf[idx];
  idx++;

  //posU, float[3]
  memcpy(&ins_impl.posU, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  //velU, float
  memcpy(&ins_impl.velU, &ins_impl.vn_packet.msg_buf[idx], sizeof(float));
  idx += sizeof(float);

  //linear acceleration imu-body frame, float [m/s^2]
  //The estimated linear acceleration (without gravity) reported in m/s^2, and given in the body frame. The
  //acceleration measurement has been bias compensated by the onboard INS filter, and the gravity
  //component has been removed using the current gravity reference vector model. This measurement is
  //attitude dependent, since the attitude solution is required to map the gravity reference vector (known
  //in the inertial NED frame), into the body frame so that it can be removed from the measurement. If the
  //device is stationary and the onboard INS filter is tracking, the measurement nominally will read 0 in all
  //three axes.
  memcpy(&ins_impl.lin_accel, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  //YprU, float[3]
  memcpy(&ins_impl.YprU, &ins_impl.vn_packet.msg_buf[idx], 3*sizeof(float));
  idx += 3*sizeof(float);

  //instatus, uint16
  memcpy(&ins_impl.ins_status, &ins_impl.vn_packet.msg_buf[idx], sizeof(uint16_t));
  idx += sizeof(uint16_t);

  //Vel body, float [m/s]
  // The estimated velocity in the body (i.e. imu) frame, given in m/s.
  memcpy(&ins_impl.vel_body, &ins_impl.vn_packet.msg_buf[idx], sizeof(float));
  idx += sizeof(float);

  ins_vectornav_propagate();
}


/**
 *  Packet Collection & state machine
 */
void ins_vectornav_parse( uint8_t c) {
  switch (ins_impl.vn_packet.status) {
  case VNMsgSync:
    // sync the header
    ins_impl.vn_packet.msg_idx = 0;
    if (c == VN_SYNC) {
      ins_impl.vn_packet.status = VNMsgHeader;
    } else {
      ins_impl.vn_packet.hdr_error++;
    }
    break;
  case VNMsgHeader:
    // read header data (we expect 0x39)
    if (c == VN_OUTPUT_GROUP) {
      // increment idx and save current byte for checksum
      ins_impl.vn_packet.status = VNMsgGroup;
      ins_impl.vn_packet.msg_buf[ins_impl.vn_packet.msg_idx] = c;
      ins_impl.vn_packet.msg_idx++;
    }
    else {
      ins_impl.vn_packet.hdr_error++;
      ins_impl.vn_packet.status = VNMsgSync;
    }
    break;
    break;
  case VNMsgGroup:
    // read header data
    ins_impl.vn_packet.msg_buf[ins_impl.vn_packet.msg_idx] = c;
    ins_impl.vn_packet.msg_idx++;
    if (ins_impl.vn_packet.msg_idx == VN_GROUP_BYTES) {
      ins_impl.vn_packet.datalength = VN_PAYLOAD_SIZE+VN_HEADER_SIZE;
      ins_impl.vn_packet.status = VNMsgData;
    }
    break;
  case VNMsgData:
    ins_impl.vn_packet.msg_buf[ins_impl.vn_packet.msg_idx] =  c;
    ins_impl.vn_packet.msg_idx++;
    if (ins_impl.vn_packet.msg_idx == (ins_impl.vn_packet.datalength+2)) {
      if (verify_chk(ins_impl.vn_packet.msg_buf,ins_impl.vn_packet.datalength, &calc_chk, &rec_chk)) {
        ins_impl.vn_packet.msg_available = TRUE;
        counter++;
      } else {
        ins_impl.vn_packet.msg_available = FALSE;
        ins_impl.vn_packet.chksm_error++;
      }
      ins_impl.vn_packet.status = VNMsgSync;
    }
    break;
  default:;
    ins_impl.vn_packet.status = VNMsgSync;
    ins_impl.vn_packet.msg_idx = 0;
    break;
  }
}

