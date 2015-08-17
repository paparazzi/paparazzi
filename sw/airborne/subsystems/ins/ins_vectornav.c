/*
 * Copyright (C) 2015 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
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

struct InsVectornav ins_vn;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_vn.ltp_pos.x, &ins_vn.ltp_pos.y, &ins_vn.ltp_pos.z,
                    &ins_vn.ltp_speed.x, &ins_vn.ltp_speed.y, &ins_vn.ltp_speed.z,
                    &ins_vn.ltp_accel.x, &ins_vn.ltp_accel.y, &ins_vn.ltp_accel.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      &ins_vn.baro_z, &ins_vn.ltp_pos.z, &ins_vn.ltp_speed.z, &ins_vn.ltp_accel.z);
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  if (ins_vn.ltp_initialized) {
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ins_vn.ltp_def.ecef.x, &ins_vn.ltp_def.ecef.y, &ins_vn.ltp_def.ecef.z,
                          &ins_vn.ltp_def.lla.lat, &ins_vn.ltp_def.lla.lon, &ins_vn.ltp_def.lla.alt,
                          &ins_vn.ltp_def.hmsl, &ins_vn.qfe);
  }
}

static void send_vn_info(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_VECTORNAV_INFO(trans, dev, AC_ID,
                               &ins_vn.timestamp,
                               &ins_vn.vn_packet.chksm_error,
                               &ins_vn.vn_packet.hdr_error,
                               &ins_vn.vn_packet.counter,
                               &ins_vn.mode,
                               &ins_vn.err,
                               &ins_vn.YprU.phi,
                               &ins_vn.YprU.theta,
                               &ins_vn.YprU.psi);
}

static void send_vn_msg(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t vnstatus = (uint16_t)ins_vn.mode;
  struct FloatEulers *attitude;
  attitude = stateGetNedToBodyEulers_f();
  struct NedCoor_f *ned_speed;
  ned_speed = stateGetSpeedNed_f();
  pprz_msg_send_VECTORNAV_MSG(trans, dev, AC_ID,
                              &ins_vn.timestamp,
                              &attitude->phi,
                              &attitude->theta,
                              &attitude->psi,
                              &imuf.gyro.p,
                              &imuf.gyro.q,
                              &imuf.gyro.r,
                              &ins_vn.pos_lla[0],
                              &ins_vn.pos_lla[1],
                              &ins_vn.pos_lla[2],
                              &ned_speed->x,
                              &ned_speed->y,
                              &ned_speed->z,
                              &imuf.accel.x,
                              &imuf.accel.y,
                              &imuf.accel.z,
                              &gps.num_sv,
                              &gps.fix,
                              &ins_vn.posU[0],
                              &ins_vn.posU[1],
                              &ins_vn.posU[2],
                              &ins_vn.velU,
                              &ins_vn.ltp_accel_f.x,
                              &ins_vn.ltp_accel_f.y,
                              &ins_vn.ltp_accel_f.z,
                              &ins_vn.YprU.phi,
                              &ins_vn.YprU.theta,
                              &ins_vn.YprU.psi,
                              &vnstatus
                             );
}
#endif

#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE");
#endif

/**
 * Empty function for IMU initialization
 */
void imu_impl_init(void) {}

/**
 * Event handling for Vectornav
 */
void imu_vectornav_event(void)
{
  // receive data
  vn200_event(&(ins_vn.vn_packet));

  // read message
  if (ins_vn.vn_packet.msg_available) {
    ins_vectornav_read_message();
    ins_vn.vn_packet.msg_available = FALSE;
  }
}

/**
 * Compulsory register function
 */
void ins_vectornav_register(void)
{
  ins_register_impl(ins_vectornav_init);
}

/**
 * Initialize Vectornav struct
 */
void ins_vectornav_init(void)
{
  // Initialize variables
  ins_vn.vn_status = VNNotTracking;
  ins_vn.vn_time = get_sys_time_float();

  // Initialize packet
  ins_vn.vn_packet.status = VNMsgSync;
  ins_vn.vn_packet.msg_idx = 0;
  ins_vn.vn_packet.msg_available = FALSE;
  ins_vn.vn_packet.chksm_error = 0;
  ins_vn.vn_packet.hdr_error = 0;
  ins_vn.vn_packet.overrun_error = 0;
  ins_vn.vn_packet.noise_error = 0;
  ins_vn.vn_packet.framing_error = 0;

  INT32_VECT3_ZERO(ins_vn.ltp_pos);
  INT32_VECT3_ZERO(ins_vn.ltp_speed);
  INT32_VECT3_ZERO(ins_vn.ltp_accel);

  FLOAT_VECT3_ZERO(ins_vn.vel_ned);
  FLOAT_VECT3_ZERO(ins_vn.lin_accel);
  FLOAT_VECT3_ZERO(ins_vn.vel_body);

#if USE_INS_NAV_INIT
  ins_init_origin_from_flightplan();
  ins_vn.ltp_initialized = TRUE;
#else
  ins_vn.ltp_initialized  = FALSE;
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "INS", send_ins);
  register_periodic_telemetry(DefaultPeriodic, "INS_Z", send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, "INS_REF", send_ins_ref);
  register_periodic_telemetry(DefaultPeriodic, "VECTORNAV_INFO", send_vn_info);
  register_periodic_telemetry(DefaultPeriodic, "VECTORNAV_MSG", send_vn_msg);
#endif
}

/**
 * Read received data
 */
void ins_vectornav_read_message(void)
{
  ins_vn.vn_time = get_sys_time_float();

  uint16_t idx = VN_HEADER_SIZE;

  // Timestamp [nanoseconds] since startup
  static uint64_t nanostamp = 0;
  memcpy(&nanostamp, &ins_vn.vn_packet.msg_buf[idx], sizeof(uint64_t));
  idx += sizeof(uint64_t);

  // Timestamp [s]
  ins_vn.timestamp = ((float)nanostamp / 1000000000); // [nanoseconds to seconds]

  //Attitude, float, [degrees], yaw, pitch, roll, NED frame
  memcpy(&ins_vn.attitude, &ins_vn.vn_packet.msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  // Rates (imu frame), float, [rad/s]
  memcpy(&imuf.gyro, &ins_vn.vn_packet.msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  //Pos LLA, double,[deg, deg, m]
  //The estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectfully.
  memcpy(&ins_vn.pos_lla, &ins_vn.vn_packet.msg_buf[idx], 3 * sizeof(double));
  idx += 3 * sizeof(double);

  //VelNed, float [m/s]
  //The estimated velocity in the North East Down (NED) frame, given in m/s.
  memcpy(&ins_vn.vel_ned, &ins_vn.vn_packet.msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  // Accel (imu-frame), float, [m/s^-2]
  memcpy(&imuf.accel, &ins_vn.vn_packet.msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  // tow (in nanoseconds), uint64
  static uint64_t tow = 0;
  memcpy(&tow, &ins_vn.vn_packet.msg_buf[idx], sizeof(uint64_t));
  idx += sizeof(uint64_t);
  tow = tow / 1000000; // nanoseconds to miliseconds
  gps.tow = (uint32_t) tow;

  //num sats, uint8
  gps.num_sv = ins_vn.vn_packet.msg_buf[idx];
  idx++;

  //gps fix, uint8
  gps.fix = ins_vn.vn_packet.msg_buf[idx];
  idx++;

  //posU, float[3]
  memcpy(&ins_vn.posU, &ins_vn.vn_packet.msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  //velU, float
  memcpy(&ins_vn.velU, &ins_vn.vn_packet.msg_buf[idx], sizeof(float));
  idx += sizeof(float);

  //linear acceleration imu-body frame, float [m/s^2]
  //The estimated linear acceleration (without gravity) reported in m/s^2, and given in the body frame. The
  //acceleration measurement has been bias compensated by the onboard INS filter, and the gravity
  //component has been removed using the current gravity reference vector model. This measurement is
  //attitude dependent, since the attitude solution is required to map the gravity reference vector (known
  //in the inertial NED frame), into the body frame so that it can be removed from the measurement. If the
  //device is stationary and the onboard INS filter is tracking, the measurement nominally will read 0 in all
  //three axes.
  memcpy(&ins_vn.lin_accel, &ins_vn.vn_packet.msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  //YprU, float[3]
  memcpy(&ins_vn.YprU, &ins_vn.vn_packet.msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  //instatus, uint16
  memcpy(&ins_vn.ins_status, &ins_vn.vn_packet.msg_buf[idx], sizeof(uint16_t));
  idx += sizeof(uint16_t);

  //Vel body, float [m/s]
  // The estimated velocity in the body (i.e. imu) frame, given in m/s.
  memcpy(&ins_vn.vel_body, &ins_vn.vn_packet.msg_buf[idx], sizeof(float));
  idx += sizeof(float);

  // Propaget the ins states
  ins_vectornav_propagate();
}

/**
 * Check INS status
 */
void ins_vectornav_check_status(void)
{
  ins_vn.mode = (uint8_t)(ins_vn.ins_status & 0x03);
  ins_vn.err = (uint8_t)((ins_vn.ins_status >> 3) & 0x0F);
}

/**
 * Set speed (velocity) uncertainty (NED)
 * speed accuracy in cm/s
 */
void ins_vectornav_set_sacc(void)
{
  gps.sacc = (uint32_t)(ins_vn.velU * 100);
}

/**
 * Find maximum uncertainty (NED)
 * position accuracy in cm
 */
void ins_vectornav_set_pacc(void)
{
  float pacc = ins_vn.posU[0]; // in meters
  if (ins_vn.posU[1] > pacc) {
    pacc = ins_vn.posU[1];
  }
  if (ins_vn.posU[2] > pacc) {
    pacc = ins_vn.posU[2];
  }

  gps.pacc = (uint32_t)(pacc * 100);
}

/**
 * Convert yaw, pitch, and roll data from VectorNav
 * to correct attitude
 * yaw(0), pitch(1), roll(2) -> phi, theta, psi
 * [deg] -> rad
 */
void ins_vectornav_yawPitchRoll_to_attitude(struct FloatEulers *vn_attitude)
{
  static struct FloatEulers att_rad;
  att_rad.phi = (vn_attitude->psi) * DEG_TO_RAD;
  att_rad.theta = (vn_attitude->theta) * DEG_TO_RAD;
  att_rad.psi = (vn_attitude->phi) * DEG_TO_RAD;

  vn_attitude->phi = att_rad.phi;
  vn_attitude->theta = att_rad.theta;
  vn_attitude->psi = att_rad.psi;
}

/**
 *  Propagate the received states into the vehicle
 *  state machine
 */
void ins_vectornav_propagate()
{
  // Acceleration [m/s^2]
  ACCELS_BFP_OF_REAL(imu.accel, imuf.accel); // for backwards compatibility with fixed point interface

  // Rates [rad/s]
  static struct FloatRates body_rate;
  RATES_BFP_OF_REAL(imu.gyro, imuf.gyro);  // for backwards compatibility with fixed point interface
  float_rmat_ratemult(&body_rate, orientationGetRMat_f(&imuf.body_to_imu), &imuf.gyro); // compute body rates
  stateSetBodyRates_f(&body_rate);   // Set state [rad/s]

  // Attitude [deg]
  ins_vectornav_yawPitchRoll_to_attitude(&ins_vn.attitude); // convert to correct units and axis [rad]
  static struct FloatQuat imu_quat; // convert from euler to quat
  float_quat_of_eulers(&imu_quat, &ins_vn.attitude);
  static struct FloatRMat imu_rmat; // convert from quat to rmat
  float_rmat_of_quat(&imu_rmat, &imu_quat);
  static struct FloatRMat ltp_to_body_rmat; // rotate to body frame
  float_rmat_comp(&ltp_to_body_rmat, &imu_rmat, orientationGetRMat_f(&imuf.body_to_imu));
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
  stateSetSpeedNed_f(&ins_vn.vel_ned); // set state

  // NED (LTP) acceleration [m/s^2]
  static struct FloatVect3 accel_meas_ltp;// first we need to rotate linear acceleration from imu-frame to body-frame
  float_rmat_transp_vmult(&accel_meas_ltp, orientationGetRMat_f(&imuf.body_to_imu), &(ins_vn.lin_accel));
  static struct NedCoor_f ltp_accel; // assign to NedCoord_f struct
  VECT3_ASSIGN(ltp_accel, accel_meas_ltp.x, accel_meas_ltp.y, accel_meas_ltp.z);
  stateSetAccelNed_f(&ltp_accel); // then set the states
  ins_vn.ltp_accel_f = ltp_accel;

  // LLA position [rad, rad, m]
  //static struct LlaCoor_f lla_pos; // convert from deg to rad, and from double to float
  ins_vn.lla_pos.lat = ((float)ins_vn.pos_lla[0]) * DEG_TO_RAD; // ins_impl.pos_lla[0] = lat
  ins_vn.lla_pos.lon = ((float)ins_vn.pos_lla[1]) * DEG_TO_RAD; // ins_impl.pos_lla[1] = lon
  ins_vn.lla_pos.alt = ((float)ins_vn.pos_lla[2]); // ins_impl.pos_lla[2] = alt
  LLA_BFP_OF_REAL(gps.lla_pos, ins_vn.lla_pos);
  stateSetPositionLla_i(&gps.lla_pos);

  // ECEF position
  struct LtpDef_f def;
  ltp_def_from_lla_f(&def, &ins_vn.lla_pos);
  struct EcefCoor_f ecef_vel;
  ecef_of_ned_point_f(&ecef_vel, &def, &ins_vn.vel_ned);
  ECEF_BFP_OF_REAL(gps.ecef_vel, ecef_vel);

  // ECEF velocity
  gps.ecef_pos.x = stateGetPositionEcef_i()->x;
  gps.ecef_pos.y = stateGetPositionEcef_i()->y;
  gps.ecef_pos.z = stateGetPositionEcef_i()->z;


#if GPS_USE_LATLONG
  // GPS UTM
  /* Computes from (lat, long) in the referenced UTM zone */
  struct UtmCoor_f utm_f;
  utm_f.zone = nav_utm_zone0;
  /* convert to utm */
  //utm_of_lla_f(&utm_f, &lla_f);
  utm_of_lla_f(&utm_f, &ins_vn.lla_pos);
  /* copy results of utm conversion */
  gps.utm_pos.east = (int32_t)(utm_f.east * 100);
  gps.utm_pos.north = (int32_t)(utm_f.north * 100);
  gps.utm_pos.alt = (int32_t)(utm_f.alt * 1000);
  gps.utm_pos.zone = (uint8_t)nav_utm_zone0;
#endif

  // GPS Ground speed
  float speed = sqrt(ins_vn.vel_ned.x * ins_vn.vel_ned.x + ins_vn.vel_ned.y * ins_vn.vel_ned.y);
  gps.gspeed = ((uint16_t)(speed * 100));

  // GPS course
  gps.course = (int32_t)(1e7 * (atan2(ins_vn.vel_ned.y, ins_vn.vel_ned.x)));

  // Because we have not HMSL data from Vectornav, we are using LLA-Altitude
  // as a workaround
  gps.hmsl = (uint32_t)(gps.lla_pos.alt);

  // set position uncertainty
  ins_vectornav_set_pacc();

  // set velocity uncertainty
  ins_vectornav_set_sacc();

  // check GPS status
  gps.last_msg_time = sys_time.nb_sec;
  gps.last_msg_ticks = sys_time.nb_sec_rem;
  if (gps.fix == GPS_FIX_3D) {
    gps.last_3dfix_time = sys_time.nb_sec;
    gps.last_3dfix_ticks = sys_time.nb_sec_rem;
  }

  // read INS status
  ins_vectornav_check_status();

  // update internal states for telemetry purposes
  ins_vn.ltp_pos = *stateGetPositionNed_i();
  ins_vn.ltp_speed = *stateGetSpeedNed_i();
  ins_vn.ltp_accel = *stateGetAccelNed_i();

  // send ABI messages
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgGPS(GPS_UBX_ID, now_ts, &gps);
  AbiSendMsgIMU_GYRO_INT32(IMU_ASPIRIN_ID, now_ts, &imu.gyro);
  AbiSendMsgIMU_ACCEL_INT32(IMU_ASPIRIN_ID, now_ts, &imu.accel);
  AbiSendMsgIMU_MAG_INT32(IMU_ASPIRIN_ID, now_ts, &imu.mag);
}


/**
 * initialize the local origin (ltp_def) from flight plan position
 */
void ins_init_origin_from_flightplan(void)
{

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_vn.ltp_def, &ecef_nav0);
  ins_vn.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_vn.ltp_def);

}
