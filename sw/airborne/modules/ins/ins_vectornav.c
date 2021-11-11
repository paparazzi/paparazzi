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
#include "modules/ins/ins_vectornav.h"
#include "math/pprz_geodetic_wgs84.h"

struct InsVectornav ins_vn;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_vn.ltp_pos_i.x, &ins_vn.ltp_pos_i.y, &ins_vn.ltp_pos_i.z,
                    &ins_vn.ltp_speed_i.x, &ins_vn.ltp_speed_i.y, &ins_vn.ltp_speed_i.z,
                    &ins_vn.ltp_accel_i.x, &ins_vn.ltp_accel_i.y, &ins_vn.ltp_accel_i.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      &ins_vn.baro_z, &ins_vn.ltp_pos_i.z, &ins_vn.ltp_speed_i.z, &ins_vn.ltp_accel_i.z);
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
                               &ins_vn.vn_data.timestamp,
                               &ins_vn.vn_packet.chksm_error,
                               &ins_vn.vn_packet.hdr_error,
                               &ins_vn.vn_rate,
                               &ins_vn.vn_data.mode,
                               &ins_vn.vn_data.err,
                               &ins_vn.vn_data.ypr_u.phi,
                               &ins_vn.vn_data.ypr_u.theta,
                               &ins_vn.vn_data.ypr_u.psi);
}

static void send_accel(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_ACCEL(trans, dev, AC_ID,
                          &ins_vn.vn_data.accel.x, &ins_vn.vn_data.accel.y, &ins_vn.vn_data.accel.z);
}

static void send_gyro(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_GYRO(trans, dev, AC_ID,
                         &ins_vn.vn_data.gyro.p, &ins_vn.vn_data.gyro.q, &ins_vn.vn_data.gyro.r);
}

static void send_accel_scaled(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_ACCEL_SCALED(trans, dev, AC_ID,
                                 &ins_vn.accel_i.x, &ins_vn.accel_i.y, &ins_vn.accel_i.z);
}

static void send_gyro_scaled(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_GYRO_SCALED(trans, dev, AC_ID,
                                 &ins_vn.gyro_i.p, &ins_vn.gyro_i.q, &ins_vn.gyro_i.r);
}
#endif

#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE");
#endif


/**
 * Monitors vectornav data rate
 * and changes GPS lock if the data rate
 * is too low.
 *
 */
void ins_vectornav_monitor(void)
{
  static uint16_t last_cnt = 0;
  static uint16_t sec_cnt = 0;

  sec_cnt = ins_vn.vn_packet.counter -  last_cnt;
  ins_vn.vn_rate = sec_cnt; // update frequency counter

  // we want at least 75% of periodic frequency to be able to control the airfcraft
  if (ins_vn.vn_rate < (PERIODIC_FREQUENCY*0.75)) {
    gps.fix = GPS_FIX_NONE;
  }

  // Make gps pacc available in GPS page on GCS
  static uint32_t last_pacc = 0;
  // update only if pacc changes
  if (last_pacc != gps.pacc) {
    last_pacc = gps.pacc;
    // we don't know the value of CNO, hence oscillate
    // between 0 and 1 to not confuse the user
    gps.svinfos[0].cno = (gps.svinfos[0].cno + 1) % 2;
  }

  // update counter
  last_cnt = ins_vn.vn_packet.counter;

  // reset mode
  ins_vn.vn_data.mode = 0;
}


/**
 * Event handling for Vectornav
 */
void ins_vectornav_event(void)
{
  // receive data
  vn200_event(&(ins_vn.vn_packet));

  // read message
  if (ins_vn.vn_packet.msg_available) {
    vn200_read_message(&(ins_vn.vn_packet),&(ins_vn.vn_data));
    ins_vn.vn_packet.msg_available = false;
    ins_vectornav_propagate();
  }
}


/**
 * Initialize Vectornav struct
 */
void ins_vectornav_init(void)
{
  // Initialize variables
  ins_vn.vn_status = VNNotTracking;
  ins_vn.vn_rate = 0;

  // Initialize packet
  ins_vn.vn_packet.status = VNMsgSync;
  ins_vn.vn_packet.msg_idx = 0;
  ins_vn.vn_packet.msg_available = false;
  ins_vn.vn_packet.chksm_error = 0;
  ins_vn.vn_packet.hdr_error = 0;
  ins_vn.vn_packet.overrun_error = 0;
  ins_vn.vn_packet.noise_error = 0;
  ins_vn.vn_packet.framing_error = 0;

  INT32_VECT3_ZERO(ins_vn.ltp_pos_i);
  INT32_VECT3_ZERO(ins_vn.ltp_speed_i);
  INT32_VECT3_ZERO(ins_vn.ltp_accel_i);
  INT32_VECT3_ZERO(ins_vn.accel_i);

  // initialize data struct
  memset(&(ins_vn.vn_data), 0, sizeof(struct VNData));

#if USE_INS_NAV_INIT
  ins_init_origin_i_from_flightplan(&ins_vn.ltp_def);
  ins_vn.ltp_initialized = true;
#else
  ins_vn.ltp_initialized  = false;
#endif

  struct FloatEulers body_to_imu_eulers =
  {INS_VN_BODY_TO_IMU_PHI, INS_VN_BODY_TO_IMU_THETA, INS_VN_BODY_TO_IMU_PSI};
  orientationSetEulers_f(&ins_vn.body_to_imu, &body_to_imu_eulers);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VECTORNAV_INFO, send_vn_info);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_ACCEL, send_accel);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_GYRO, send_gyro);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_ACCEL_SCALED, send_accel_scaled);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_GYRO_SCALED, send_gyro_scaled);
#endif
}




/**
 * Set speed (velocity) uncertainty (NED)
 * speed accuracy in cm/s
 */
void ins_vectornav_set_sacc(void)
{
  gps.sacc = (uint32_t)(ins_vn.vn_data.vel_u * 100);
}

/**
 * Find maximum uncertainty (NED)
 * position accuracy in cm
 */
void ins_vectornav_set_pacc(void)
{
  float pacc = ins_vn.vn_data.pos_u[0]; // in meters
  if (ins_vn.vn_data.pos_u[1] > pacc) {
    pacc = ins_vn.vn_data.pos_u[1];
  }
  if (ins_vn.vn_data.pos_u[2] > pacc) {
    pacc = ins_vn.vn_data.pos_u[2];
  }
  gps.pacc = (uint32_t)(pacc * 100);
}



/**
 *  Propagate the received states into the vehicle
 *  state machine
 */
void ins_vectornav_propagate()
{
  // Acceleration [m/s^2]
  // in fixed point for sending as ABI and telemetry msgs
  ACCELS_BFP_OF_REAL(ins_vn.accel_i, ins_vn.vn_data.accel);

  // Rates [rad/s]
  static struct FloatRates body_rate;
  // in fixed point for sending as ABI and telemetry msgs
  RATES_BFP_OF_REAL(ins_vn.gyro_i, ins_vn.vn_data.gyro);
  float_rmat_ratemult(&body_rate, orientationGetRMat_f(&ins_vn.body_to_imu), &ins_vn.vn_data.gyro); // compute body rates
  stateSetBodyRates_f(&body_rate);   // Set state [rad/s]

  // Attitude [deg]
  static struct FloatQuat imu_quat; // convert from euler to quat
  float_quat_of_eulers(&imu_quat, &ins_vn.vn_data.attitude);
  static struct FloatRMat imu_rmat; // convert from quat to rmat
  float_rmat_of_quat(&imu_rmat, &imu_quat);
  static struct FloatRMat ltp_to_body_rmat; // rotate to body frame
  float_rmat_comp(&ltp_to_body_rmat, &imu_rmat, orientationGetRMat_f(&ins_vn.body_to_imu));
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
  stateSetSpeedNed_f(&ins_vn.vn_data.vel_ned); // set state

  // NED (LTP) acceleration [m/s^2]
  static struct FloatVect3 accel_meas_ltp;// first we need to rotate linear acceleration from imu-frame to body-frame
  float_rmat_transp_vmult(&accel_meas_ltp, orientationGetRMat_f(&ins_vn.body_to_imu), &(ins_vn.vn_data.lin_accel));
  static struct NedCoor_f ltp_accel; // assign to NedCoord_f struct
  VECT3_ASSIGN(ltp_accel, accel_meas_ltp.x, accel_meas_ltp.y, accel_meas_ltp.z);
  stateSetAccelNed_f(&ltp_accel); // then set the states
  ins_vn.ltp_accel_f = ltp_accel;

  // LLA position [rad, rad, m]
  //static struct LlaCoor_f lla_pos; // convert from deg to rad, and from double to float
  ins_vn.lla_pos.lat = RadOfDeg((float)ins_vn.vn_data.pos_lla[0]); // ins_impl.pos_lla[0] = lat
  ins_vn.lla_pos.lon = RadOfDeg((float)ins_vn.vn_data.pos_lla[1]); // ins_impl.pos_lla[1] = lon
  ins_vn.lla_pos.alt = ((float)ins_vn.vn_data.pos_lla[2]); // ins_impl.pos_lla[2] = alt
  LLA_BFP_OF_REAL(gps.lla_pos, ins_vn.lla_pos);
  SetBit(gps.valid_fields, GPS_VALID_POS_LLA_BIT);
  stateSetPositionLla_i(&gps.lla_pos);

  // ECEF velocity
  // TODO: properly implement

  // ECEF position
  gps.ecef_pos.x = stateGetPositionEcef_i()->x;
  gps.ecef_pos.y = stateGetPositionEcef_i()->y;
  gps.ecef_pos.z = stateGetPositionEcef_i()->z;
  SetBit(gps.valid_fields, GPS_VALID_POS_ECEF_BIT);

  // GPS Ground speed
  float speed = sqrt(ins_vn.vn_data.vel_ned.x * ins_vn.vn_data.vel_ned.x + ins_vn.vn_data.vel_ned.y * ins_vn.vn_data.vel_ned.y);
  gps.gspeed = ((uint16_t)(speed * 100));

  // GPS course
  gps.course = (int32_t)(1e7 * (atan2(ins_vn.vn_data.vel_ned.y, ins_vn.vn_data.vel_ned.x)));
  SetBit(gps.valid_fields, GPS_VALID_COURSE_BIT);

  // Because we have not HMSL data from Vectornav, we are using LLA-Altitude
  // as a workaround
  float geoid_h = wgs84_ellipsoid_to_geoid_f(ins_vn.lla_pos.lat, ins_vn.lla_pos.lon);
  gps.hmsl =  (int32_t)((ins_vn.lla_pos.alt - geoid_h)* 1000.0f);
  SetBit(gps.valid_fields, GPS_VALID_HMSL_BIT);

  // Set GPS fix
  gps.fix = ins_vn.vn_data.gps_fix;

  // Set GPS num_sv
  gps.num_sv = ins_vn.vn_data.num_sv;

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

  // update internal states for telemetry purposes
  // TODO: directly convert vectornav output instead of using state interface
  // to support multiple INS running at the same time
  ins_vn.ltp_pos_i = *stateGetPositionNed_i();
  ins_vn.ltp_speed_i = *stateGetSpeedNed_i();
  ins_vn.ltp_accel_i = *stateGetAccelNed_i();
}
