// toytronics_interface.c
// Greg Horn, Joby Robotics 2011

#include "toytronics_interface.h"

#include "math/pprz_algebra_int.h" // Int32Quat etc types
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_int.h" // for Int32Quat stab_att_sp_quat
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_int.h" // for current_stabilization_gains/stabilization_gains
#include "subsystems/ahrs.h" // for attitude struct ahrs
#include "subsystems/radio_control.h" // for rc
#include "subsystems/imu.h" // for imu, duh

const quat_t * get_q_n2b(void){
  static quat_t q_n2b = {1,0,0,0};
  /* q_n2b.q0 = ahrs_float.ltp_to_body_quat.qi; */
  /* q_n2b.q1 = ahrs_float.ltp_to_body_quat.qx; */
  /* q_n2b.q2 = ahrs_float.ltp_to_body_quat.qy; */
  /* q_n2b.q3 = ahrs_float.ltp_to_body_quat.qz; */
  q_n2b.q0 = QUAT1_FLOAT_OF_BFP(ahrs.ltp_to_body_quat.qi);
  q_n2b.q1 = QUAT1_FLOAT_OF_BFP(ahrs.ltp_to_body_quat.qx);
  q_n2b.q2 = QUAT1_FLOAT_OF_BFP(ahrs.ltp_to_body_quat.qy);
  q_n2b.q3 = QUAT1_FLOAT_OF_BFP(ahrs.ltp_to_body_quat.qz);

  quat_normalize(&q_n2b);

  return &q_n2b;
}

const double get_y_accel(void){
  return imu.accel.y/1024.0;
}

const euler_t * get_e_n2b(void){
  static euler_t e_n2b = {0,0,0};
  static quat_t q_n2b_old = {1,0,0,0};

  const quat_t * const q_n2b = get_q_n2b();

  if (memcmp( q_n2b, &q_n2b_old, 4*sizeof(double) )){
    euler321_of_quat( &e_n2b, q_n2b );
    quat_memcpy( &q_n2b_old, q_n2b );
  }

  return &e_n2b;
}

const rc_t * get_rc(void){
  static rc_t rc = {0,0,0,0};

  rc.roll  = ((double)radio_control.values[RADIO_ROLL])/MAX_PPRZ;
  rc.pitch = ((double)radio_control.values[RADIO_PITCH])/MAX_PPRZ;
  rc.yaw   = ((double)radio_control.values[RADIO_YAW])/MAX_PPRZ;

  return &rc;
}

void
set_stabilization_setpoint(const quat_t * const q_n2sp)
{
  stab_att_sp_quat.qi = QUAT1_BFP_OF_REAL(q_n2sp->q0);
  stab_att_sp_quat.qx = QUAT1_BFP_OF_REAL(q_n2sp->q1);
  stab_att_sp_quat.qy = QUAT1_BFP_OF_REAL(q_n2sp->q2);
  stab_att_sp_quat.qz = QUAT1_BFP_OF_REAL(q_n2sp->q3);
}

void
set_stabilization_gains(struct Int32AttitudeGains * new_stabilization_gains){
  if (new_stabilization_gains == NULL){
    current_stabilization_gains = &stabilization_gains;
    return;
  }

  // TODO - scale integral state to give the same integral feedback command with new I gain?
  current_stabilization_gains = new_stabilization_gains;
}
