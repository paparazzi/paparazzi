#include "booz2_hf_float.h"

#include "booz2_imu.h"
#include "booz2_filter_attitude.h"
#include "booz_geometry_mixed.h"


struct Int32Vect3 b2ins_accel_bias;
struct Int32Vect3 b2ins_accel_ltp;
struct Int32Vect3 b2ins_speed_ltp;
struct Int64Vect3 b2ins_pos_ltp;

struct Int32Eulers b2ins_body_to_imu_eulers;
struct Int32Quat   b2ins_body_to_imu_quat;
struct Int32Quat   b2ins_imu_to_body_quat;

struct Int32Vect2  b2ins_ltp_meas_gps;


#include "downlink.h"

void b2ins_init(void) {
  INT32_VECT3_ZERO(b2ins_accel_bias);
  b2ins_body_to_imu_eulers.phi   = FILTER_ALIGNMENT_DPHI;
  b2ins_body_to_imu_eulers.theta = FILTER_ALIGNMENT_DTHETA;
  b2ins_body_to_imu_eulers.psi   = FILTER_ALIGNMENT_DPSI;
  BOOZ_IQUAT_OF_EULER(b2ins_body_to_imu_quat, b2ins_body_to_imu_eulers);
  INT32_QUAT_INVERT(b2ins_imu_to_body_quat, b2ins_body_to_imu_quat);

}

#include "booz_flight_model.h"
#include "6dof.h"
void b2ins_propagate(void) {

#ifdef BYPASS_AHRS
  booz2_filter_attitude_quat_aligned.qi = BOOZ_INT_OF_FLOAT(bfm.quat->ve[QUAT_QI], IQUAT_RES);
  booz2_filter_attitude_quat_aligned.qx = BOOZ_INT_OF_FLOAT(bfm.quat->ve[QUAT_QX], IQUAT_RES);
  booz2_filter_attitude_quat_aligned.qy = BOOZ_INT_OF_FLOAT(bfm.quat->ve[QUAT_QY], IQUAT_RES);
  booz2_filter_attitude_quat_aligned.qz = BOOZ_INT_OF_FLOAT(bfm.quat->ve[QUAT_QZ], IQUAT_RES);
#endif /* BYPASS_AHRS */


  struct Int32Vect3 accel_imu;
  /* unbias accelerometers */
  INT32_VECT3_DIFF(accel_imu, booz2_imu_accel, b2ins_accel_bias);
  /* convert to LTP */
  struct Int32Quat body_to_ltp;
  BOOZ_IQUAT_INVERT(body_to_ltp, booz2_filter_attitude_quat_aligned);
  struct Int32Quat imu_to_ltp;
  INT32_QUAT_MULT(imu_to_ltp, body_to_ltp, b2ins_imu_to_body_quat);
  BOOZ_IQUAT_VMULT(b2ins_accel_ltp, imu_to_ltp, accel_imu);
  /* correct for gravity */
  b2ins_accel_ltp.z += BOOZ_ACCEL_I_OF_F(9.81);

  /* propagate speed */
  b2ins_speed_ltp.x += b2ins_accel_ltp.x;
  b2ins_speed_ltp.y += b2ins_accel_ltp.y;
  b2ins_speed_ltp.z += b2ins_accel_ltp.z;


  /* propagate position */
  b2ins_pos_ltp.x += b2ins_speed_ltp.x;
  b2ins_pos_ltp.y += b2ins_speed_ltp.y;
  b2ins_pos_ltp.z += b2ins_speed_ltp.z;



  RunOnceEvery(10, {
      struct Int32Vect3 pos_low_res;			\
      pos_low_res.x = (int32_t)(b2ins_pos_ltp.x>> 20);	\
      pos_low_res.y = (int32_t)(b2ins_pos_ltp.y>> 20);	\
      pos_low_res.z = (int32_t)(b2ins_pos_ltp.z>> 20);	\
      DOWNLINK_SEND_BOOZ2_INS2(&b2ins_accel_ltp.x,	\
			       &b2ins_accel_ltp.y,	\
			       &b2ins_accel_ltp.z,	\
			       &b2ins_speed_ltp.x,	\
			       &b2ins_speed_ltp.y,	\
			       &b2ins_speed_ltp.z,	\
			       &pos_low_res.x,		\
			       &pos_low_res.y,		\
			       &pos_low_res.z		\
			       );			\
    });


}

void b2ins_update_gps(void) {

  b2ins_ltp_meas_gps.x = 0;
  b2ins_ltp_meas_gps.y = 0;



}

