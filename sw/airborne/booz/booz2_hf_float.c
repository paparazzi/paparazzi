#include "booz2_hf_float.h"
#include "booz2_ins.h"

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

struct Int32Vect3  b2ins_meas_gps_pos_ned;
struct Int32Vect3  b2ins_meas_gps_speed_ned;


#include "downlink.h"

void b2ins_init(void) {
  INT32_VECT3_ZERO(b2ins_accel_bias);
  b2ins_body_to_imu_eulers.phi   = FILTER_ALIGNMENT_DPHI;
  b2ins_body_to_imu_eulers.theta = FILTER_ALIGNMENT_DTHETA;
  b2ins_body_to_imu_eulers.psi   = FILTER_ALIGNMENT_DPSI;
  BOOZ_IQUAT_OF_EULER(b2ins_body_to_imu_quat, b2ins_body_to_imu_eulers);
  INT32_QUAT_INVERT(b2ins_imu_to_body_quat, b2ins_body_to_imu_quat);

}

#ifdef BYPASS_AHRS
#include "booz_flight_model.h"
#include "6dof.h"
#endif /* BYPASS_AHRS */

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



}

void b2ins_update_gps(void) {

  /* FIXME : with Q_int32_XX_8 we overflow for 256m */
  INT32_VECT3_SCALE_2(b2ins_meas_gps_pos_ned, booz_ins_gps_pos_cm_ned, 
		      IPOS_OF_CM_NUM, IPOS_OF_CM_DEN); 
  INT32_VECT3_SCALE_2(b2ins_meas_gps_speed_ned, booz_ins_gps_speed_cm_s_ned,
		      ISPEED_OF_CM_S_NUM, ISPEED_OF_CM_S_DEN); 

  struct Int64Vect2 scaled_pos_meas;
  VECT2_COPY(scaled_pos_meas, b2ins_meas_gps_pos_ned);
  VECT2_SMUL(scaled_pos_meas, (1<<(B2INS_POS_LTP_FRAC-IPOS_FRAC)), scaled_pos_meas);
  struct Int64Vect3 pos_residual;
  VECT2_DIFF(pos_residual, scaled_pos_meas, b2ins_pos_ltp); 
  struct Int32Vect2 pos_cor_1;
  VECT2_SDIV(pos_cor_1, (1<<9), pos_residual);
  VECT2_ADD(b2ins_pos_ltp, pos_cor_1);
  struct Int32Vect2 speed_cor_1;
  VECT2_SDIV(speed_cor_1, (1<<18), pos_residual);
  VECT2_ADD(b2ins_speed_ltp, speed_cor_1);


#if 1
  struct Int32Vect2 scaled_speed_meas;
  VECT2_SMUL(scaled_speed_meas, (1<<(B2INS_SPEED_LTP_FRAC-ISPEED_RES)), b2ins_meas_gps_speed_ned);
  struct Int32Vect2 speed_residual;
  VECT2_DIFF(speed_residual, scaled_speed_meas, b2ins_speed_ltp);
  struct Int32Vect2 pos_cor_2;
  VECT2_SDIV(pos_cor_2, (1<<2), speed_residual);
  VECT2_ADD(b2ins_pos_ltp, pos_cor_2);
  struct Int32Vect2 speed_cor_2;
  VECT2_SDIV(speed_cor_2, (1<<11), speed_residual);
  VECT2_ADD(b2ins_speed_ltp, speed_cor_2);
#endif  


  
  

}

