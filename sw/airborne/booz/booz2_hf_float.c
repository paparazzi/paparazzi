#include "booz2_hf_float.h"



struct FloatVect3 bhff_accel_bias;
struct FloatVect3 bhff_pos;
struct FloatVect3 bhff_speed_ltp;
struct FloatVect3 bhff_accel_ltp;




void bhff_update(void) {

  struct FloatVect3 accel_body;
  /* unbias accels */
  FLOAT_VECT3_SUB(accel_body, booz2_imu_accel, bhff_accel_bias);
  /* convert to LTP */
  


}
