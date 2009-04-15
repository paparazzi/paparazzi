#include "rdyb_ahrs.h"


#include <stdlib.h>

extern struct RdybAhrs* ahrs_new(void) {

  struct RdybAhrs* me = malloc(sizeof(struct RdybAhrs));

  FloatQuatZero(me->ltp2imu_quat);
  FloatRMatZero(me->ltp2imu_rmat);
  FloatEulersZero(me->ltp2imu_euler);
  FloatVect3Zero(me->bias);
  FloatVect3Zero(me->imu_rate);

  return me;
  
}

#define DT_PROPAGATE (1./512.)

void ahrs_propagate(struct RdybAhrs* me, struct FloatVect3* gyro) {

  // compute unbiased rate
  Vect3Diff(me->imu_rate, *gyro, me->bias);
  //Vect3Copy(me->imu_rate, *gyro);

  //
  // Propagate state
  //
  
  // time derivative of our quaternion 
  struct FloatQuat quat_dot;
  quat_dot.qi = 0.5 * ( -me->imu_rate.x * me->ltp2imu_quat.qx +  
                        -me->imu_rate.y * me->ltp2imu_quat.qy +
                        -me->imu_rate.z * me->ltp2imu_quat.qz );
 
  quat_dot.qx = 0.5 * (  me->imu_rate.x * me->ltp2imu_quat.qi +  
                         me->imu_rate.z * me->ltp2imu_quat.qy +
                        -me->imu_rate.y * me->ltp2imu_quat.qz);
 
  quat_dot.qy = 0.5 * (  me->imu_rate.y * me->ltp2imu_quat.qi +  
                        -me->imu_rate.z * me->ltp2imu_quat.qx +
                         me->imu_rate.x * me->ltp2imu_quat.qz);
 
  quat_dot.qz = 0.5 * (  me->imu_rate.z * me->ltp2imu_quat.qi +  
                         me->imu_rate.y * me->ltp2imu_quat.qx +
                        -me->imu_rate.x * me->ltp2imu_quat.qy);

  // add a lagrange mutiplier to constrain norm
  const float nq = FloatQuatNorm(me->ltp2imu_quat);
  const float k_lagrange = 1.;
  quat_dot.qi += k_lagrange * ( 1.-nq ) * me->ltp2imu_quat.qi;
  quat_dot.qx += k_lagrange * ( 1.-nq ) * me->ltp2imu_quat.qx;
  quat_dot.qy += k_lagrange * ( 1.-nq ) * me->ltp2imu_quat.qy;
  quat_dot.qz += k_lagrange * ( 1.-nq ) * me->ltp2imu_quat.qz;
  
  // zero order integration
  me->ltp2imu_quat.qi += quat_dot.qi * DT_PROPAGATE;
  me->ltp2imu_quat.qx += quat_dot.qx * DT_PROPAGATE;
  me->ltp2imu_quat.qy += quat_dot.qy * DT_PROPAGATE;
  me->ltp2imu_quat.qz += quat_dot.qz * DT_PROPAGATE;


  //
  // Propagate covariance
  //
  



  FloatEulersOfQuat(me->ltp2imu_euler, me->ltp2imu_quat);

}


void ahrs_update(struct RdybAhrs* me, struct FloatVect3* accel, struct FloatVect3* mag) {

  me->measurement.phi = -atan2f(accel->y, -accel->z); 
  const float norm_g = FloatVect3Norm(*accel);
  me->measurement.theta = asinf(accel->x/norm_g);

}

