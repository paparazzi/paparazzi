#include "booz2_filter_attitude_cmpl_euler.h"

#include "booz2_imu.h"
#include "booz2_filter_aligner.h"

#include "airframe.h"
#include "booz_geometry_mixed.h"

struct Booz_ahrs_state booz_ahrs_state;

uint8_t booz2_filter_attitude_status;

struct booz_ieuler booz2_filter_attitude_euler;
struct booz_ivect  booz2_filter_attitude_rate;

struct booz_ieuler booz2_filter_attitude_euler_aligned;


struct booz_ivect  booz2_face_gyro_bias;
struct booz_ieuler booz2_face_measure;
struct booz_ieuler booz2_face_residual;
struct booz_ieuler booz2_face_uncorrected;
struct booz_ieuler booz2_face_corrected;

int32_t booz2_face_reinj_1;

/* 
 * ACC_AMP
 * 9.81 * 2^10 * sin(pi/4) * ACC_AMP = pi/4 * 2^12 * F_UPDATE
 *
 */
#define ACC_AMP 226

#define SAMPLES_NB 256
//static struct booz_ivect lp_gyro_samples[SAMPLES_NB];
//static struct booz_ivect lp_gyro_sum;
static struct booz_ivect lp_accel_samples[SAMPLES_NB];
static struct booz_ivect lp_accel_sum;
static uint8_t samples_idx;


static inline void apply_alignment(void);

void booz2_filter_attitude_init(void) {
  booz2_filter_attitude_status = BOOZ2_FILTER_ATTITUDE_UNINIT;
  BOOZ_IEULER_ZERO(booz2_filter_attitude_euler);
  BOOZ_IVECT_ZERO( booz2_filter_attitude_rate);
  BOOZ_IVECT_ZERO( booz2_face_gyro_bias);
  //  booz2_face_reinj_1 = 1024;
  booz2_face_reinj_1 = 2048;
  samples_idx = 0;
}

void booz2_filter_attitude_align(void) {

  BOOZ_IVECT_COPY( booz2_face_gyro_bias, booz2_filter_aligner_lp_gyro);

  booz2_filter_attitude_status = BOOZ2_FILTER_ATTITUDE_RUNNING;

}


#define F_UPDATE 512

#define PI_INTEG_EULER         (PI_INT * F_UPDATE)
#define TWO_PI_INTEG_EULER (TWO_PI_INT * F_UPDATE)
#define INTEG_EULER_NORMALIZE(_a) {				\
    while (_a >  PI_INTEG_EULER)  _a -= TWO_PI_INTEG_EULER;	\
    while (_a < -PI_INTEG_EULER)  _a += TWO_PI_INTEG_EULER;	\
  }


#define PSI_OF_MAG(_psi, _mag, _phi_est, _theta_est) {			\
									\
    int32_t sphi;							\
    BOOZ_ISIN(sphi, _phi_est);						\
    int32_t cphi;							\
    BOOZ_ICOS(cphi, _phi_est);						\
    int32_t stheta;							\
    BOOZ_ISIN(stheta, _theta_est);					\
    int32_t ctheta;							\
    BOOZ_ICOS(ctheta, _theta_est);					\
									\
    int32_t sphi_stheta = BOOZ_IMULT(sphi, stheta, ITRIG_RES );		\
    int32_t cphi_stheta = BOOZ_IMULT(cphi, stheta, ITRIG_RES );		\
    const int32_t mn =							\
      ctheta      * _mag.x+						\
      sphi_stheta * _mag.y+						\
      cphi_stheta * _mag.z;						\
    const int32_t me =							\
      0           * _mag.x+						\
      cphi        * _mag.y+						\
      -sphi       * _mag.z;						\
    float m_psi = -atan2(me, mn);					\
    _psi = ((m_psi)*(FLOAT_T)(1<<(IANGLE_RES))*F_UPDATE);		\
    									\
  }

/*
 *
 * fc = 1/(2*pi*tau)
 *
 * alpha = dt / ( tau + dt )
 *
 *
 *  y(i) = alpha x(i) + (1-alpha) y(i-1)
 *  or
 *  y(i) = y(i-1) + alpha * (x(i) - y(i-1))
 *
 *
 */



void booz2_filter_attitude_propagate(void) {

  /* low pass accels         */
  BOOZ_IVECT_DIFF(lp_accel_sum, lp_accel_sum, lp_accel_samples[samples_idx]);
  BOOZ_IVECT_COPY(lp_accel_samples[samples_idx], booz2_imu_accel);
  BOOZ_IVECT_SUM(lp_accel_sum, lp_accel_sum, booz2_imu_accel);

  /* unbias gyro             */
  struct booz_ivect  uf_rate;
  BOOZ_IVECT_DIFF(uf_rate, booz2_imu_gyro, booz2_face_gyro_bias);
  /* low pass rate */  
  BOOZ_IVECT_SUM(booz2_filter_attitude_rate, booz2_filter_attitude_rate, uf_rate);
  BOOZ_IVECT_SDIV(booz2_filter_attitude_rate, booz2_filter_attitude_rate, 2);

  /* dumb integrate eulers */
  struct booz_ieuler euler_dot;
  euler_dot.phi   = booz2_filter_attitude_rate.x;
  euler_dot.theta = booz2_filter_attitude_rate.y;
  euler_dot.psi   = booz2_filter_attitude_rate.z;
  BOOZ_IEULER_SUM(booz2_face_uncorrected, booz2_face_uncorrected, euler_dot);
  BOOZ_IEULER_SUM(booz2_face_corrected, booz2_face_corrected, euler_dot);


  /* build a measurement */
  struct booz_ieuler measurement;
  measurement.phi   = -booz2_imu_accel.y * ACC_AMP;
  measurement.theta =  booz2_imu_accel.x * ACC_AMP;
  PSI_OF_MAG(measurement.psi, booz2_imu_mag, 
	     booz2_filter_attitude_euler.phi, booz2_filter_attitude_euler.theta);

  /* low pass it */
  BOOZ_IEULER_SUM(booz2_face_measure, booz2_face_measure, measurement);
  BOOZ_IEULER_SDIV(booz2_face_measure, booz2_face_measure, 2);

  /* compute residual */
  BOOZ_IEULER_DIFF(booz2_face_residual, booz2_face_measure, booz2_face_corrected);
  INTEG_EULER_NORMALIZE(booz2_face_residual.psi);


  struct booz_ieuler correction;
  /* compute a correction */
  BOOZ_IEULER_SDIV(correction, booz2_face_residual, booz2_face_reinj_1);
  /* correct estimation */
  BOOZ_IEULER_SUM(booz2_face_corrected, booz2_face_corrected, correction);
  INTEG_EULER_NORMALIZE(booz2_face_corrected.psi);


  /* scale our result      */
  BOOZ_IEULER_SDIV(booz2_filter_attitude_euler, booz2_face_corrected, F_UPDATE);
  /* convert to quaternion */
  //  BOOZ_IQUAT_OF_EULER(booz2_filter_attitude_quat, booz2_filter_attitude_euler);

  apply_alignment();

}

void booz2_filter_attitude_update(void) {


}


// FIXME

static inline void apply_alignment(void) {
  
#if 0
  booz2_filter_attitude_euler_aligned.phi = 
    booz2_filter_attitude_euler.phi
    + (FILTER_ALIGNMENT_DPSI   * booz2_filter_attitude_euler.theta) >>  IANGLE_RES
    - (FILTER_ALIGNMENT_DTHETA * booz2_filter_attitude_euler.psi)   >>  IANGLE_RES;

  booz2_filter_attitude_euler_aligned.theta = 
    - (FILTER_ALIGNMENT_DPSI   * booz2_filter_attitude_euler.phi)   >>  IANGLE_RES
    + booz2_filter_attitude_euler.theta
    + (FILTER_ALIGNMENT_DPHI * booz2_filter_attitude_euler.psi)     >>  IANGLE_RES;

  booz2_filter_attitude_euler_aligned.psi = 
      (FILTER_ALIGNMENT_DTHETA * booz2_filter_attitude_euler.phi)    >>  IANGLE_RES
    - (FILTER_ALIGNMENT_DPHI   * booz2_filter_attitude_euler.theta)  >>  IANGLE_RES
    + booz2_filter_attitude_euler.psi;
#else

  booz2_filter_attitude_euler_aligned.phi   = booz2_filter_attitude_euler.phi - FILTER_ALIGNMENT_DPHI;
  booz2_filter_attitude_euler_aligned.theta = booz2_filter_attitude_euler.theta - FILTER_ALIGNMENT_DTHETA;
  booz2_filter_attitude_euler_aligned.psi   = booz2_filter_attitude_euler.psi;

#endif


}
