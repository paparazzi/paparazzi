//#include <iostream>
//#include <iomanip>

#include <Eigen/Core>

#include "ins_qkf.hpp"
#include "paparazzi_eigen_conversion.h"
#include "estimate_attitude.h"
#include "estimate_attitude.c" // should be done by the makefile
#include <stdint.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

//#include <event.h>

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_double.h"
#include <math.h>

#include <unistd.h>
#include <time.h>
#include "std.h"
#include "subsystems/imu.h"
#include "fms/fms_autopilot_msg.h"
#include "fms/libeknav/raw_log.h"
  /* our sensors            */
  struct ImuFloat imu_float;
  double imu_baro_height;
  struct EcefCoor_i imu_ecef_pos,
										imu_ecef_vel;




/* constants */
/** Compilation-control **/
#define UPDATE_WITH_GRAVITY 1
#define FILTER_OUTPUT_IN_NED 1

#define WITH_GPS 1

#define PRINT_MAG 0
#define PRINT_GPS 0
#define PRINT_EULER_NED 0


/** geodetic **/
#define EARTHS_GEOMAGNETIC_FIELD_NORMED(ref) VECT3_ASSIGN(ref, 0.51562740288882, -0.05707735220832, 0.85490967783446)
Vector3d reference_direction;

static void set_reference_direction(void);

/** other **/
#define GRAVITY 9.81
#define MAX_DISTANCE_FROM_GRAVITY_FOR_UPDATE 0.03

/* Initialisation */
static void main_init(void);
static struct raw_log_entry first_entry_after_initialisation(int);

/** initial state **/
struct LlaCoor_f pos_0_lla;
double baro_0_height      = 0;
Vector3d pos_0_ecef       = Vector3d::Zero();
Vector3d speed_0_ecef     = Vector3d::Zero();
Vector3d bias_0           = Vector3d::Zero();
Quaterniond orientation_0 = Quaterniond::Identity();
static void init_ins_state(void);

/** initial covariance **/
//  const double orientation_cov_0 =  RadOfDeg(5.)*RadOfDeg(5.);
Vector3d bias_cov_0         = Vector3d::Ones()*0.447;
Vector3d orientation_cov_0  = Vector3d::Ones()*10*M_PI/180;
Vector3d pos_cov_0          = Vector3d::Ones()*1e4;
Vector3d speed_cov_0        = Vector3d::Ones()*0.1;
//const double mag_noise = std::pow(5 / 180.0 * M_PI, 2);


/* Sensors	*/
// NOTE: Measured during hovering in the air. Movement in the range of a 1 m³ cube with (approx.) max. 0.2 m/s speed.
/// IMU
const Vector3d gyroscope_noise      ( 1.0449e-1,  1.1191e-1,  4.5906e-2 );
//const Vector3d gyroscope_noise      ( 1.0000e-1,  1.0000e-1,  1.0000e-1 );
const Vector3d gyro_stability_noise ( 1.0000e-3,  1.0000e-3,  1.0000e-3 );
const Vector3d accelerometer_noise  ( 2.5457e+0,  1.8242e+0,  1.5660e+0 );
const unsigned short imu_frequency  = 512;

/// MAGnetometer
const Vector3d magnetometer_noise   ( 1.5783e-2,  1.4736e-2,  1.0911e-2 );
const unsigned short mag_frequency  = 13;

/// GPS
const Vector3d gps_pos_noise        ( 6.9348e+0,  1.4180e+0,  7.3982e+0 );
const Vector3d gps_speed_noise      ( 1.4283e+0,  4.2384e-1,  1.5453e+0 );
//const Vector3d gps_speed_noise      ( 1.7935e+0,  4.9214e-1,  6.6279e-1 );
const unsigned short gps_frequency  = 4;
//const Vector3d gps_pos_noise        = Vector3d::Ones() *10  *10  ;

const double baro_noise             =  0.25;
// measured with GPS while climbing approximately 80 m
#define BARO_SCALING                  10.17

//const double   mag_error            = 2.536e-3;
//const Vector3d gyro_white_noise     (  1.1328*1.1328e-4,    0.9192*0.9192e-4,    1.2291*1.2291e-4);
//const Vector3d gyro_stability_noise ( -1.7605*1.7605e-4,    0.5592*0.5592e-4,    1.1486*1.1486e-4);
//const Vector3d accel_white_noise    (  2.3707*2.3707e-4,    2.4575*2.4575e-4,    2.5139*2.5139e-4);


/* libeknav */
static void main_run_from_file(int, struct raw_log_entry);
static void main_run_ins(uint8_t);


/* Logging */
static struct raw_log_entry read_raw_log_entry(int, uint8_t *);
static struct raw_log_entry next_GPS(int);

static void print_estimator_state(double);
#define AC_ID 210
#define INS_LOG_FILE "log_ins_test3.data"



/* Other */
/** Average-Calculation **/
#define MINIMAL_IMU_MEASUREMENTS            1000
#define MINIMAL_MAGNETIC_FIELD_MEASUREMENTS   30
#define MINIMAL_BARO_MEASUREMENTS             30
#define MINIMAL_GPS_MEASUREMENTS              10

#define NOT_ENOUGH_MEASUREMENTS(imu,mag,baro,gps)   (NOT_ENOUGH_IMU_MEASUREMENTS(imu)            || \
                                                     NOT_ENOUGH_MAGNETIC_FIELD_MEASUREMENTS(mag) || \
                                                     NOT_ENOUGH_BARO_MEASUREMENTS(baro)          || \
                                                     NOT_ENOUGH_GPS_MEASUREMENTS(gps)               )
#define NOT_ENOUGH_IMU_MEASUREMENTS(imu)            ((imu) <(MINIMAL_IMU_MEASUREMENTS)           )
#define NOT_ENOUGH_MAGNETIC_FIELD_MEASUREMENTS(mag) ((mag) <(MINIMAL_MAGNETIC_FIELD_MEASUREMENTS))
#define NOT_ENOUGH_BARO_MEASUREMENTS(baro)          ((baro)<(MINIMAL_BARO_MEASUREMENTS)          )
#define NOT_ENOUGH_GPS_MEASUREMENTS(gps)            ((gps) <(MINIMAL_GPS_MEASUREMENTS)           )

#define NEW_MEAN(old_mean, new_observation, index) (((old_mean)*(index-1)+(new_observation))/(index))

	/** Sensors **/
#define INT32_BARO_FRAC 8
#define BARO_FLOAT_OF_BFP(_ai) (FLOAT_OF_BFP((_ai), INT32_BARO_FRAC)*BARO_SCALING)

#define COPY_RATES_ACCEL_TO_IMU_FLOAT(pointer){						\
  RATES_FLOAT_OF_BFP(imu_float.gyro, pointer.gyro);			\
  ACCELS_FLOAT_OF_BFP(imu_float.accel, pointer.accel); 	\
}
#define COPY_MAG_TO_IMU_FLOAT(pointer) MAGS_FLOAT_OF_BFP(imu_float.mag, pointer.mag)
#define COPY_BARO_TO_IMU(pointer) imu_baro_height = -BARO_FLOAT_OF_BFP(pointer.pressure_absolute)
#define COPY_GPS_TO_IMU(pointer){													\
  VECT3_COPY(imu_ecef_pos, pointer.ecef_pos);						\
  VECT3_COPY(imu_ecef_vel, pointer.ecef_vel);						\
}


#define IMU_READY(data_valid) (data_valid & (1<<VI_IMU_DATA_VALID))
#define MAG_READY(data_valid) (data_valid & (1<<VI_MAG_DATA_VALID))
#define BARO_READY(data_valid) (data_valid & (1<<VI_BARO_ABS_DATA_VALID))
#define GPS_READY(data_valid) (data_valid & (1<<VI_GPS_DATA_VALID))

#define CLOSE_TO_GRAVITY(accel) (ABS(FLOAT_VECT3_NORM(accel)-GRAVITY)<MAX_DISTANCE_FROM_GRAVITY_FOR_UPDATE)

	/** Conversions	**/
Quaterniond ecef2body_from_pprz_ned2body(Vector3d, struct DoubleQuat);
struct DoubleEulers sigma_euler_from_sigma_q(struct DoubleQuat, struct DoubleQuat);

	/* copied and modified form pprz_geodetic */
#define NED_TO_ECEF_MAT(lla, mat) {			\
	const double sin_lat = sin(lla.lat);	\
	const double cos_lat = cos(lla.lat);	\
	const double sin_lon = sin(lla.lon);	\
	const double cos_lon = cos(lla.lon);	\
	MAT33_ROW(mat, 0, -sin_lat*cos_lon,	-sin_lon, -cos_lat*cos_lon);	\
	MAT33_ROW(mat, 1,  sin_lat*sin_lon,  cos_lon, -cos_lat*sin_lon);	\
	MAT33_ROW(mat, 2,  cos_lat        ,   0     , -sin_lat        );	\
}





#define DISPLAY_FLOAT_RMAT(text, mat) {					\
    printf("%s\n %f %f %f\n %f %f %f\n %f %f %f\n",text, \
	   mat.m[0], mat.m[1], mat.m[2], mat.m[3], mat.m[4], mat.m[5],	\
	   mat.m[6], mat.m[7], mat.m[8]);				\
  }
#define DISPLAY_DOUBLE_QUAT(text, quat) {				\
    double quat_norm = NORM_VECT4(quat);				\
    printf("%s %f %f %f %f (%f)\n",text, quat.qi, quat.qx, quat.qy, quat.qz, quat_norm); \
  }
#define DISPLAY_DOUBLE_EULERS_DEG(text, _e) {				\
  printf("%s %f %f %f\n",text,  DegOfRad((_e).phi),			\
  DegOfRad((_e).theta), DegOfRad((_e).psi));			\
}

#define DISPLAY_DOUBLE_QUAT_AS_EULERS_DEG(text, quat) {			\
    struct DoubleEulers _fe;						\
    DOUBLE_EULERS_OF_QUAT(_fe, quat);					\
    DISPLAY_DOUBLE_EULERS_DEG(text, _fe);				\
  }
