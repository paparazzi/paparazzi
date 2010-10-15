//#include <iostream>
//#include <iomanip>

#include <Eigen/Core>

#include "ins_qkf.hpp"
#include "paparazzi_eigen_conversion.h"
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
#include "firmwares/rotorcraft/imu.h"
#include "fms/fms_autopilot_msg.h"
#include "fms/libeknav/raw_log.h"
  /* our sensors            */
  struct ImuFloat imu_float;
  struct EcefCoor_i imu_ecef_pos,
										imu_ecef_vel;




/* constants */
/** Compilation-control **/
#define UPDATE_WITH_GRAVITY 1
#define SYNTHETIC_MAG_MODE 0
#define FILTER_OUTPUT_IN_NED 1

#define PRINT_MAG 0
#define PRINT_GPS 1
#define PRINT_EULER_NED 0

/** geodetic **/
//Toulouse Lat: 	43° 35' 24''		Lon: 	1° 25' 48''
#define TOULOUSE_LATTITUDE ARCSEC_ACRMIN_ANGLE_IN_RADIANS(43,35,24)
#define TOULOUSE_LONGITUDE ARCSEC_ACRMIN_ANGLE_IN_RADIANS(1,25,48)
#define TOULOUSE_HEIGHT 0
//Toulouse Declination is  22' West	and	Inclination 59° 8' Down
#define TOULOUSE_DECLINATION -ARCSEC_ACRMIN_ANGLE_IN_RADIANS(0,22,0)
#define TOULOUSE_INCLINATION -ARCSEC_ACRMIN_ANGLE_IN_RADIANS(59,8,0)

/** magnetic field 
 ** how to compute the magnetic field:
 **     http://gsc.nrcan.gc.ca/geomag/field/comp_e.php
 ** 
 ** online-calculator:
 **   http://geomag.nrcan.gc.ca/apps/mfcal-eng.php
 **/
#if 0
#define EARTHS_GEOMAGNETIC_FIELD_NORMED(ref) {	\
  ref.z = sin(TOULOUSE_INCLINATION);						\
	double h = sqrt(1-ref.z*ref.z);								\
	ref.x = h*cos(TOULOUSE_DECLINATION);					\
	ref.y = h*sin(TOULOUSE_DECLINATION);					\
}
#else
//#define EARTHS_GEOMAGNETIC_FIELD_NORMED(ref) VECT3_ASSIGN(ref, 0.51292422348174, -0.00331095113378, 0.85842750338526)
#define EARTHS_GEOMAGNETIC_FIELD_NORMED(ref) VECT3_ASSIGN(ref, 0.51562740288882, -0.05707735220832, 0.85490967783446)
#endif
Vector3d reference_direction;

static void set_reference_direction(void);

/** other **/
#define GRAVITY 9.81
#define MAX_DISTANCE_FROM_GRAVITY_FOR_UPDATE 0.03

/* Initialisation */
static void main_init(void);

/** initial state **/
struct LlaCoor_f pos_0_lla;
Vector3d pos_0_ecef;
Vector3d speed_0_ecef = Vector3d::Zero();
Vector3d bias_0(0., 0., 0.);
static void init_ins_state(void);

/** initial covariance **/
const double pos_cov_0 =  1e4;
const double speed_cov_0 =  3.;
//  const double orientation_cov_0 =  RadOfDeg(5.)*RadOfDeg(5.);
const double bias_cov_0 =  0.447;
const double mag_noise = std::pow(5 / 180.0 * M_PI, 2);


/* system noise	*/
const double   mag_error            = 2.536e-3;
const Vector3d gyro_white_noise     (  1.1328*1.1328e-4,    0.9192*0.9192e-4,    1.2291*1.2291e-4);
const Vector3d gyro_stability_noise ( -1.7605*1.7605e-4,    0.5592*0.5592e-4,    1.1486*1.1486e-4);
const Vector3d accel_white_noise    (  2.3707*2.3707e-4,    2.4575*2.4575e-4,    2.5139*2.5139e-4);
const Vector3d gps_pos_noise        = Vector3d::Ones() *10  *10  ;
const Vector3d gps_speed_noise      = Vector3d::Ones() * 0.1* 0.1;


/* libeknav */
static void main_run_ins(uint8_t);


/* Logging */
static void print_estimator_state(double);
#define INS_LOG_FILE "log_ins_test3.data"



/* Other */

	/** Sensors **/
#define COPY_RATES_ACCEL_TO_IMU_FLOAT(pointer){						\
  RATES_FLOAT_OF_BFP(imu_float.gyro, pointer.gyro);			\
  ACCELS_FLOAT_OF_BFP(imu_float.accel, pointer.accel); 	\
}
#define COPY_MAG_TO_IMU_FLOAT(pointer) MAGS_FLOAT_OF_BFP(imu_float.mag, pointer.mag)
#define COPY_GPS_TO_IMU(pointer){													\
  VECT3_COPY(imu_ecef_pos, pointer.ecef_pos);						\
  VECT3_COPY(imu_ecef_vel, pointer.ecef_vel);						\
}


#define IMU_READY(data_valid) (data_valid & (1<<VI_IMU_DATA_VALID))
#define GPS_READY(data_valid) (data_valid & (1<<VI_GPS_DATA_VALID))
#define MAG_READY(data_valid) (data_valid & (1<<VI_MAG_DATA_VALID))

#define CLOSE_TO_GRAVITY(accel) (ABS(FLOAT_VECT3_NORM(accel)-GRAVITY)<MAX_DISTANCE_FROM_GRAVITY_FOR_UPDATE)

	/** Conversions	**/
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
