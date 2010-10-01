#include <iostream>
#include <iomanip>

#include <Eigen/Core>

#include "ins_qkf.hpp"
#include <stdint.h>

#include <event.h>
extern "C" {
#include <unistd.h>
#include <time.h>
#include "std.h"
#include "fms/fms_debug.h"
#include "fms/fms_periodic.h"
#include "fms/fms_spi_link.h"
#include "fms/fms_autopilot_msg.h"
#include "firmwares/rotorcraft/imu.h"
#include "fms/libeknav/raw_log.h"
  /* our sensors            */
  struct ImuFloat imu_float;
  struct EcefCoor_i imu_ecef_pos,
										imu_ecef_vel;
  /* raw log */
  static int raw_log_fd;
}

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_float.c"
#include "math/pprz_geodetic_double.c"
#include <math.h>

#define FILTER_OUTPUT_IN_NED 1


/*
 * 
 * Initialization
 * 
 */

/* initial state */

//Toulouse Lat: 	43° 35' 24''		Lon: 	1° 25' 48''
#define TOULOUSE_LATTITUDE UGLY_ANGLE_IN_RADIANS(43,35,24)
#define TOULOUSE_LONGITUDE UGLY_ANGLE_IN_RADIANS(1,25,48)
#define TOULOUSE_HEIGHT 0
//Toulouse Declination is  22' West	and	Inclination 59° 8' Down
#define TOULOUSE_DECLINATION -UGLY_ANGLE_IN_RADIANS(0,22,0)
#define TOULOUSE_INCLINATION -UGLY_ANGLE_IN_RADIANS(59,8,0)

struct LlaCoor_f pos_0_lla;
Vector3d pos_0_ecef;
Vector3d speed_0_ecef;
//  Vector3d orientation(0., 0., 0.);
Vector3d bias_0(0., 0., 0.);

/**
 * how to compute the magnetic field:
 *     http://gsc.nrcan.gc.ca/geomag/field/comp_e.php
 * 
 * online-calculator:
 *     http://geomag.nrcan.gc.ca/apps/mfcal-eng.php
 */
#if 0
#define EARTHS_GEOMAGNETIC_FIELD_NORMED(ref) {	\
    ref.z = sin(TOULOUSE_INCLINATION);			\
	double h = sqrt(1-ref.z*ref.z);				\
	ref.x = h*cos(TOULOUSE_DECLINATION);		\
	ref.y = h*sin(TOULOUSE_DECLINATION);		\
}
#else
#define EARTHS_GEOMAGNETIC_FIELD_NORMED(ref) VECT3_ASSIGN(ref, 0.51292422348174, -0.00331095113378, 0.85842750338526)
#endif

// mean of the measurment data
#define LAB_REFERENCE(ref) VECT3_ASSIGN(ref, -0.22496030821134, 0.70578892222179, 0.67175505729281)

Vector3d reference_direction;

/* initial covariance */
const double pos_cov_0 =  1e4;
const double speed_cov_0 =  3.;
//  const double orientation_cov_0 =  RadOfDeg(5.)*RadOfDeg(5.);
const double bias_cov_0 =  0.447;
const double mag_noise = std::pow(5 / 180.0 * M_PI, 2);

/* system noise      */
const double mag_error =  2.536e-3;
const Vector3d gyro_white_noise(1.1328*1.1328e-4, 0.9192*0.9192e-4, 1.2291*1.2291e-4);
const Vector3d gyro_stability_noise(-1.7605*1.7605e-4,    0.5592*0.5592e-4,    1.1486*1.1486e-4 );
const Vector3d accel_white_noise(2.3707*2.3707e-4,    2.4575*2.4575e-4,    2.5139*2.5139e-4);


/*
 * 
 * HEADERS
 * 
 */


static void main_trick_libevent(void);
static void on_foo_event(int fd, short event __attribute__((unused)), void *arg);
static struct event foo_event;


static void main_rawlog_init(const char* filename);
static void main_rawlog_dump(void);

static void main_init(void);
static void init_ins_state(void);
static void set_reference_direction(void);
static void main_periodic(int my_sig_num);
static uint8_t main_dialog_with_io_proc(void);
static void main_run_ins(uint8_t);



/* Logging */
#define IMU_LOG_FILE "/tmp/log_test3.bin"
#define INS_LOG_FILE "/tmp/log_ins_test3.data"
static void print_estimator_state(double);

/* time measurement */
#define TIMER CLOCK_MONOTONIC 
double absTime(struct timespec);
struct timespec time_diff(struct timespec, struct timespec);

/* Other */
#define UGLY_ANGLE_IN_RADIANS(degree, arcmin, arcsec) ((degree+arcmin/60+arcsec/3600)*M_PI/180)
#define COORDS_AS_VECTOR(coords) Vector3d(coords.x, coords.y, coords.z)
#define RATES_AS_VECTOR(rates) Vector3d(rates.p,rates.q,rates.r)
#define ABS(a) ((a<0)?-a:a)
#define VECTOR_AS_COORDS(coords, vector) { coords.x = vector(0); coords.y = vector(1); coords.z = vector(2);}
