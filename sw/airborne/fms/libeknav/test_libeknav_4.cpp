
#include "test_libeknav_4.hpp"

#include <stdlib.h>


struct timespec start, prev;
FILE* ins_logfile;		// note: initilaized in init_ins_state


//useless initialization (I hate C++)
static basic_ins_qkf ins = basic_ins_qkf(Vector3d::Zero(), 0, 0, 0,
					 Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero());

// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN

int main(int, char *[]) {

  std::cout << "test libeknav 3" << std::endl;
  clock_gettime(TIMER, &start);
  main_init();
  /* add dev/null as event source so that libevent doesn't die */
  main_trick_libevent();
  
  
  TRACE(TRACE_DEBUG, "%s", "Entering mainloop\n");
  
  /* Enter our mainloop */
  event_dispatch();
  
  TRACE(TRACE_DEBUG, "%s", "leaving mainloop... goodbye!\n");
  
  return 0;

}


static void main_init(void) {

  TRACE(TRACE_DEBUG, "%s", "Starting initialization\n");

  /* Initalize our SPI link to IO processor */
  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return;
  }
  
  /* Initalize the event library */
  event_init();
  
  /* Initalize our ô so accurate periodic timer */
  if (fms_periodic_init(main_periodic)) {
    TRACE(TRACE_ERROR, "%s", "failed to start periodic generator\n");
    return; 
  }
  
  init_ins_state();
  set_reference_direction();
   
  main_rawlog_init(IMU_LOG_FILE);

}


static void main_periodic(int my_sig_num __attribute__ ((unused))) {

  uint8_t data_valid = main_dialog_with_io_proc();
  main_run_ins(data_valid);
  main_rawlog_dump();

}


static uint8_t main_dialog_with_io_proc() {

  struct AutopilotMessageCRCFrame msg_in;
  struct AutopilotMessageCRCFrame msg_out;
  uint8_t crc_valid; 
  
  //  for (uint8_t i=0; i<6; i++) msg_out.payload.msg_down.pwm_outputs_usecs[i] = otp.servos_outputs_usecs[i];
  
  spi_link_send(&msg_out, sizeof(struct AutopilotMessageCRCFrame), &msg_in, &crc_valid);
  
  struct AutopilotMessageVIUp *in = &msg_in.payload.msg_up; 
  RATES_FLOAT_OF_BFP(imu_float.gyro, in->gyro);
  ACCELS_FLOAT_OF_BFP(imu_float.accel, in->accel); 
  
  if(in->valid_sensors & MAG_DATA_VALID){
	  MAGS_FLOAT_OF_BFP(imu_float.mag, in->mag); 
  }
  
  if(in->valid_sensors & GPS_DATA_VALID){
		VECT3_COPY(imu_ecef_pos, in->ecef_pos);
		printf("GPS: %d %d %d\n", imu_ecef_pos.x, imu_ecef_pos.y, imu_ecef_pos.z);
		VECT3_COPY(imu_ecef_vel, in->ecef_vel);
	}
  return in->valid_sensors;
}

static void main_run_ins(uint8_t data_valid) {

  struct timespec now;
  clock_gettime(TIMER, &now);
  
  double dt_imu_freq = 0.001953125; //  1/512; // doesn't work?
  
  ins.predict(RATES_AS_VECTOR(imu_float.gyro), COORDS_AS_VECTOR(imu_float.accel), dt_imu_freq);
  
  if(data_valid & MAG_DATA_VALID){
		ins.obs_vector(reference_direction, COORDS_AS_VECTOR(imu_float.mag), mag_noise);
	}
  
  if(ABS(FLOAT_VECT3_NORM(imu_float.accel)-9.81)<0.03){
		// use the gravity as reference
		ins.obs_vector(ins.avg_state.position.normalized(), COORDS_AS_VECTOR(imu_float.accel), 0.027);
	}
  
  if(data_valid & GPS_DATA_VALID){
    const Vector3d gps_pos_noise = Vector3d::Ones()  *10*10;
    const Vector3d gps_speed_noise = Vector3d::Ones()*0.1*0.1;
		//ins.obs_gps_pv_report(COORDS_AS_VECTOR(imu_ecef_pos)/100, COORDS_AS_VECTOR(imu_ecef_vel)/100, gps_pos_noise, gps_speed_noise);
	}
  
  print_estimator_state(absTime(time_diff(now, start)));
  
  
}




#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

static void main_trick_libevent(void) {

  int fd = open("/dev/ttyS0", O_RDONLY);
  if (fd == -1) {
    TRACE(TRACE_ERROR, "%s", "failed to open /dev/null \n");
    return;
  }
  event_set(&foo_event, fd, EV_READ | EV_PERSIST, on_foo_event, NULL);
  event_add(&foo_event, NULL);

}

static void on_foo_event(int fd __attribute__((unused)), short event __attribute__((unused)), void *arg __attribute__((unused))) {

} 


static void init_ins_state(void){
	
	ins_logfile = fopen(INS_LOG_FILE, "w");
	
	LLA_ASSIGN(pos_0_lla, TOULOUSE_LATTITUDE, TOULOUSE_LONGITUDE, TOULOUSE_HEIGHT)
	struct EcefCoor_f pos_0_ecef_pprz;
	ecef_of_lla_f(&pos_0_ecef_pprz, &pos_0_lla);
	pos_0_ecef = COORDS_AS_VECTOR(pos_0_ecef_pprz);
	
	printf("Starting position\t%f\t%f\t%f\n", pos_0_ecef(0), pos_0_ecef(1), pos_0_ecef(2));
	
	speed_0_ecef = Vector3d::Zero();
	
	ins.avg_state.position = pos_0_ecef;
	ins.avg_state.gyro_bias = Vector3d::Zero();
	ins.avg_state.orientation = Quaterniond::Identity();
	ins.avg_state.velocity = speed_0_ecef;
	
	
	Matrix<double, 12, 1> diag_cov;
	diag_cov << Vector3d::Ones()*bias_cov_0*bias_cov_0,
				Vector3d::Ones()*M_PI*M_PI*0.5,
				Vector3d::Ones()*pos_cov_0*pos_cov_0,
				Vector3d::Ones()*speed_cov_0*speed_cov_0;
	ins.cov = diag_cov.asDiagonal();
	
}

static void set_reference_direction(void){
	DoubleVect3 ref_dir_ned,
				ref_dir_ecef;
	EARTHS_GEOMAGNETIC_FIELD_NORMED(ref_dir_ned);		// the "true" magnetic field
	//LAB_REFERENCE(ref_dir_ned);						// measured in the LAB
	
	DoubleRMat ned2ecef;
	/* copied and modified form pprz_geodetic */
		const double sin_lat = sin(pos_0_lla.lat);
		const double cos_lat = cos(pos_0_lla.lat);
		const double sin_lon = sin(pos_0_lla.lon);
		const double cos_lon = cos(pos_0_lla.lon);
		ned2ecef.m[0] = -sin_lat*cos_lon;
		ned2ecef.m[1] = -sin_lon;
		ned2ecef.m[2] = -cos_lat*cos_lon;
		ned2ecef.m[3] = sin_lat*sin_lon;
		ned2ecef.m[4] = cos_lon;
		ned2ecef.m[5] = -cos_lat*sin_lon;
		ned2ecef.m[6] = cos_lat;
		ned2ecef.m[7] = 0.;
		ned2ecef.m[8] = -sin_lat;
	
	RMAT_VECT3_MUL(ref_dir_ecef, ned2ecef, ref_dir_ned);
	//MAT33_VECT3_TRANSP_MUL(ref_dir_ecef, ned2ecef, ref_dir_ned);
	reference_direction = COORDS_AS_VECTOR(ref_dir_ecef);
	//reference_direction = Vector3d(1, 0, 0);
	std::cout <<"reference direction: " << reference_direction.transpose() << std::endl;
}



/* helpstuff
 * 
 * 
 * 
 */
/* time measurement */

double absTime(struct timespec T){
	return (double)(T.tv_sec + T.tv_nsec*1e-9);
}

struct timespec time_diff(struct timespec end, struct timespec start){
	double difference = absTime(end)-absTime(start);
	struct timespec dT;
	dT.tv_sec = (int)difference;
	dT.tv_nsec = (difference-dT.tv_sec)*1000000000;
	return dT;
}

/* Logging
 * 
 * 
 * 
 */

static void main_rawlog_init(const char* filename) {
  
  raw_log_fd = open(filename, O_WRONLY|O_CREAT, 00644);
  if (raw_log_fd == -1) {
    TRACE(TRACE_ERROR, "failed to open rawlog outfile (%s)\n", filename);
    return;
  }
}

static void main_rawlog_dump(void) {
  struct timespec now;
  clock_gettime(TIMER, &now);
  struct raw_log_entry e;
  
  e.time = absTime(time_diff(now, start));
  RATES_COPY(e.gyro, imu_float.gyro);
  VECT3_COPY(e.accel, imu_float.accel);
  VECT3_COPY(e.mag, imu_float.mag);
  VECT3_COPY(e.ecef_pos, imu_ecef_pos);
  VECT3_COPY(e.ecef_vel, imu_ecef_vel);
  write(raw_log_fd, &e, sizeof(e));

}

static void print_estimator_state(double time) {

#if FILTER_OUTPUT_IN_NED
	
	struct LtpDef_d		current_ltp;
	struct EcefCoor_d pos_ecef,
										cur_pos_ecef,
										cur_vel_ecef;
	struct NedCoor_d	pos_ned,
										vel_ned;
										
	VECTOR_AS_COORDS(pos_ecef,pos_0_ecef);
	VECTOR_AS_COORDS(cur_pos_ecef,ins.avg_state.position);
	VECTOR_AS_COORDS(cur_vel_ecef,ins.avg_state.velocity);
	
	ltp_def_from_ecef_d(&current_ltp, &pos_ecef);
	
	ned_of_ecef_point_d(&pos_ned, &current_ltp, &cur_pos_ecef);
	ned_of_ecef_vect_d(&vel_ned, &current_ltp, &cur_vel_ecef);
	
  int32_t xdd = 0;
  int32_t ydd = 0;
  int32_t zdd = 0;
  
  int32_t xd = vel_ned.x/0.0000019073;
  int32_t yd = vel_ned.y/0.0000019073;
  int32_t zd = vel_ned.z/0.0000019073;
  
  int32_t x = pos_ned.x/0.0039;
  int32_t y = pos_ned.y/0.0039;
  int32_t z = pos_ned.z/0.0039;

  fprintf(ins_logfile, "%f %d BOOZ2_INS2 %d %d %d %d %d %d %d %d %d\n", time, AC_ID, xdd, ydd, zdd, xd, yd, zd, x, y, z);
  
  struct FloatQuat q;
  QUAT_ASSIGN(q, ins.avg_state.orientation.coeffs()(3), ins.avg_state.orientation.coeffs()(0),
	         ins.avg_state.orientation.coeffs()(1), ins.avg_state.orientation.coeffs()(2));
  struct FloatEulers e;
  FLOAT_EULERS_OF_QUAT(e, q);

  fprintf(ins_logfile, "%f %d AHRS_EULER %f %f %f\n", time, AC_ID, e.phi, e.theta, e.psi);
  fprintf(ins_logfile, "%f %d DEBUG_COVARIANCE %f %f %f %f %f %f %f %f %f %f %f %f\n", time, AC_ID,
				sqrt(ins.cov( 0, 0)),  sqrt(ins.cov( 1, 1)),  sqrt(ins.cov( 2, 2)), 
				sqrt(ins.cov( 3, 3)),  sqrt(ins.cov( 4, 4)),  sqrt(ins.cov( 5, 5)), 
				sqrt(ins.cov( 6, 6)),  sqrt(ins.cov( 7, 7)),  sqrt(ins.cov( 8, 8)), 
				sqrt(ins.cov( 9, 9)),  sqrt(ins.cov(10,10)),  sqrt(ins.cov(11,11)));
  fprintf(ins_logfile, "%f %d BOOZ_SIM_GYRO_BIAS %f %f %f\n", time, AC_ID, ins.avg_state.gyro_bias(0), ins.avg_state.gyro_bias(1), ins.avg_state.gyro_bias(2));
	
#else
  int32_t xdd = 0;
  int32_t ydd = 0;
  int32_t zdd = 0;

  int32_t xd = ins.avg_state.velocity(0)/0.0000019073;
  int32_t yd = ins.avg_state.velocity(1)/0.0000019073;
  int32_t zd = ins.avg_state.velocity(2)/0.0000019073;
  
  int32_t x = ins.avg_state.position(0)/0.0039;
  int32_t y = ins.avg_state.position(1)/0.0039;
  int32_t z = ins.avg_state.position(2)/0.0039;

  fprintf(ins_logfile, "%f %d BOOZ2_INS2 %d %d %d %d %d %d %d %d %d\n", time, AC_ID, xdd, ydd, zdd, xd, yd, zd, x, y, z);
  
  struct FloatQuat q;
  QUAT_ASSIGN(q, ins.avg_state.orientation.coeffs()(3), ins.avg_state.orientation.coeffs()(0),
	         ins.avg_state.orientation.coeffs()(1), ins.avg_state.orientation.coeffs()(2));
  struct FloatEulers e;
  FLOAT_EULERS_OF_QUAT(e, q);

  fprintf(ins_logfile, "%f %d AHRS_EULER %f %f %f\n", time, AC_ID, e.phi, e.theta, e.psi);
  fprintf(ins_logfile, "%f %d DEBUG_COVARIANCE %f %f %f %f %f %f %f %f %f %f %f %f\n", time, AC_ID,
				sqrt(ins.cov( 0, 0)),  sqrt(ins.cov( 1, 1)),  sqrt(ins.cov( 2, 2)), 
				sqrt(ins.cov( 3, 3)),  sqrt(ins.cov( 4, 4)),  sqrt(ins.cov( 5, 5)), 
				sqrt(ins.cov( 6, 6)),  sqrt(ins.cov( 7, 7)),  sqrt(ins.cov( 8, 8)), 
				sqrt(ins.cov( 9, 9)),  sqrt(ins.cov(10,10)),  sqrt(ins.cov(11,11)));
  fprintf(ins_logfile, "%f %d BOOZ_SIM_GYRO_BIAS %f %f %f\n", time, AC_ID, ins.avg_state.gyro_bias(0), ins.avg_state.gyro_bias(1), ins.avg_state.gyro_bias(2));
#endif
}

