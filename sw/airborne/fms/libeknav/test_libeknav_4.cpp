
#include "test_libeknav_4.hpp"

#include <stdlib.h>

struct timespec start, prev;
FILE* ins_logfile;		// note: initilaized in init_ins_state
unsigned int counter;

// UGLY CODE WARNING
int16_t baro_measurement;
// UGLY CODE WARNING


#if RUN_FILTER
//useless initialization (I hate C++)
static basic_ins_qkf ins = basic_ins_qkf(Vector3d::Zero(), 0, 0, 0,
					 Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero());

// import most common Eigen types 
#endif
USING_PART_OF_NAMESPACE_EIGEN

int main(int, char *[]) {
	counter = 0;

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
	
	#if RUN_FILTER
		printf("FILTER output will be in ");
		#if FILTER_OUTPUT_IN_NED
			printf("NED\n");
		#else
			printf("ECEF\n");
		#endif
	#else
		printf("Filter wont run\n");
	#endif
	
	#if UPDATE_WITH_GRAVITY
	printf("the orientation becomes UPDATED with the GRAVITY\n");
  #endif

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
  #if RUN_FILTER
  init_ins_state();
  set_reference_direction();
  #endif 
   
  main_rawlog_init(IMU_LOG_FILE);

}


static void main_periodic(int my_sig_num __attribute__ ((unused))) {
	
	counter++;
	if(counter%128 == 0){
		printf("%6.2f s\t", (double)counter/512);
		if(counter%512 == 0){
			printf("\n");
		}
	}

  //uint8_t data_valid = main_dialog_with_io_proc();
  main_dialog_with_io_proc();
  #if RUN_FILTER
  main_run_ins(data_valid);
  #endif
  //main_rawlog_dump(data_valid);

}


//static uint8_t main_dialog_with_io_proc() {
static void main_dialog_with_io_proc() {
	
	DEFINE_AutopilotMessageCRCFrame_IN_and_OUT(message);
  uint8_t crc_valid;
  
  //  for (uint8_t i=0; i<6; i++) msg_out.payload.msg_down.pwm_outputs_usecs[i] = otp.servos_outputs_usecs[i];
  
  spi_link_send(&message_out, sizeof(struct AutopilotMessageCRCFrame), &message_in, &crc_valid);
  main_rawlog_dump(&message_in.payload.msg_up);
  
  
  if(GPS_READY(message_in.payload.msg_up.valid_sensors)){printf("GPS!\n");}
  /*struct AutopilotMessageVIUp *in = &message_in.payload.msg_up; 
  
  if(IMU_READY(in->valid_sensors)){
		COPY_RATES_ACCEL_TO_IMU_FLOAT(in);
  }
  
  if(MAG_READY(in->valid_sensors)){
		COPY_MAG_TO_IMU_FLOAT(in);
    #if PRINT_MAG
    printmag();
    #endif
  }
  
  if(BARO_READY(in->valid_sensors)){
		baro_measurement = in->pressure_absolute;
	}
  
  if(GPS_READY(in->valid_sensors)){
		COPY_GPS_TO_IMU(in);
    #if PRINT_GPS
    printgps();
    #endif
  }
  
  return in->valid_sensors;*/

}

#if RUN_FILTER
static void main_run_ins(uint8_t data_valid) {

  struct timespec now;
  clock_gettime(TIMER, &now);
  
  double dt_imu_freq = 0.001953125; //  1/512; // doesn't work?
  ins.predict(RATES_AS_VECTOR3D(imu_float.gyro), VECT3_AS_VECTOR3D(imu_float.accel), dt_imu_freq);
  
  if(MAG_READY(data_valid)){
		ins.obs_vector(reference_direction, VECT3_AS_VECTOR3D(imu_float.mag), mag_noise);
	}
  
  #if UPDATE_WITH_GRAVITY
  if(CLOSE_TO_GRAVITY(imu_float.accel)){
		// use the gravity as reference
		ins.obs_vector(ins.avg_state.position.normalized(), VECT3_AS_VECTOR3D(imu_float.accel), 1.0392e-3);
	}
  #endif
  
  if(GPS_READY(data_valid)){
		ins.obs_gps_pv_report(VECT3_AS_VECTOR3D(imu_ecef_pos)/100, VECT3_AS_VECTOR3D(imu_ecef_vel)/100, gps_pos_noise, gps_speed_noise);
	}
  
  print_estimator_state(absTime(time_diff(now, start)));
  
}
#endif



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

#if RUN_FILTER
static void init_ins_state(void){
	
	ins_logfile = fopen(INS_LOG_FILE, "w");
	
	LLA_ASSIGN(pos_0_lla, TOULOUSE_LATTITUDE, TOULOUSE_LONGITUDE, TOULOUSE_HEIGHT)
	PPRZ_LLA_TO_EIGEN_ECEF(pos_0_lla, pos_0_ecef);
	
	printf("Starting position\t%f\t%f\t%f\n", pos_0_ecef(0), pos_0_ecef(1), pos_0_ecef(2));
	
	ins.avg_state.position    = pos_0_ecef;
	ins.avg_state.gyro_bias   = Vector3d::Zero();
	ins.avg_state.orientation = Quaterniond::Identity();
	ins.avg_state.velocity    = speed_0_ecef;
	
	
	Matrix<double, 12, 1> diag_cov;
	diag_cov << Vector3d::Ones() * bias_cov_0  * bias_cov_0 ,
							Vector3d::Ones() *  M_PI*0.5   *  M_PI*0.5  ,
							Vector3d::Ones() *  pos_cov_0  *  pos_cov_0 ,
							Vector3d::Ones() * speed_cov_0 * speed_cov_0;
	ins.cov = diag_cov.asDiagonal();
	
}

static void set_reference_direction(void){
	struct NedCoor_d	ref_dir_ned;
	struct EcefCoor_d pos_0_ecef_pprz,
										ref_dir_ecef;
	EARTHS_GEOMAGNETIC_FIELD_NORMED(ref_dir_ned);
	
	struct LtpDef_d current_ltp;
	VECTOR_AS_VECT3(pos_0_ecef_pprz, pos_0_ecef);
	ltp_def_from_ecef_d(&current_ltp, &pos_0_ecef_pprz);
	ecef_of_ned_vect_d(&ref_dir_ecef, &current_ltp, &ref_dir_ned);
	
	//		THIS SOMEWHERE ELSE!
	DoubleQuat initial_orientation;
	FLOAT_QUAT_ZERO(initial_orientation);
	ENU_NED_transformation(current_ltp.ltp_of_ecef);
	DOUBLE_QUAT_OF_RMAT(initial_orientation, current_ltp.ltp_of_ecef);
	ins.avg_state.orientation = DOUBLEQUAT_AS_QUATERNIOND(initial_orientation);
	//		THIS SOMEWHERE ELSE! (END)
	
	// old transformation:
	//struct DoubleRMat ned2ecef;
	//NED_TO_ECEF_MAT(pos_0_lla, ned2ecef.m);
	//RMAT_VECT3_MUL(ref_dir_ecef, ned2ecef, ref_dir_ned);
	
	reference_direction = VECT3_AS_VECTOR3D(ref_dir_ecef).normalized();
	//reference_direction = Vector3d(1, 0, 0);
	std::cout <<"reference direction NED : " << VECT3_AS_VECTOR3D(ref_dir_ned).transpose() << std::endl;
	std::cout <<"reference direction ECEF: " << reference_direction.transpose() << std::endl;
}
#endif


/* 		helpstuff	 	*/
/** tiny little functions **/
void printmag(void){
	printf("MAG: %f %f %f\n", imu_float.mag.x, imu_float.mag.y, imu_float.mag.z);
}

void printgps(void){
	printf("GPS: %d %d %d\n", imu_ecef_pos.x, imu_ecef_pos.y, imu_ecef_pos.z);
}


/** time measurement **/

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

/** Logging **/

static void main_rawlog_init(const char* filename) {
  
  raw_log_fd = open(filename, O_WRONLY|O_CREAT|O_TRUNC, 00644);
  if (raw_log_fd == -1) {
    TRACE(TRACE_ERROR, "failed to open rawlog outfile (%s)\n", filename);
    return;
  }
}

//static void main_rawlog_dump(uint8_t data_valid) {
static void main_rawlog_dump(struct AutopilotMessageVIUp* current_message) {
	struct timespec now;
  clock_gettime(TIMER, &now);
  struct raw_log_entry e;
  e.time = absTime(time_diff(now, start));
  memcpy(&e.message, current_message, sizeof(*current_message));
  
  write(raw_log_fd, &e, sizeof(e));
  /*struct raw_log_entry e;
  
  e.time = absTime(time_diff(now, start));
  e.message = current_message;
  RATES_COPY(e.gyro, imu_float.gyro);
  VECT3_COPY(e.accel, imu_float.accel);
  VECT3_COPY(e.mag, imu_float.mag);
  VECT3_COPY(e.ecef_pos, imu_ecef_pos);
  VECT3_COPY(e.ecef_vel, imu_ecef_vel);
  e.pressure_absolute = baro_measurement;
  e.data_valid = data_valid;
  write(raw_log_fd, &e, sizeof(e));
	*/
}

#if RUN_FILTER
static void print_estimator_state(double time) {

#if FILTER_OUTPUT_IN_NED
	
	struct LtpDef_d		current_ltp;
	struct EcefCoor_d pos_ecef,
										cur_pos_ecef,
										cur_vel_ecef;
	struct NedCoor_d	pos_ned,
										vel_ned;
										
	struct DoubleQuat q_ecef2body,
										q_ecef2enu,
										q_enu2body,
										q_ned2enu,
										q_ned2body;
										
	VECTOR_AS_VECT3(pos_ecef,pos_0_ecef);
	VECTOR_AS_VECT3(cur_pos_ecef,ins.avg_state.position);
	VECTOR_AS_VECT3(cur_vel_ecef,ins.avg_state.velocity);
	
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
  
  QUAT_ASSIGN(q_ecef2body, ins.avg_state.orientation.w(), -ins.avg_state.orientation.x(),
	         -ins.avg_state.orientation.y(), -ins.avg_state.orientation.z());
  QUAT_ASSIGN(q_ned2enu, 0, M_SQRT1_2, M_SQRT1_2, 0);
  
  FLOAT_QUAT_OF_RMAT(q_ecef2enu, current_ltp.ltp_of_ecef);
	FLOAT_QUAT_INV_COMP(q_enu2body, q_ecef2enu, q_ecef2body);		// q_enu2body = q_ecef2body * (q_ecef2enu)^*
  FLOAT_QUAT_COMP(q_ned2body, q_ned2enu, q_enu2body)					// q_ned2body = q_enu2body * q_ned2enu

  
  struct FloatEulers e;
  FLOAT_EULERS_OF_QUAT(e, q_ned2body);
	
	#if PRINT_EULER_NED
		printf("EULER % 6.1f % 6.1f % 6.1f\n", e.phi*180*M_1_PI, e.theta*180*M_1_PI, e.psi*180*M_1_PI);
	#endif
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
  
  struct FloatQuat q_ecef2body;
  QUAT_ASSIGN(q_ecef2body, ins.avg_state.orientation.w(), ins.avg_state.orientation.x(),
	         ins.avg_state.orientation.y(), ins.avg_state.orientation.z());
  struct FloatEulers e_ecef2body;
  FLOAT_EULERS_OF_QUAT(e_ecef2body, q_ecef2body);

  fprintf(ins_logfile, "%f %d AHRS_EULER %f %f %f\n", time, AC_ID, e_ecef2body.phi, e_ecef2body.theta, e_ecef2body.psi);
  fprintf(ins_logfile, "%f %d DEBUG_COVARIANCE %f %f %f %f %f %f %f %f %f %f %f %f\n", time, AC_ID,
				sqrt(ins.cov( 0, 0)),  sqrt(ins.cov( 1, 1)),  sqrt(ins.cov( 2, 2)), 
				sqrt(ins.cov( 3, 3)),  sqrt(ins.cov( 4, 4)),  sqrt(ins.cov( 5, 5)), 
				sqrt(ins.cov( 6, 6)),  sqrt(ins.cov( 7, 7)),  sqrt(ins.cov( 8, 8)), 
				sqrt(ins.cov( 9, 9)),  sqrt(ins.cov(10,10)),  sqrt(ins.cov(11,11)));
  fprintf(ins_logfile, "%f %d BOOZ_SIM_GYRO_BIAS %f %f %f\n", time, AC_ID, ins.avg_state.gyro_bias(0), ins.avg_state.gyro_bias(1), ins.avg_state.gyro_bias(2));
#endif
}
#endif
