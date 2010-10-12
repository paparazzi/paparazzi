
#include "libeknav_from_log.hpp"


FILE* ins_logfile;		// note: initilaized in init_ins_state

//useless initialization (I hate C++)
static basic_ins_qkf ins = basic_ins_qkf(Vector3d::Zero(), 0, 0, 0,
					 Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero());

// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN

int main(int, char *argv[]) {

  std::cout << "test libeknav 3" << std::endl;
  main_init();
  
  
  int raw_log_fd = open(argv[1], O_RDONLY); 
  
  if (raw_log_fd == -1) {
    perror("opening log\n");
    return -1;
  }
  
  while (1) {
    struct raw_log_entry e;
    ssize_t nb_read = read(raw_log_fd, &e, sizeof(e));
    if (nb_read != sizeof(e)) break;
    
    COPY_RATES_ACCEL_TO_IMU_FLOAT(e);
    COPY_MAG_TO_IMU_FLOAT(e);
    COPY_GPS_TO_IMU(e);
    main_run_ins(e.data_valid);
    print_estimator_state(e.time);
  }
  
  return 0;

}


static void main_init(void) {
	printf("FILTER output will be in ");
	#if FILTER_OUTPUT_IN_NED
		printf("NED\n");
	#else
		printf("ECEF\n");
	#endif
	
	#if UPDATE_WITH_GRAVITY
	printf("the orientation becomes UPDATED with the GRAVITY\n");
  #endif

  init_ins_state();
  set_reference_direction();

}

static void main_run_ins(uint8_t data_valid) {
  
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
  
}




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


/* 		helpstuff	 	*/
/** Logging **/

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
