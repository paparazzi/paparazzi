
#include "libeknav_from_log.hpp"


FILE* ins_logfile;		// note: initilaized in init_ins_state

//useless initialization (I hate C++)
static basic_ins_qkf ins = basic_ins_qkf(Vector3d::Zero(), 0, 0, 0,
					 Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero());

// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN

int main(int, char *argv[]) {

  printf("==============================\nRunning libeknav from File...\n==============================\n");
  
  int raw_log_fd = open(argv[1], O_RDONLY); 
  
  if (raw_log_fd == -1) {
    perror("opening log\n");
    return -1;
  }
  
  printf("Initialisation...\n");
  struct raw_log_entry e = first_entry_after_initialisation(raw_log_fd);
  main_init();
  printf("Running filter from file...\n");
  main_run_from_file(raw_log_fd, next_GPS(raw_log_fd));
  
  printf("Finished\n");
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

static struct raw_log_entry first_entry_after_initialisation(int file_descriptor){
  int        imu_measurements = 0,      // => Gyro + Accel
    magnetometer_measurements = 0,
             gps_measurements = 0;      // only the position
  
  struct DoubleMat33 attitude_profile_matrix;
  struct Orientation_Measurement  gravity,
                                  magneto,
                                  fake;  
  struct DoubleQuat q_ned2body;
  
  /* Prepare the attitude profile matrix */
  FLOAT_MAT33_ZERO(attitude_profile_matrix);
  
  /* set the gravity measurement */
  VECT3_ASSIGN(gravity.reference_direction, 0,0,1);
  gravity.weight_of_the_measurement = 1;
  
  /* set the magneto - measurement */
  EARTHS_GEOMAGNETIC_FIELD_NORMED(magneto.reference_direction);
  magneto.weight_of_the_measurement = 1;
    
  uint8_t read_ok = 1;
  struct raw_log_entry e = next_GPS(file_descriptor);
  
  while( (read_ok) && NOT_ENOUGH_MEASUREMENTS(imu_measurements, magnetometer_measurements, gps_measurements) ){
    if(IMU_READY(e.data_valid)){
      imu_measurements++;
      
      // update the estimated bias
      bias_0 = NEW_MEAN(bias_0, RATES_AS_VECTOR3D(e.gyro), imu_measurements);
      
      // update the attitude profile matrix
      VECT3_COPY(gravity.measured_direction,e.accel);
      add_orientation_measurement(&attitude_profile_matrix, gravity);
    }
    if(MAG_READY(e.data_valid)){
      magnetometer_measurements++;
      // update the attitude profile matrix
      VECT3_COPY(magneto.measured_direction,e.mag);
      add_orientation_measurement(&attitude_profile_matrix, magneto);
      
      // now, generate fake measurement with the last gravity measurement
      fake = fake_orientation_measurement(gravity, magneto);
      add_orientation_measurement(&attitude_profile_matrix, fake);
    }
    if(GPS_READY(e.data_valid)){
      gps_measurements++;
      // update the estimated bias
      pos_0_ecef = NEW_MEAN(pos_0_ecef, VECT3_AS_VECTOR3D(e.ecef_pos)/100, gps_measurements);
    }
    
    e = read_raw_log_entry(file_descriptor, &read_ok);
  }
  q_ned2body = estimated_attitude(attitude_profile_matrix, 1000, 1e-6);
	orientation_0 = ecef2body_from_pprz_ned2body(pos_0_ecef,q_ned2body);
  return e;
}

static void main_run_from_file(int file_descriptor, struct raw_log_entry first_entry){
  struct raw_log_entry e = first_entry;
  uint8_t read_ok = 1;
  while (read_ok) {
    main_run_ins(e.data_valid);
    print_estimator_state(e.time);
    e = read_raw_log_entry(file_descriptor, &read_ok);
  }
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
	
	ins.avg_state.position    = pos_0_ecef;
	ins.avg_state.gyro_bias   = bias_0;
	ins.avg_state.orientation = orientation_0;
	ins.avg_state.velocity    = speed_0_ecef;
  
  struct DoubleQuat ecef2body;
  struct DoubleEulers eu_ecef2body;
  QUATERNIOND_AS_DOUBLEQUAT(ecef2body, orientation_0);
  DOUBLE_EULERS_OF_QUAT(eu_ecef2body, ecef2body);

  
	printf("Initial state\n\n");
  printf("Bias        % 7.2f\n", bias_0(0)*180*M_1_PI);
  printf("Bias        % 7.2f\n", bias_0(1)*180*M_1_PI);
  printf("Bias        % 7.2f\n", bias_0(2)*180*M_1_PI);
  printf("\n");
  printf("Orientation % 7.2f\n", orientation_0.w());
  printf("Orientation % 7.2f %7.2f\n", orientation_0.x(),   eu_ecef2body.phi*180*M_1_PI);
  printf("Orientation % 7.2f %7.2f\n", orientation_0.y(), eu_ecef2body.theta*180*M_1_PI);
  printf("Orientation % 7.2f %7.2f\n", orientation_0.z(),   eu_ecef2body.psi*180*M_1_PI); 
  printf("\n");
  printf("Position    % 9.0f\n", pos_0_ecef(0));
  printf("Position    % 9.0f\n", pos_0_ecef(1));
  printf("Position    % 9.0f\n", pos_0_ecef(2));
  printf("\n");
  printf("Velocity    % 7.2f\n", speed_0_ecef(0));
  printf("Velocity    % 7.2f\n", speed_0_ecef(1));
  printf("Velocity    % 7.2f\n", speed_0_ecef(2));
  printf("\n");
	
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
	
	
	reference_direction = VECT3_AS_VECTOR3D(ref_dir_ecef).normalized();
}


/* 		helpstuff	 	*/
/** Transformation **/
Quaterniond ecef2body_from_pprz_ned2body(Vector3d ecef_pos, struct DoubleQuat q_ned2body){
  Quaterniond       ecef2body;
  struct LtpDef_d   current_ltp;
	struct EcefCoor_d ecef_pos_pprz;
  struct DoubleQuat q_ecef2enu,
                    q_ecef2ned,
                    q_ecef2body;
  
	VECTOR_AS_VECT3(ecef_pos_pprz, ecef_pos);
  ltp_def_from_ecef_d(&current_ltp, &ecef_pos_pprz);
  DOUBLE_QUAT_OF_RMAT(q_ecef2enu, current_ltp.ltp_of_ecef);
  QUAT_ENU_FROM_TO_NED(q_ecef2enu, q_ecef2ned);
  
  FLOAT_QUAT_COMP(q_ecef2body, q_ecef2ned, q_ned2body);
  
  return DOUBLEQUAT_AS_QUATERNIOND(q_ecef2body);
}


/** Logging **/

static struct raw_log_entry read_raw_log_entry(int file_descriptor, uint8_t *read_ok){
  struct raw_log_entry e;
  ssize_t nb_read = read(file_descriptor, &e, sizeof(e));
  *read_ok =  (nb_read == sizeof(e));
  
  COPY_RATES_ACCEL_TO_IMU_FLOAT(e);
  COPY_MAG_TO_IMU_FLOAT(e);
  COPY_GPS_TO_IMU(e);
  return e;
}

static struct raw_log_entry next_GPS(int file_descriptor){
  uint8_t read_ok;
  struct raw_log_entry e = read_raw_log_entry(file_descriptor, &read_ok);
  while ((read_ok)&&(!GPS_READY(e.data_valid))) {
    e = read_raw_log_entry(file_descriptor, &read_ok);
  }
  return e;
}

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
