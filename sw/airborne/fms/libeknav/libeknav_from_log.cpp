
#include "libeknav_from_log.hpp"

// NOTE: This in the headfile?
struct LtpDef_d current_ltp;
// NOTE-END

unsigned int entry_counter;

FILE* ins_logfile;		// note: initilaized in init_ins_state

//useless initialization (I hate C++)
static basic_ins_qkf ins = basic_ins_qkf(Vector3d::Zero(), 0, 0, 0,
					 gyro_stability_noise,gyroscope_noise, accelerometer_noise);

// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN

int main(int, char *argv[]) {
  
  entry_counter = 0;
  printf("==============================\nRunning libeknav from File...\n==============================\n");
  
  int raw_log_fd = open(argv[1], O_RDONLY); 
  
  if (raw_log_fd == -1) {
    perror("opening log\n");
    return -1;
  }
  
  printf("Initialisation...\n");
  struct raw_log_entry e = first_entry_after_initialisation(raw_log_fd);
  printf("Starting at t = %5.2f s\n", e.time);
  printf("entry counter: %i\n", entry_counter);
  main_init();
  printf("Running filter from file...\n");
  main_run_from_file(raw_log_fd, e);
  
  printf("Finished\n");
  return 0;

}


static void main_init(void) {
	printf("FILTER output will be in ");
	#if FILTER_OUTPUT_IN_NED
		printf("NED\n");
  #else /* FILTER_OUTPUT_IN_ECEF */
		printf("ECEF\n");
	#endif /* FILTER_OUTPUT_IN_NED / ECEF */
	
	#if UPDATE_WITH_GRAVITY
	printf("the orientation becomes UPDATED with the GRAVITY\n");
  #endif /* UPDATE_WITH_GRAVITY */

  init_ins_state();
  set_reference_direction();

}

static struct raw_log_entry first_entry_after_initialisation(int file_descriptor){
  int        imu_measurements = 0,      // => Gyro + Accel
    magnetometer_measurements = 0,
            baro_measurements = 0,
             gps_measurements = 0;      // only the position
  
  struct DoubleMat33 attitude_profile_matrix, sigmaB;   // the attitude profile matrix is often called "B"
  struct Orientation_Measurement  gravity,
                                  magneto,
                                  fake;  
  struct DoubleQuat q_ned2body, sigma_q;
  
  /* Prepare the attitude profile matrix */
  FLOAT_MAT33_ZERO(attitude_profile_matrix);
  FLOAT_MAT33_ZERO(sigmaB);
  
  // for faster converging, but probably more rounding error
  #define MEASUREMENT_WEIGHT_SCALE 10
  
  /* set the gravity measurement */
  VECT3_ASSIGN(gravity.reference_direction, 0,0,-1);
  gravity.weight_of_the_measurement = MEASUREMENT_WEIGHT_SCALE/(double)(imu_frequency);    // originally 1/(imu_frequency*gravity.norm()
  //gravity.weight_of_the_measurement = 1;
  
  /* set the magneto - measurement */
  EARTHS_GEOMAGNETIC_FIELD_NORMED(magneto.reference_direction);
  magneto.weight_of_the_measurement = MEASUREMENT_WEIGHT_SCALE/(double)(mag_frequency);    // originally 1/(mag_frequency*reference_direction.norm()
  //magneto.weight_of_the_measurement = 1;
  
  uint8_t read_ok;
  #if WITH_GPS
  struct raw_log_entry e = next_GPS(file_descriptor);
  #else /* WITH_GPS */
  struct raw_log_entry e = read_raw_log_entry(file_descriptor, &read_ok);
  pos_0_ecef = Vector3d(4627578.56, 119659.25, 4373248.00);
  pos_cov_0 = Vector3d::Ones()*100;
  speed_0_ecef    = Vector3d::Zero();
  speed_cov_0 = Vector3d::Ones();
  #endif /* WITH_GPS */
  
  #ifdef EKNAV_FROM_LOG_DEBUG
    int imu_ready = 0, 
        mag_ready = 0,
        baro_ready = 0,
        gps_ready = 0;
  #endif /* EKNAV_FROM_LOG_DEBUG */
  
  for(read_ok = 1; (read_ok) && NOT_ENOUGH_MEASUREMENTS(imu_measurements, magnetometer_measurements, baro_measurements, gps_measurements); e = read_raw_log_entry(file_descriptor, &read_ok)){
    if(IMU_READY(e.message.valid_sensors)){
      imu_measurements++;
      
      // update the estimated bias
      bias_0 = NEW_MEAN(bias_0, RATES_BFP_AS_VECTOR3D(e.message.gyro), imu_measurements);
      
      // update the attitude profile matrix
      ACCELS_FLOAT_OF_BFP(gravity.measured_direction,e.message.accel);
      add_orientation_measurement(&attitude_profile_matrix, gravity);
    }
    if(MAG_READY(e.message.valid_sensors)){
      magnetometer_measurements++;
      // update the attitude profile matrix
      MAGS_FLOAT_OF_BFP(magneto.measured_direction,e.message.mag);
      add_orientation_measurement(&attitude_profile_matrix, magneto);
      
      // now, generate fake measurement with the last gravity measurement
      fake = fake_orientation_measurement(gravity, magneto);
      add_orientation_measurement(&attitude_profile_matrix, fake);
    }
    if(BARO_READY(e.message.valid_sensors)){
      baro_measurements++;
      // TODO: Fix it!
      //NEW_MEAN(baro_0_height, BARO_FLOAT_OF_BFP(e.message.pressure_absolute), baro_measurements);
      baro_0_height = (baro_0_height*(baro_measurements-1)+BARO_FLOAT_OF_BFP(e.message.pressure_absolute))/baro_measurements;
    }
    if(GPS_READY(e.message.valid_sensors)){
      gps_measurements++;
      // update the estimated bias
      pos_0_ecef = NEW_MEAN(pos_0_ecef, VECT3_AS_VECTOR3D(e.message.ecef_pos)/100, gps_measurements);
      speed_0_ecef = NEW_MEAN(speed_0_ecef, VECT3_AS_VECTOR3D(e.message.ecef_vel)/100, gps_measurements);
    }
    
    #ifdef EKNAV_FROM_LOG_DEBUG
    if(imu_ready==0){
      if(!NOT_ENOUGH_IMU_MEASUREMENTS(imu_measurements)){
        printf("IMU READY %3i %3i %3i %3i\n", imu_measurements, magnetometer_measurements, baro_measurements, gps_measurements);
        imu_ready = 1;
      }
    }
    if(mag_ready==0){
      if(!NOT_ENOUGH_MAGNETIC_FIELD_MEASUREMENTS(magnetometer_measurements)){
        printf("MAG READY %3i %3i %3i %3i\n", imu_measurements, magnetometer_measurements, baro_measurements, gps_measurements);
        mag_ready = 1;
      }
    }
    if(baro_ready==0){
      if(!NOT_ENOUGH_BARO_MEASUREMENTS(baro_measurements)){
        printf("BARO READY %3i %3i %3i %3i\n", imu_measurements, magnetometer_measurements, baro_measurements, gps_measurements);
        baro_ready = 1;
      }
    }
    if(gps_ready==0){
      if(!NOT_ENOUGH_GPS_MEASUREMENTS(gps_measurements)){
        printf("GPS READY %3i %3i %3i %3i\n", imu_measurements, magnetometer_measurements, baro_measurements, gps_measurements);
        gps_ready = 1;
      }
    }
    #endif /* EKNAV_FROM_LOG_DEBUG */
  }
  // setting the covariance
  gravity.weight_of_the_measurement *= imu_measurements;
  VECTOR_AS_VECT3(gravity.measured_direction, accelerometer_noise);
  magneto.weight_of_the_measurement *= magnetometer_measurements;
  VECTOR_AS_VECT3(magneto.measured_direction, magnetometer_noise);
  add_set_of_three_measurements(&sigmaB, gravity, magneto);
  
  #ifdef EKNAV_FROM_LOG_DEBUG
  DISPLAY_FLOAT_RMAT("     B", attitude_profile_matrix);
  DISPLAY_FLOAT_RMAT("sigmaB", sigmaB);
  #endif /* EKNAV_FROM_LOG_DEBUG */
  
  //  setting the initial orientation
  q_ned2body = estimated_attitude(attitude_profile_matrix, 1000, 1e-6, sigmaB, &sigma_q);
	orientation_0 = ecef2body_from_pprz_ned2body(pos_0_ecef,q_ned2body);
  
  baro_0_height += pos_0_ecef.norm();
  
  struct DoubleEulers sigma_eu = sigma_euler_from_sigma_q(q_ned2body, sigma_q);
  orientation_cov_0 = EULER_AS_VECTOR3D(sigma_eu);
  #if WITH_GPS
  pos_cov_0 = 10*gps_pos_noise / gps_measurements;
  speed_cov_0 = 10*gps_speed_noise / gps_measurements;
  #endif  // WITH_GPS
  
  return e;
}


static void main_run_from_file(int file_descriptor, struct raw_log_entry first_entry){
  struct raw_log_entry e = first_entry;
  uint8_t read_ok = 1;
  int t = 10*(int)(1+e.time*0.1);
  while (read_ok) {
    if(e.time>t){
      printf("%6.2fs %6i\n", e.time, entry_counter);
      t += 10;
    }
    if ((e.time<68.48)||(e.time>68.51)){
      print_estimator_state(e.time);
      main_run_ins(e.message.valid_sensors);
    } 
    e = read_raw_log_entry(file_descriptor, &read_ok);
  }
}

static void main_run_ins(uint8_t data_valid) {
  
  double dt_imu_freq = 0.001953125; //  1/512; // doesn't work?
  ins.predict(RATES_AS_VECTOR3D(imu_float.gyro), VECT3_AS_VECTOR3D(imu_float.accel), dt_imu_freq);
  
  if(MAG_READY(data_valid)){
		ins.obs_vector(reference_direction, VECT3_AS_VECTOR3D(imu_float.mag), magnetometer_noise.norm());
	}
  
  #if UPDATE_WITH_GRAVITY
  if(CLOSE_TO_GRAVITY(imu_float.accel)){
		// use the gravity as reference
		ins.obs_vector(ins.avg_state.position.normalized(), VECT3_AS_VECTOR3D(imu_float.accel), accelerometer_noise.norm());
	}
  #endif /* UPDATE_WITH_GRAVITY */
  
   if(BARO_READY(data_valid)){
    ins.obs_baro_report(baro_0_height+imu_baro_height, baro_noise);
  }   // comment out multiple lines */
  
  if(GPS_READY(data_valid)){
		ins.obs_gps_pv_report(VECT3_AS_VECTOR3D(imu_ecef_pos)/100, VECT3_AS_VECTOR3D(imu_ecef_vel)/100, 10*gps_pos_noise, 10*gps_speed_noise);
	}   // comment out multiple lines */
}




static void init_ins_state(void){
	
	ins_logfile = fopen(INS_LOG_FILE, "w");
	
	ins.avg_state.gyro_bias   = bias_0;
	ins.avg_state.orientation = orientation_0;
	ins.avg_state.position    = pos_0_ecef;
	ins.avg_state.velocity    = speed_0_ecef;
  
  struct DoubleQuat ecef2body;
  struct DoubleEulers eu_ecef2body;
  QUATERNIOND_AS_DOUBLEQUAT(ecef2body, orientation_0);
  DOUBLE_EULERS_OF_QUAT(eu_ecef2body, ecef2body);

  
	printf("Initial state\n\n");
  printf("Bias        % 6.1f°/s       +-%7.2f°/s\n", bias_0(0)*180*M_1_PI, bias_cov_0(0)*180*M_1_PI);
  printf("Bias        % 6.1f°/s       +-%7.2f°/s\n", bias_0(1)*180*M_1_PI, bias_cov_0(1)*180*M_1_PI);
  printf("Bias        % 6.1f°/s       +-%7.2f°/s\n", bias_0(2)*180*M_1_PI, bias_cov_0(2)*180*M_1_PI);
  printf("\n");
  printf("Orientation % 7.2f\n", orientation_0.w());
  printf("Orientation % 7.2f %6.1f° +-%6.1f°\n", orientation_0.x(),   eu_ecef2body.phi*180*M_1_PI, orientation_cov_0(0)*180*M_1_PI);
  printf("Orientation % 7.2f %6.1f° +-%6.1f°\n", orientation_0.y(), eu_ecef2body.theta*180*M_1_PI, orientation_cov_0(1)*180*M_1_PI);
  printf("Orientation % 7.2f %6.1f° +-%6.1f°\n", orientation_0.z(),   eu_ecef2body.psi*180*M_1_PI, orientation_cov_0(2)*180*M_1_PI); 
  printf("\n");
  printf("Position    % 9.0f m     +-%7.2f\n", pos_0_ecef(0), pos_cov_0(0));
  printf("Position    % 9.0f m     +-%7.2f\n", pos_0_ecef(1), pos_cov_0(1));
  printf("Position    % 9.0f m     +-%7.2f\n", pos_0_ecef(2), pos_cov_0(2));
  printf("\n");
  printf("Velocity    % 7.2f m/s     +-%7.2f\n", speed_0_ecef(0), speed_cov_0(0));
  printf("Velocity    % 7.2f m/s     +-%7.2f\n", speed_0_ecef(1), speed_cov_0(1));
  printf("Velocity    % 7.2f m/s     +-%7.2f\n", speed_0_ecef(2), speed_cov_0(2));
  printf("\n");
	
	Matrix<double, 12, 1> diag_cov;
  
	/*diag_cov << Vector3d::Ones() * bias_cov_0  * bias_cov_0 ,
            //Vector3d::Ones() *  M_PI*0.5   *  M_PI*0.5  ,
              Vector3d::Ones() *  angle_cov  *  angle_cov ,
							Vector3d::Ones() *  pos_cov_0  *  pos_cov_0 ,
							Vector3d::Ones() * speed_cov_0 * speed_cov_0;*/
  diag_cov << gyro_stability_noise,
              orientation_cov_0,
							pos_cov_0,
							speed_cov_0;
	ins.cov = (diag_cov.cwise()*diag_cov).asDiagonal();
	
}

static void set_reference_direction(void){
	struct NedCoor_d	ref_dir_ned;
	struct EcefCoor_d pos_0_ecef_pprz,
										ref_dir_ecef;
	EARTHS_GEOMAGNETIC_FIELD_NORMED(ref_dir_ned);
	
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
//FLOAT_QUAT_COMP_NORM_SHORTEST(_a2c,          _a2b,        _b2c)
// a = ecef b = ned c = body
  FLOAT_QUAT_COMP_INV_NORM_SHORTEST(q_ecef2body, q_ecef2ned, q_ned2body);
  
  #ifdef EKNAV_FROM_LOG_DEBUG
  printf("Right after initialization:\n");
  DISPLAY_DOUBLE_QUAT("\t ned2body quaternion:", q_ned2body);
  DISPLAY_DOUBLE_QUAT_AS_EULERS_DEG("\t\t\t", q_ned2body);
  DISPLAY_DOUBLE_QUAT("\tecef2enu  quaternion:", q_ecef2enu);
  DISPLAY_DOUBLE_QUAT_AS_EULERS_DEG("\t\t\t", q_ecef2enu);
  DISPLAY_DOUBLE_QUAT("\tecef2ned  quaternion:", q_ecef2ned);
  DISPLAY_DOUBLE_QUAT_AS_EULERS_DEG("\t\t\t", q_ecef2ned);
  DISPLAY_DOUBLE_QUAT("\tecef2body quaternion:", q_ecef2body);
  DISPLAY_DOUBLE_QUAT_AS_EULERS_DEG("\t\t\t", q_ecef2body);
  #endif /* EKNAV_FROM_LOG_DEBUG */
  
  return DOUBLEQUAT_AS_QUATERNIOND(q_ecef2body);
}

struct DoubleEulers sigma_euler_from_sigma_q(struct DoubleQuat q, struct DoubleQuat sigma_q){
  struct DoubleVect3 v_q, v_sigma, temporary_result;
  struct DoubleEulers sigma_eu;
  
  QUAT_IMAGINARY_PART(q, v_q);
  QUAT_IMAGINARY_PART(sigma_q, v_sigma);
  if(DOUBLE_VECT3_NORM(v_sigma)>0.5){
    EULERS_ASSIGN(sigma_eu, M_PI_2, M_PI_2, M_PI_2);
    return sigma_eu;
  }
  
  VECT3_CROSS_PRODUCT(temporary_result, v_q, v_sigma);
  
  VECT3_SMUL(v_q, v_q, sigma_q.qi);
  VECT3_SMUL(v_sigma, v_sigma, q.qi);
  
  VECT3_ADD(temporary_result, v_sigma);
  VECT3_SUB(temporary_result, v_q);
  
  VECT3_TO_EULERS(temporary_result, sigma_eu);
  
  return sigma_eu;
}


/** Logging **/

static struct raw_log_entry read_raw_log_entry(int file_descriptor, uint8_t *read_ok){
  struct raw_log_entry e;
  ssize_t nb_read = read(file_descriptor, &e, sizeof(e));
  *read_ok =  (nb_read == sizeof(e));
  entry_counter ++;
  
  COPY_BARO_TO_IMU(e.message);
  COPY_RATES_ACCEL_TO_IMU_FLOAT(e.message);
  COPY_MAG_TO_IMU_FLOAT(e.message);
  COPY_GPS_TO_IMU(e.message);
  return e;
}

static struct raw_log_entry next_GPS(int file_descriptor){
  uint8_t read_ok;
  struct raw_log_entry e = read_raw_log_entry(file_descriptor, &read_ok);
  while ((read_ok)&&(!GPS_READY(e.message.valid_sensors))) {
    e = read_raw_log_entry(file_descriptor, &read_ok);
  }
  return e;
}

static void print_estimator_state(double time) {

#if FILTER_OUTPUT_IN_NED
	
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
  #if 0
  QUAT_ASSIGN(q_ecef2body, ins.avg_state.orientation.w(), -ins.avg_state.orientation.x(),
	         -ins.avg_state.orientation.y(), -ins.avg_state.orientation.z());
  QUAT_ASSIGN(q_ned2enu, 0, M_SQRT1_2, M_SQRT1_2, 0);
  
  FLOAT_QUAT_OF_RMAT(q_ecef2enu, current_ltp.ltp_of_ecef);
	FLOAT_QUAT_INV_COMP(q_enu2body, q_ecef2enu, q_ecef2body);		// q_enu2body = q_ecef2body * (q_ecef2enu)^*
  FLOAT_QUAT_COMP(q_ned2body, q_ned2enu, q_enu2body);					// q_ned2body = q_enu2body * q_ned2enu

  #else /* if 0 */
  QUATERNIOND_AS_DOUBLEQUAT(q_ecef2body, ins.avg_state.orientation);
  DOUBLE_QUAT_OF_RMAT(q_ecef2enu, current_ltp.ltp_of_ecef);
  FLOAT_QUAT_INV_COMP(q_enu2body, q_ecef2enu, q_ecef2body);
  QUAT_ENU_FROM_TO_NED(q_enu2body, q_ned2body);
  
  #endif /* if 0 */
  
  struct FloatEulers e;
  FLOAT_EULERS_OF_QUAT(e, q_ned2body);
  
  
	#if PRINT_EULER_NED
		printf("EULER % 6.1f % 6.1f % 6.1f\n", e.phi*180*M_1_PI, e.theta*180*M_1_PI, e.psi*180*M_1_PI);
	#endif /* PRINT_EULER_NED */
  fprintf(ins_logfile, "%f %d AHRS_EULER %f %f %f\n", time, AC_ID, e.phi, e.theta, e.psi);
  fprintf(ins_logfile, "%f %d DEBUG_COVARIANCE %f %f %f %f %f %f %f %f %f %f %f %f\n", time, AC_ID,
				sqrt(ins.cov( 0, 0)),  sqrt(ins.cov( 1, 1)),  sqrt(ins.cov( 2, 2)), 
				sqrt(ins.cov( 3, 3)),  sqrt(ins.cov( 4, 4)),  sqrt(ins.cov( 5, 5)), 
				sqrt(ins.cov( 6, 6)),  sqrt(ins.cov( 7, 7)),  sqrt(ins.cov( 8, 8)), 
				sqrt(ins.cov( 9, 9)),  sqrt(ins.cov(10,10)),  sqrt(ins.cov(11,11)));
  fprintf(ins_logfile, "%f %d BOOZ_SIM_GYRO_BIAS %f %f %f\n", time, AC_ID, ins.avg_state.gyro_bias(0), ins.avg_state.gyro_bias(1), ins.avg_state.gyro_bias(2));

#else /* FILTER_OUTPUT_IN_ECEF */
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
#endif /* FILTER_OUTPUT_IN_NED / ECEF */
}
