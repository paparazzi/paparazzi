#include "modules/meteo/ekf_aw_wrapper.h"
#include "modules/meteo/ekf_aw.h"
#include <stdio.h>

#include "state.h"
#include "filters/low_pass_filter.h"
#include "math/pprz_algebra.h"

#include "modules/core/abi.h"

#include "autopilot.h"
#include "modules/actuators/actuators.h"

#include "mcu_periph/sys_time.h" // FOR DEBUG



#ifndef EKF_AW_WRAPPER_ROT_WING
#define EKF_AW_WRAPPER_ROT_WING true
#endif
#ifndef EKF_AW_WRAPPER_ROT_WING_TYPE_A
#define EKF_AW_WRAPPER_ROT_WING_TYPE_A true
#endif
#ifndef EKF_AW_WRAPPER_RANDOM_INPUTS
#define EKF_AW_WRAPPER_RANDOM_INPUTS false
#endif
#ifndef EKF_AW_QUICK_CONVERGENCE
#define EKF_AW_QUICK_CONVERGENCE false
#endif
#ifndef EKF_AW_QUICK_CONVERGENCE_TIME
#define EKF_AW_QUICK_CONVERGENCE_TIME 10.0f
#endif
#ifndef EKF_AW_DEBUG
#define EKF_AW_DEBUG false
#endif

#if EKF_AW_WRAPPER_ROT_WING_TYPE_A
  #include "modules/rot_wing_drone/wing_rotation_controller_v3a.h"
#else
  #include "modules/rot_wing_drone/wing_rotation_controller_v3b.h"
#endif

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

// TO DO: implement circular wrap filter for psi angle
// TO DO: modify force functions to simpler model (wing)

// Telemetry Message functions
static void send_airspeed_wind_ekf(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t healthy = (uint8_t)ekf_aw.health.healthy;
  
  pprz_msg_send_AIRSPEED_WIND_ESTIMATOR_EKF(trans, dev, AC_ID,
                              &ekf_aw.V_body.x,&ekf_aw.V_body.y,&ekf_aw.V_body.z,
                              &ekf_aw.wind.x,&ekf_aw.wind.y,&ekf_aw.wind.z,
                              &ekf_aw.offset.x,&ekf_aw.offset.y,&ekf_aw.offset.z,
                              &healthy,
                              &ekf_aw.health.crashes_n,
                              &ekf_aw.innov_V_gnd.x,&ekf_aw.innov_V_gnd.y,&ekf_aw.innov_V_gnd.z,
                              &ekf_aw.innov_acc_filt.x,&ekf_aw.innov_acc_filt.y,&ekf_aw.innov_acc_filt.z,
                              &ekf_aw.innov_V_pitot,
                              &ekf_aw.acc.x,
                              &ekf_aw.acc.y,
                              &ekf_aw.acc.z);
}

static void send_airspeed_wind_ekf_cov(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AIRSPEED_WIND_ESTIMATOR_EKF_COV(trans, dev, AC_ID,
                              &ekf_aw.process_cov[0],
                              &ekf_aw.process_cov[3],
                              &ekf_aw.process_cov[6],
                              &ekf_aw.process_cov[7],
                              &ekf_aw.process_cov[8],
                              &ekf_aw.process_cov[9],
                              &ekf_aw.meas_cov[0],
                              &ekf_aw.meas_cov[3],
                              &ekf_aw.meas_cov[4],
                              &ekf_aw.meas_cov[5],
                              &ekf_aw.meas_cov[6],
                              &ekf_aw.state_cov[0],&ekf_aw.state_cov[1],&ekf_aw.state_cov[2],
                              &ekf_aw.state_cov[3],&ekf_aw.state_cov[4],&ekf_aw.state_cov[5],
                              &ekf_aw.state_cov[6],&ekf_aw.state_cov[7],&ekf_aw.state_cov[8]);                      
}

static void send_airspeed_wind_ekf_forces(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AIRSPEED_WIND_ESTIMATOR_EKF_FORCES(trans, dev, AC_ID,
                              &ekf_aw.fuselage_force[0],&ekf_aw.fuselage_force[1],&ekf_aw.fuselage_force[2],
                              &ekf_aw.wing_force[0],&ekf_aw.wing_force[1],&ekf_aw.wing_force[2],
                              &ekf_aw.elevator_force[0],&ekf_aw.elevator_force[1],&ekf_aw.elevator_force[2],
                              &ekf_aw.hover_force[0],&ekf_aw.hover_force[1],&ekf_aw.hover_force[2],
                              &ekf_aw.pusher_force[0],&ekf_aw.pusher_force[1],&ekf_aw.pusher_force[2],
                              &ekf_aw.skew,
                              &ekf_aw.elevator_angle,
                              &ekf_aw.RPM_pusher,
                              &ekf_aw.RPM_hover[0]);
}
#endif

// RPM ABI Event
abi_event RPM_ev;
float time_of_rpm = 0.0f;
static void rpm_cb(uint8_t sender_id __attribute__((unused)), struct rpm_act_t *rpm_message, uint8_t num_act);

// Filter struct
struct ekfAw ekf_aw; // Local wrapper
static struct ekfAwParameters *ekf_params; ///< The EKF parameters

// Define settings to change filter tau value
float tau_filter_high = 25.0f;
float tau_filter_low = 0.2f;

// Bool Reset EKF Filter
bool reset_filter = false;

struct NedCoor_f zero_speed = {
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
  };

// Define filter arrays
Butterworth2LowPass filt_groundspeed[3];
Butterworth2LowPass filt_acc[3];
Butterworth2LowPass filt_acc_low[3];
Butterworth2LowPass filt_rate[3];
Butterworth2LowPass filt_euler[3];
Butterworth2LowPass filt_hover_prop_rpm[4];
Butterworth2LowPass filt_pusher_prop_rpm;
Butterworth2LowPass filt_skew;
Butterworth2LowPass filt_elevator_pprz;
Butterworth2LowPass filt_airspeed_pitot;

#ifndef PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH
#define PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH 100
#endif

#ifndef PERIODIC_FREQUENCY_AIRSPEED_EKF
#define PERIODIC_FREQUENCY_AIRSPEED_EKF 25
#endif

void ekf_aw_wrapper_init(void){
  
  // Define filter frequencies
  float sample_time = 1.0f / PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH;
  float tau_low = 1.0f / (2.0f * M_PI * tau_filter_low);
  float tau_high = 1.0f / (2.0f * M_PI * tau_filter_high);

  // Init filters
  for(int8_t i=0; i<3; i++) {
    init_butterworth_2_low_pass(&filt_groundspeed[i], tau_high, sample_time, 0.0); // Init filters groundspeed
    init_butterworth_2_low_pass(&filt_acc[i], tau_high, sample_time, 0.0); // Init filters Accelerations
    init_butterworth_2_low_pass(&filt_acc_low[i], tau_low, sample_time, 0.0); // Init filters Accelerations Low
    init_butterworth_2_low_pass(&filt_rate[i], tau_high, sample_time, 0.0); // Init filters Rate
    init_butterworth_2_low_pass(&filt_euler[i], tau_high, sample_time, 0.0); // Init filters Euler
  }

  for(int8_t i=0; i<4; i++) {
    init_butterworth_2_low_pass(&filt_hover_prop_rpm[i], tau_low, sample_time, 0.0);
    ekf_aw.last_RPM_hover[i] = 0;
  }

  init_butterworth_2_low_pass(&filt_pusher_prop_rpm, tau_low, sample_time, 0.0); // Init filters Pusher Prop
  init_butterworth_2_low_pass(&filt_skew, tau_low, sample_time, 0.0); // Init filters Skew
  init_butterworth_2_low_pass(&filt_elevator_pprz, tau_low, sample_time, 0.0); // Init filters Pusher Prop
  init_butterworth_2_low_pass(&filt_airspeed_pitot, tau_low, sample_time, 0.0); // Init filters Pusher Prop

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_WIND_ESTIMATOR_EKF, send_airspeed_wind_ekf);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_WIND_ESTIMATOR_EKF_COV, send_airspeed_wind_ekf_cov);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_WIND_ESTIMATOR_EKF_FORCES, send_airspeed_wind_ekf_forces);
  #endif
  
  // Init EKF Filter
  ekf_aw_init();
  
  // Register ABI message
  AbiBindMsgRPM(RPM_SENSOR_ID, &RPM_ev, rpm_cb);
  

  //Get EKF param handle
  ekf_params = ekf_aw_get_param_handle();
  
  // Related to in air / on ground logic
  ekf_aw.in_air = false;
  ekf_aw.internal_clock=0;
  ekf_aw.time_last_on_gnd=0;

  // For override of filter using dlsettings
  ekf_aw.override_start = false;
  ekf_aw.override_quick_convergence = false;
  
  // Init of public filter states //TO DO: Is it necessary or just safer?
  ekf_aw.last_RPM_hover[0] = 0;ekf_aw.last_RPM_hover[1] = 0;ekf_aw.last_RPM_hover[2] = 0;ekf_aw.last_RPM_hover[3] = 0;
  ekf_aw.last_RPM_pusher = 0;

  ekf_aw.V_body.x = 0.0f; ekf_aw.V_body.y = 0.0f; ekf_aw.V_body.z = 0.0f;
  ekf_aw.wind.x = 0.0f;   ekf_aw.wind.y = 0.0f;   ekf_aw.wind.z = 0.0f;
  ekf_aw.offset.x = 0.0f; ekf_aw.offset.y = 0.0f; ekf_aw.offset.z = 0.0f;
  ekf_aw.health.healthy = true; ekf_aw.health.crashes_n = 0.0f;
  ekf_aw.innov_V_gnd.x = 0.0f; ekf_aw.innov_V_gnd.y = 0.0f; ekf_aw.innov_V_gnd.z = 0.0f;
  ekf_aw.innov_acc_filt.y = 0.0f;ekf_aw.innov_acc_filt.y = 0.0f;ekf_aw.innov_acc_filt.z = 0.0f;
  ekf_aw.innov_V_pitot = 0.0f;

  ekf_aw.fuselage_force[0] = 0.0f;ekf_aw.fuselage_force[1] = 0.0f;ekf_aw.fuselage_force[2] = 0.0f;
  ekf_aw.wing_force[0] = 0.0f;ekf_aw.wing_force[1] = 0.0f;ekf_aw.wing_force[2] = 0.0f;
  ekf_aw.elevator_force[0] = 0.0f;ekf_aw.elevator_force[1] = 0.0f;ekf_aw.elevator_force[2] = 0.0f;
  ekf_aw.pusher_force[0] = 0.0f;ekf_aw.pusher_force[1] = 0.0f;ekf_aw.pusher_force[2] = 0.0f;
  ekf_aw.hover_force[0] = 0.0f;ekf_aw.hover_force[1] = 0.0f;ekf_aw.hover_force[2] = 0.0f;

  ekf_aw.skew = 0.0f;
  ekf_aw.elevator_angle = 0.0f; 
  ekf_aw.RPM_pusher = 0.0f;
  ekf_aw.RPM_hover[0] = 0.0f; ekf_aw.RPM_hover[1] = 0.0f; ekf_aw.RPM_hover[2] = 0.0f; ekf_aw.RPM_hover[3] = 0.0f;
};

void ekf_aw_wrapper_periodic(void){

  // FOR DEBUG
  // uint32_t tic = get_sys_time_usec();

  //printf("Running periodic Airspeed EKF Module\n");
  //printf("Airspeed is: %2.2f\n",filt_groundspeed[0].o[0]);
  
  // FOR DEBUG Random acc inputs to filter
  if(EKF_AW_WRAPPER_RANDOM_INPUTS){
    
    ekf_aw.acc.x = 1.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.acc.y = 1.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.acc.z = 1.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.gyro.p = 0.1f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.gyro.q = 0.1f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.gyro.r = 0.1f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.euler.phi = 1.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.euler.theta = 1.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.euler.psi = 1.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.RPM_hover[0] = 1000.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.RPM_hover[1] = 1000.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.RPM_hover[2] = 1000.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.RPM_hover[3] = 1000.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.RPM_pusher = 1000.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.skew = 10.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.elevator_angle = 10.0f*rand()/RAND_MAX; // TO BE REMOVED

    ekf_aw.Vg_NED.x = 10.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.Vg_NED.y = 10.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.Vg_NED.z = 10.0f*rand()/RAND_MAX; // TO BE REMOVED

    ekf_aw.acc_filt.x = 1.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.acc_filt.y = 1.0f*rand()/RAND_MAX; // TO BE REMOVED
    ekf_aw.acc_filt.z = 1.0f*rand()/RAND_MAX; // TO BE REMOVED

    ekf_aw.V_pitot = 10.0f*rand()/RAND_MAX; // TO BE REMOVED
  }
  else{
    // Get latest filtered values to ekf struct
    ekf_aw.acc.x = filt_acc[0].o[0];
    ekf_aw.acc.y = filt_acc[1].o[0];
    ekf_aw.acc.z = filt_acc[2].o[0];
    
    ekf_aw.gyro.p = filt_rate[0].o[0];
    ekf_aw.gyro.q = filt_rate[1].o[0];
    ekf_aw.gyro.r = filt_rate[2].o[0];

    ekf_aw.euler.phi = filt_euler[0].o[0];
    ekf_aw.euler.theta = filt_euler[1].o[0];
    //ekf_aw.euler.psi = filt_euler[2].o[0];
    ekf_aw.euler.psi = stateGetNedToBodyEulers_f()->psi; // TO DO: implement circular wrap filter for psi angle

    for(int8_t i=0; i<4; i++) {
    ekf_aw.RPM_hover[i] = filt_hover_prop_rpm[i].o[0];
    }

    ekf_aw.RPM_pusher = filt_pusher_prop_rpm.o[0];
    ekf_aw.skew = filt_skew.o[0];
    ekf_aw.elevator_angle = filt_elevator_pprz.o[0];

    ekf_aw.Vg_NED.x = filt_groundspeed[0].o[0];
    ekf_aw.Vg_NED.y = filt_groundspeed[1].o[0];
    ekf_aw.Vg_NED.z = filt_groundspeed[2].o[0];

    ekf_aw.acc_filt.x = filt_acc_low[0].o[0];
    ekf_aw.acc_filt.y = filt_acc_low[1].o[0];
    ekf_aw.acc_filt.z = filt_acc_low[2].o[0];

    ekf_aw.V_pitot = filt_airspeed_pitot.o[0];
    
  }

  // Sample time of EKF filter
  float sample_time = 1.0f / PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH;
  
  // Check if in flight and altitude higher than 1m
  set_in_air_status(autopilot_in_flight() & (-stateGetPositionNed_f()->z>1.0f));

  // Propagate
  if (ekf_aw.in_air | ekf_aw.override_start){
    // Quick convergence for the first 10 s
    if ( ((ekf_aw.internal_clock-ekf_aw.time_last_on_gnd<PERIODIC_FREQUENCY_AIRSPEED_EKF*EKF_AW_QUICK_CONVERGENCE_TIME) & EKF_AW_QUICK_CONVERGENCE) | ekf_aw.override_quick_convergence){
      ekf_params->quick_convergence = true;
    }
    else{
      ekf_params->quick_convergence = false;
    }
    
    ekf_aw_propagate(&ekf_aw.acc,&ekf_aw.gyro, &ekf_aw.euler, &ekf_aw.RPM_pusher,ekf_aw.RPM_hover, &ekf_aw.skew, &ekf_aw.elevator_angle, &ekf_aw.Vg_NED, &ekf_aw.acc_filt, &ekf_aw.V_pitot,sample_time);
  }
  else{
    // Set body velocity to 0 when landed
    ekf_aw_set_speed_body(&zero_speed);
  };

  // Get states, health and innovation from EKF filter
  ekf_aw.V_body = ekf_aw_get_speed_body();
  ekf_aw.wind = ekf_aw_get_wind_ned();
  ekf_aw.offset = ekf_aw_get_offset();
  ekf_aw.health = ekf_aw_get_health();
  ekf_aw.innov_V_gnd = ekf_aw_get_innov_V_gnd();
  ekf_aw.innov_acc_filt = ekf_aw_get_innov_accel_filt();
  ekf_aw.innov_V_pitot = ekf_aw_get_innov_V_pitot();

  // Get covariance
  ekf_aw_get_meas_cov(ekf_aw.meas_cov);
  ekf_aw_get_state_cov(ekf_aw.state_cov);
  ekf_aw_get_process_cov(ekf_aw.process_cov);

  // Get forces
  ekf_aw_get_fuselage_force(ekf_aw.fuselage_force);
  ekf_aw_get_wing_force(ekf_aw.wing_force);
  ekf_aw_get_elevator_force(ekf_aw.elevator_force);
  ekf_aw_get_hover_force(ekf_aw.hover_force);
  ekf_aw_get_pusher_force(ekf_aw.pusher_force);

  ekf_aw.internal_clock++;

  // FOR DEBUG
  //ekf_aw.offset.x = get_sys_time_usec()-tic;
};

// Function to get information from different modules and set it in the different filters
void ekf_aw_wrapper_fetch(void){

  // For debug
  // uint32_t tic_2 = get_sys_time_usec();

  // NED Speed
  update_butterworth_2_low_pass(&filt_groundspeed[0], stateGetSpeedNed_f()->x);
  update_butterworth_2_low_pass(&filt_groundspeed[1], stateGetSpeedNed_f()->y);
  update_butterworth_2_low_pass(&filt_groundspeed[2], stateGetSpeedNed_f()->z);

  // Getting body accel
  struct FloatVect3 body_accel_f = {0.0f,0.0f,0.0f};
  if (EKF_AW_WRAPPER_ROT_WING){
    // If body accel available, can use this
    struct Int32Vect3 *body_accel_i;
    body_accel_i = stateGetAccelBody_i();
    ACCELS_FLOAT_OF_BFP(body_accel_f, *body_accel_i);
  }
  else{
    // Transferring from NED to Body as body is not available right now
    struct NedCoor_i *accel_tmp = stateGetAccelNed_i();
    struct Int32Vect3 ned_accel_i,body_accel_i;
    struct Int32RMat *ned_to_body_rmat = stateGetNedToBodyRMat_i();
    VECT3_COPY(ned_accel_i, (*accel_tmp));
    ned_accel_i.z += ACCEL_BFP_OF_REAL(-9.81); // Add gravity
    int32_rmat_vmult(&body_accel_i, ned_to_body_rmat, &ned_accel_i);
    ACCELS_FLOAT_OF_BFP(body_accel_f, body_accel_i);
  }

  // Body accel
  update_butterworth_2_low_pass(&filt_acc[0], body_accel_f.x);
  update_butterworth_2_low_pass(&filt_acc[1], body_accel_f.y);
  update_butterworth_2_low_pass(&filt_acc[2], body_accel_f.z);
  update_butterworth_2_low_pass(&filt_acc_low[0], body_accel_f.x);
  update_butterworth_2_low_pass(&filt_acc_low[1], body_accel_f.y);
  update_butterworth_2_low_pass(&filt_acc_low[2], body_accel_f.z);

  // Body rates
  update_butterworth_2_low_pass(&filt_rate[0], stateGetBodyRates_f()->p);
  update_butterworth_2_low_pass(&filt_rate[1], stateGetBodyRates_f()->q);
  update_butterworth_2_low_pass(&filt_rate[2], stateGetBodyRates_f()->r);

  // Euler angles
  update_butterworth_2_low_pass(&filt_euler[0], stateGetNedToBodyEulers_f()->phi);
  update_butterworth_2_low_pass(&filt_euler[1], stateGetNedToBodyEulers_f()->theta);
  //update_butterworth_2_low_pass(&filt_euler[2], stateGetNedToBodyEulers_f()->psi); // TO DO: implement circular wrap filter for psi angle

  for(int8_t i=0; i<4; i++) {
    update_butterworth_2_low_pass(&filt_hover_prop_rpm[i], ekf_aw.last_RPM_hover[i]*1.0f);
  }
  update_butterworth_2_low_pass(&filt_pusher_prop_rpm, ekf_aw.last_RPM_pusher*1.0f);

  if (EKF_AW_WRAPPER_ROT_WING){
    update_butterworth_2_low_pass(&filt_skew, wing_rotation.wing_angle_rad);

    // Get elevator pprz signal
    int16_t *elev_pprz = &actuators_pprz[5];
    float de = 0.0f;
    if (EKF_AW_WRAPPER_ROT_WING_TYPE_A){
      // Calculate deflection angle in [deg]
      de = (-0.004885417f * *elev_pprz + 36.6f)*3.14f/180.0f;
    }
    else{
      // Calculate deflection angle in [deg]
      de = (-0.004885417f * *elev_pprz + 36.6f)*3.14f/180.0f;
    }
    
    update_butterworth_2_low_pass(&filt_elevator_pprz, de);
  }
  else{
    update_butterworth_2_low_pass(&filt_skew, 0.0f);
    update_butterworth_2_low_pass(&filt_elevator_pprz, 0.0f);
  }
  
  update_butterworth_2_low_pass(&filt_airspeed_pitot, stateGetAirspeed_f());

  // FOR DEBUG
  //ekf_aw.offset.y = get_sys_time_usec()-tic_2;
  
};

// ABI callback that obtains the RPM from a module
static void rpm_cb(uint8_t sender_id __attribute__((unused)), struct rpm_act_t * rpm_message, uint8_t num_act)
{
  // Sanity check that index is valid
  if (rpm_message->actuator_idx<num_act){
    // Assign rpm to right actuator
    switch (rpm_message->actuator_idx) {
      case 0:
          ekf_aw.last_RPM_hover[0] = rpm_message->rpm;
          break;
      case 1:
          ekf_aw.last_RPM_hover[1] = rpm_message->rpm;
          break;
      case 2:
          ekf_aw.last_RPM_hover[2] = rpm_message->rpm;
          break;
      case 3:
          ekf_aw.last_RPM_hover[3] = rpm_message->rpm;
          break;
      case 4:
          ekf_aw.last_RPM_pusher = rpm_message->rpm;
          break;
      default:
          break;
    }  
  time_of_rpm = get_sys_time_float();
  }
};

// Set vehicle landed status data
	void set_in_air_status(bool in_air)
	{
		if (!in_air) {
			ekf_aw.time_last_on_gnd = ekf_aw.internal_clock;

		} else {
			ekf_aw.time_last_in_air = ekf_aw.internal_clock;
		}
		ekf_aw.in_air = in_air;
	}

  /*
  For this debug config:

  To start the filter manually
  -"Start" dlsetting can be used to put filter on

  To send random values in the filter:
  - Set define EKF_AW_WRAPPER_RANDOM_INPUTS in ekf_aw_wrapper.c to true

  To check filter timing:
  - time required to run different parts of filter sent on telementry  AIRSPEED_WIND_ESTIMATOR_EKF_FORCES, on different fields, if EKF_AW_DEBUG set to TRUE
  
  */