/*
 * Copyright (C) GJ van Dam
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/rl_obstacle_avoidance/rl_obstacle_avoidance.c"
 * @author GJ van Dam
 * Obstacle avoidance method using RL and the aerodynamic interaction between obstacle and quadrotor
 */
// Include header file
#ifndef RL_OBSTACLE_AVOIDANCE_H
#include "modules/rl_obstacle_avoidance/rl_obstacle_avoidance.h"
#endif

// Include standard libraries
#include <stdio.h>
#include "std.h"
#include <string.h>
#include <errno.h>

// Include telemetry headers
#include "subsystems/datalink/telemetry.h"

// Include modules which will be used to retrieve measurements
#include "state.h"
#include "subsystems/imu.h"
#include "subsystems/actuators.h"
#include "boards/bebop/actuators.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "filters/low_pass_filter.h"
#include "subsystems/gps/gps_datalink.h"

// Include other paparazzi modules
#include "generated/modules.h"
#include "subsystems/datalink/datalink.h" // dl_buffer

// Include module specific files
#include "log_to_file.h"

// Set pre-processor constants
#define RL_OBSTACLE_AVOIDANCE_LOG TRUE
#define RL_OBSTACLE_AVOIDANCE_TELEMETRY TRUE

// Declaration of global variables
float rl_obstacle_avoidance_filter_cutoff = 3.0;
float rl_obstacle_avoidance_termination_dist = 0.05;
float rl_obstacle_avoidance_descend_speed = -0.3;
float rl_wall_heading = 237.0;
int rl_obstacle_avoidance_policy_received = false;
int set_estimated_accel_bias = false;
int set_estimated_k_over_m = false;
float rl_exploration_rate = 0.0;
int rl_autostart = false;
int rl_exploring_starts = true;
int rl_exploring_starts_frozen = false;

// Declaration of local variables
static int32_t number_of_variables = 0;
struct timeval currentTime;
static int32_t start_time_seconds = 0;
static int32_t start_time_milliseconds = 0;
static int32_t episode_start_time_seconds = 0;
static int32_t episode_start_time_milliseconds = 0;

static int32_t episode = 0;
static int32_t timestep = 0;
static int32_t time_rl = 0;
static int32_t episode_time_rl = 0;
static float body_acceleration_u = 0.0;
static float body_acceleration_v = 0.0;
static float body_acceleration_w = 0.0;
static float body_speed_u = 0.0;
static float body_speed_v = 0.0;
static float body_speed_w = 0.0;
static float gps_vertical_speed = 0.0;
static float enu_position_x = 0.0;
static float enu_position_y = 0.0;
static float enu_position_z = 0.0;
static float body_rate_p = 0.0;
static float body_rate_q = 0.0;
static float body_rate_r = 0.0;
static float body_attitude_phi = 0.0;
static float body_attitude_theta = 0.0;
static float body_attitude_psi = 0.0;
static uint16_t motor_speed_nw = 0;
static uint16_t motor_speed_ne = 0;
static uint16_t motor_speed_se = 0;
static uint16_t motor_speed_sw = 0;
static uint16_t motor_speed_nw_cmd = 0;
static uint16_t motor_speed_ne_cmd = 0;
static uint16_t motor_speed_se_cmd = 0;
static uint16_t motor_speed_sw_cmd = 0;
static int32_t flight_status = 0;
static float F_ext_onboard_est = 0.0;
static float body_acceleration_w_filt = 0.0;
static float F_thrust_est = 0.0;

//static float filt_tau;
//static float filt_sample_time;
static Butterworth4LowPass filt_accel_body[3];
static Butterworth4LowPass filt_motor_speed[4];
static Butterworth4LowPass filt_body_rate[4];

// Quadrotor specific variables
//static float k_D = 0.27106224; // Bebop 1
//static float k_D = 0.37283936; // Bebop 2

#ifndef RL_OBSTACLE_AVOID_k_D_z
#define RL_OBSTACLE_AVOID_k_D_z 0.0
#endif

static int estimate_k_over_m_counter = 0;
static double estimated_k_over_m[4] = {0.0, 0.0, 0.0, 0.0};
//static float estimated_k_over_m[4] = {3.5851632392191094E-06, 3.5851632392191094E-06, 3.5851632392191094E-06, 3.5851632392191094E-06};
static float estimated_accel_bias[3] = {0.0, 0.0, 0.0};
static int estimate_accel_bias_counter = 0;

// Episode variables
static int32_t max_episode_length = 10000;

// RL variables
#define RL_DISCR_BINS_F_EXT 9
#define RL_DISCR_BINS_PREV_ACTION 3
#define POLICY_SIZE_1 9
#define POLICY_SIZE_2 3

static uint16_t prev_action;
static rl_state current_state;
static uint16_t action_policy;
static uint16_t action_chosen;
static uint16_t action_performed;
static float discr_state_F_ext_bounds[POLICY_SIZE_1] = {-1.05, -0.95, -0.85, -0.75, -0.65, -0.55, -0.45, -0.35, -0.25}; // Bebop 1
//static float discr_state_F_ext_bounds[POLICY_SIZE_1] = {-1.35, -1.25, -1.15, -1.05, -0.95, -0.85, -0.75, -0.65, -0.55}; // Bebop 2
//static uint16_t discr_state_prev_action_bounds[POLICY_SIZE_2] = {0.5, 1.5, 2.5};
static uint16_t action_space[3] = {1, 2, 3};
static uint16_t current_policy[POLICY_SIZE_1][POLICY_SIZE_2] = {};
//static int current_policy[POLICY_SIZE_1][POLICY_SIZE_2] = {{1}, {1}, {1}, {1}, {1}, {1}, {1}, {1}, {1}, {1}};

//static int current_policy[5][5] = {
//        {1, 1, 1, 1, 1},
//        {1, 1, 1, 1, 1},
//        {1, 1, 1, 1, 1},
//        {1, 1, 1, 1, 1},
//        {1, 1, 1, 1, 1}
//};

static int rl_intervention_hover = false;
static int rl_intervention_save = false;
static int rl_episode_fail = false;
static int rl_episode_timeout = false;
static float F_ext_rolling_mean;
static float rl_starting_height = 1.0;

static rl_variable list_of_variables_to_log[40] = {
        // Name, Type, Format, *pointer
        {"Timestep","int32_t","%d",&timestep},
        {"Time (milliseconds)","int32_t","%li",&time_rl},
        {"Flight status","int32_t","%d",&flight_status},
        {"Episode","int32_t","%d", &episode},
        {"Episode Time (milliseconds)","int32_t","%li",&episode_time_rl},
        {"Body Acceleration u","float","%f",&body_acceleration_u},
        {"Body Acceleration v","float","%f",&body_acceleration_v},
        {"Body Acceleration w","float","%f",&body_acceleration_w},
        {"Body speed u","float","%f",&body_speed_u},
        {"Body speed v","float","%f",&body_speed_v},
        {"Body speed w","float","%f",&body_speed_w},
        {"GPS vertical speed", "float", "%f", &gps_vertical_speed},
        {"ENU position x","float","%f",&enu_position_x},
        {"ENU position y","float","%f",&enu_position_y},
        {"ENU position z","float","%f",&enu_position_z},
        {"Body rate p","float","%f",&body_rate_p},
        {"Body rate q","float","%f",&body_rate_q},
        {"Body rate r","float","%f",&body_rate_r},
        {"Body attitude roll (phi)","float","%f",&body_attitude_phi},
        {"Body attitude pitch (theta)","float","%f",&body_attitude_theta},
        {"Body attitude yaw (psi)","float","%f",&body_attitude_psi},
        {"Motorspeed 1 (NW)","uint16_t","%d",&motor_speed_nw},
        {"Motorspeed 2 (NE)","uint16_t","%d",&motor_speed_ne},
        {"Motorspeed 3 (SE)","uint16_t","%d",&motor_speed_se},
        {"Motorspeed 4 (SW)","uint16_t","%d",&motor_speed_sw},
        {"Motorspeed 1 (NW)_cmd","uint16_t","%d",&motor_speed_nw_cmd},
        {"Motorspeed 2 (NE)_cmd","uint16_t","%d",&motor_speed_ne_cmd},
        {"Motorspeed 3 (SE)_cmd","uint16_t","%d",&motor_speed_se_cmd},
        {"Motorspeed 4 (SW)_cmd","uint16_t","%d",&motor_speed_sw_cmd},
        {"F_ext_onboard_est","float","%f",&F_ext_onboard_est},
        {"F_ext_rolling_mean", "float", "%f",&F_ext_rolling_mean},
        {"body_acceleration_w_filt","float","%f",&body_acceleration_w_filt},
        {"F_thrust_est","float","%f",&F_thrust_est},
        {"Discr_state_F_ext","uint16_t","%u",&current_state.discr_F_ext},
        {"Discr_state_prev_action","uint16_t","%u",&current_state.discr_prev_action},
        {"Policy action","uint16_t","%u",&action_policy},
        {"Action chosen","uint16_t","%u",&action_chosen},
        {"Action performed","uint16_t","%u",&action_performed},
        {"accel_bias_w","float","%f",&estimated_accel_bias[2]},
        {"k_over_m","double","%.10f",&estimated_k_over_m[0]},
};

// Define static functions
#ifndef USE_NPS
    /*
     * Specific in-flight variables and functions
     * */

    // Functions
    static float estimate_F_ext_onboard(void);

#else
    /*
     * Specific simulator variables and functions
     */
    // Variables
//    #define RL_NUMBER_OF_MEAUSUREMENTS 150432
//    static float F_ext_measurements_z[RL_NUMBER_OF_MEAUSUREMENTS];
//    static float F_ext_measurements_F_ext[RL_NUMBER_OF_MEAUSUREMENTS];
//    static FILE *csv_measurements = NULL;
    static Butterworth2LowPass filt_noise_F_ext_simulated;
    static float noise_F_ext_cutoff = 1.8;

    // Functions
    static float estimate_F_ext_from_measurements(float height);
//    static void parseCSVmeasurements(void);
//    static float binary_search(float sorted_list[], int low, int high, float element);
//    static int random_in_range(int min, int max);
#endif

static void rl_obstacle_avoidance_send_message_down(char *_request, char *_parameters);
static uint16_t rl_obstacle_avoidance_get_action(rl_state state);
static void rl_obstacle_avoidance_state_estimator(void);
static void rl_obstacle_avoidance_perform_action(uint16_t action);
static int rl_obstacle_avoidance_check_crash(void);
static float randn (float mu, float sigma);
static float random_float_in_range(float min, float max);
static void send_rl_variables(struct transport_tx *trans, struct link_device *dev);
static void send_rl_variables(struct transport_tx *trans, struct link_device *dev){
    // When prompted, return all the telemetry variables
    pprz_msg_send_RL_OBSTACLE_AVOIDANCE(trans, dev, AC_ID,
                                        &timestep, &time_rl,
                                        &body_acceleration_u, &body_acceleration_v, &body_acceleration_w,
                                        &body_speed_u, &body_speed_v, &body_speed_w, &gps_vertical_speed,
                                        &enu_position_x, &enu_position_y, &enu_position_z,
                                        &body_rate_p, &body_rate_q, &body_rate_r,
                                        &body_attitude_phi, &body_attitude_theta, &body_attitude_psi,
                                        &motor_speed_nw, &motor_speed_ne, &motor_speed_se, &motor_speed_sw,
                                        &F_ext_onboard_est, &F_ext_rolling_mean);
}

/** Initialization function **/
void rl_obstacle_avoidance_init(void) {
    // Determine number of variables
    number_of_variables = sizeof(list_of_variables_to_log) / sizeof(rl_variable);

    // Register telemetery function
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RL_OBSTACLE_AVOIDANCE, send_rl_variables);

//#ifdef USE_NPS // Simulation
//    // Load CSV measurements
//    parseCSVmeasurements();
//#endif
}

/*
 *  Function to close current log file and start a new one
 */
void rl_obstace_avoidance_new_log_file(void){
    if(RL_OBSTACLE_AVOIDANCE_LOG){
        // Close previous log file
        log_to_file_stop();

        // Create a new one

        // Create csv header line
        char csv_header_line[1000] = "";
        int i;
        for (i=0; i < number_of_variables; i++){
            if(i>0){
                strcat(csv_header_line, ",");
            }
            strcat(csv_header_line, list_of_variables_to_log[i].name);
        }
        strcat(csv_header_line, "\n");

        // Write header line to file

        log_to_file_start(csv_header_line);

    }
}

/** Function called when the module is started, performs the following functions:
 * -> Create log file
 * -> Open log file
 * */
void rl_obstacle_avoidance_start(void){
    // If logging is turned on, create file
    if(RL_OBSTACLE_AVOIDANCE_LOG){
        // Create csv header line
        char csv_header_line[1000] = "";
        int i;
        for (i=0; i < number_of_variables; i++){
            if(i>0){
                strcat(csv_header_line, ",");
            }
            strcat(csv_header_line, list_of_variables_to_log[i].name);
        }
        strcat(csv_header_line, "\n");

        // Write header line to file

        log_to_file_start(csv_header_line);
    }

    // Re-estimate acceleration bias
    set_estimated_accel_bias = false;
    set_estimated_k_over_m = false;
    estimate_accel_bias_counter = 0;
    estimate_k_over_m_counter = 0;
    for (int8_t i = 0; i < 4; i++) {
        estimated_k_over_m[i] = 0.0;
    }
    for (int8_t i = 0; i < 3; i++) {
        estimated_accel_bias[i] = 0.0;
    }

    // Initialize butterworth filters
    float filt_tau = 1.0 / (2.0 * M_PI * rl_obstacle_avoidance_filter_cutoff);
    float filt_sample_time= 1.0 / PERIODIC_FREQUENCY;

    // Filtered body accelerations
    for (int8_t i = 0; i < 3; i++) {
        init_butterworth_4_low_pass(&filt_accel_body[i], filt_tau, filt_sample_time, 0.0);
    }

    // Filtered RPM
    for (int8_t i = 0; i < 4; i++) {
        init_butterworth_4_low_pass(&filt_motor_speed[i], filt_tau, filt_sample_time, 0.0);
    }

    // Filtered body rates
    for (int8_t i = 0; i < 3; i++) {
        init_butterworth_4_low_pass(&filt_body_rate[i], filt_tau, filt_sample_time, 0.0);
    }

#ifdef USE_NPS // Simulation
    float filt_tau_noise = 1.0 / (2.0 * M_PI * noise_F_ext_cutoff);

    // Filter used to generate noise on the simulated F_ext signal
    init_butterworth_2_low_pass(&filt_noise_F_ext_simulated, filt_tau_noise, filt_sample_time, 0.0);
#endif

    // Set start time in seconds
    gettimeofday(&currentTime, NULL);
    start_time_seconds = currentTime.tv_sec;
    start_time_milliseconds = currentTime.tv_usec;
}

void rl_obstacle_avoidance_update_measurements(void){
    // Update timestep
    timestep++;

    // Get time in milliseconds since the measurement has started
    gettimeofday(&currentTime, NULL);
    time_rl = (currentTime.tv_sec - start_time_seconds) * 1000 + (currentTime.tv_usec) / 1000 - start_time_milliseconds / 1000;
    if( episode > 0) {
        episode_time_rl = (currentTime.tv_sec - episode_start_time_seconds) * 1000 + (currentTime.tv_usec) / 1000 -
                          episode_start_time_milliseconds / 1000;
    }
    // Update accelerations in the body frame
    struct Int32Vect3 *body_accelerations = stateGetAccelBody_i();
    body_acceleration_u = ACCEL_FLOAT_OF_BFP(body_accelerations->x);
    body_acceleration_v = ACCEL_FLOAT_OF_BFP(body_accelerations->y);
    body_acceleration_w = ACCEL_FLOAT_OF_BFP(body_accelerations->z);

    // Update filtered accelerations in the body frame
    update_butterworth_4_low_pass(&filt_accel_body[0], body_acceleration_u);
    update_butterworth_4_low_pass(&filt_accel_body[1], body_acceleration_v);
    update_butterworth_4_low_pass(&filt_accel_body[2], body_acceleration_w);
    body_acceleration_w_filt = filt_accel_body[2].lp2.o[0];

    // Update speeds in the body frame
    struct FloatVect3 *speed_ned = (struct FloatVect3 *)stateGetSpeedNed_f();
    struct FloatRMat *ned_to_body = stateGetNedToBodyRMat_f();
    struct FloatVect3 speed_body;
    float_rmat_vmult(&speed_body, ned_to_body, speed_ned);
    body_speed_u = speed_body.x;
    body_speed_v = speed_body.y;
    body_speed_w = speed_body.z;

#ifndef USE_NPS // Simulation
    // Get speed from GPS (more accurate, but with delay)
    if (bit_is_set(gps_datalink.valid_fields, GPS_VALID_VEL_ECEF_BIT)) {
//        Get value and convert from cm/s to m/s and flip axis
        gps_vertical_speed = gps_datalink.ecef_vel.z*-0.01;
    }
#endif
    // Update position in the ENU frame
    struct EnuCoor_f *enu_position_f = stateGetPositionEnu_f();
    enu_position_x = enu_position_f->x;
    enu_position_y = enu_position_f->y;
    enu_position_z = enu_position_f->z;

    // Update angular rates in the body frame
    struct FloatRates *body_rates_f = stateGetBodyRates_f();
    body_rate_p = body_rates_f->p;
    body_rate_q = body_rates_f->q;
    body_rate_r = body_rates_f->r;

    // Update filtered angular rates in the body frame
    update_butterworth_4_low_pass(&filt_body_rate[0], body_rate_p);
    update_butterworth_4_low_pass(&filt_body_rate[1], body_rate_q);
    update_butterworth_4_low_pass(&filt_body_rate[2], body_rate_r);

    // Update euler angels in the body frame
    struct FloatEulers *ned_attitude_f = stateGetNedToBodyEulers_f();
    body_attitude_phi = ned_attitude_f->phi;
    body_attitude_theta = ned_attitude_f->theta;
    body_attitude_psi = ned_attitude_f->psi;

#ifndef USE_NPS
    // Update motor speeds
    motor_speed_nw = actuators_bebop.rpm_obs[0];
    motor_speed_ne = actuators_bebop.rpm_obs[1];
    motor_speed_se = actuators_bebop.rpm_obs[2];
    motor_speed_sw = actuators_bebop.rpm_obs[3];

    // Update commanded motor speeds
    motor_speed_nw_cmd = actuators_bebop.rpm_ref[0];
    motor_speed_ne_cmd = actuators_bebop.rpm_ref[1];
    motor_speed_se_cmd = actuators_bebop.rpm_ref[2];
    motor_speed_sw_cmd = actuators_bebop.rpm_ref[3];

    // Update filtered motor speeds
    for (int8_t i = 0; i < 4; i++) {
        update_butterworth_4_low_pass(&filt_motor_speed[i], actuators_bebop.rpm_obs[i]);
    }
#endif

    // Estimate F_ext
#ifdef USE_NPS
    // Simulation, so use
    F_ext_onboard_est = estimate_F_ext_from_measurements(enu_position_z);
#else
    // Onboard estimate
    F_ext_onboard_est = estimate_F_ext_onboard();
#endif
}

/* Function that estimates the accelerometer bias, runs before the motors are started
 * */
int rl_obstacle_avoidance_est_accel_bias(void){
#ifndef USE_NPS
    int number_of_samples = 1000;

    // Counter starts at 0
    estimate_accel_bias_counter++;

    if(estimate_accel_bias_counter <= number_of_samples){
        // Add current acceleration to sample
        estimated_accel_bias[2] = (float) estimated_accel_bias[2] +  (float) body_acceleration_w / (float) number_of_samples;
    }

    if(estimate_accel_bias_counter >= number_of_samples){
        // Finished
        printf("Accelerometer bias estimated to be: %f \n",estimated_accel_bias[2]);
        set_estimated_accel_bias = true;
        return false;
    } else {
        // Continue the estimation
        return true;
    }
#else
    return false;
#endif

}

void rl_obstacle_avoidance_est_k_over_m(void){
#ifndef USE_NPS

    // Counter starts at 0
    estimate_k_over_m_counter++;

    if(estimate_k_over_m_counter <= number_of_samples){
        int number_of_samples = 1000;
        double current_k_over_m;
        float sum_omega_squared;
        float F_drag;

        sum_omega_squared = 0.0;
        // Calculate sum omega squared
        for (int8_t i = 0; i < 4; i++) {
            sum_omega_squared = sum_omega_squared + powf(filt_motor_speed[i].lp2.o[0] * 2 * M_PI / 60, 2);
        }
        // Estimate current k_over_m
        if (body_speed_w < 0)
            F_drag = -RL_OBSTACLE_AVOID_k_D_z*powf(body_speed_w,2);
        else
            F_drag = RL_OBSTACLE_AVOID_k_D_z*powf(body_speed_w,2);

        current_k_over_m = (9.81 + (filt_accel_body[2].lp2.o[0] - estimated_accel_bias[2]) + F_drag)/ sum_omega_squared;
//        printf("Sum_omega_squared: %f -> Current k over m: %f\n", sum_omega_squared, current_k_over_m);

        // Add current estimation to mean
        for (int8_t i = 0; i < 4; i++) {
            estimated_k_over_m[i] = (double )estimated_k_over_m[i] + (double) current_k_over_m / (double) number_of_samples;
        }
    }

    if(estimate_k_over_m_counter >= number_of_samples){
        // Finished
        printf("K_over_m estimated to be: %f \n",estimated_k_over_m[2]);
        set_estimated_k_over_m = true;
    }
#else
    set_estimated_k_over_m = true;
#endif
}

#ifndef USE_NPS
static float estimate_F_ext_onboard(void){
    float F_ext = 0.0;
    float F_drag;

    if(set_estimated_k_over_m && set_estimated_accel_bias) {
        // Estimate thrust produced by rotors
        F_thrust_est = 0.0;
        for (int8_t i = 0; i < 4; i++) {
            F_thrust_est =
                    F_thrust_est + estimated_k_over_m[i] * powf((filt_motor_speed[i].lp2.o[0] * 2 * M_PI / 60), 2);
        }
        // Estimate dragF
        if (body_speed_w < 0)
            F_drag = -RL_OBSTACLE_AVOID_k_D_z*powf(body_speed_w,2);
        else
            F_drag = RL_OBSTACLE_AVOID_k_D_z*powf(body_speed_w,2);

        // Sum to get F_ext
        F_ext = (filt_accel_body[2].lp2.o[0] - estimated_accel_bias[2]) \
                 + (filt_body_rate[0].lp2.o[0] * body_speed_v) \
                 - (filt_body_rate[1].lp2.o[0] * body_speed_u) \
                 - 9.81 * cos(body_attitude_phi) * cos(body_attitude_theta) \
                 + F_thrust_est + F_drag;
/*        printf("(%f)+(%f*%f)-(%f*%f)-9.81*%f*%f+%f\n", \
                filt_accel_body[2].lp2.o[0], \
                filt_body_rate[0].lp2.o[0], body_speed_v, \
                filt_body_rate[1].lp2.o[0], body_speed_u, \
                cos(body_attitude_phi), cos(body_attitude_theta), \
                F_thrust_est);*/
    }
    return F_ext;
}

#endif

/* Function called at the start of each episode
 */
void rl_obstacle_avoidance_start_episode(){
    // Set intervention & fail states to false
    rl_episode_fail = false;
    rl_intervention_hover = false;
    rl_intervention_save = false;
    rl_episode_timeout = false;

    // Increase episode counter and save start time
    episode = episode+1;
    gettimeofday(&currentTime, NULL);
    episode_start_time_seconds = currentTime.tv_sec;
    episode_start_time_milliseconds = currentTime.tv_usec;

    // Set action to one (no-action) (for the state prev_action)
    action_chosen = 1;
    action_performed = 1;

    // Determine exploring starts height
    if(rl_exploring_starts){
        rl_exploring_starts_frozen = true;
        rl_starting_height = random_float_in_range(0.1, 0.45);
    } else {
        rl_exploring_starts_frozen = false;
        rl_starting_height = 1.0;
    }

    // Print to the console
    if(rl_exploring_starts) {
        printf("New episode started, episode number %d, frozen till %.2fm\n", episode, rl_starting_height);
    } else {
        printf("New episode started, episode number %d\n", episode);
    }
}

/*
 * Function periodic, the heartbeat of the module
 */
void rl_obstacle_avoidance_periodic(void) {
    double random_double;
#ifndef USE_NPS
    // Estimate accelerometer bias
    if(set_estimated_accel_bias == false){
        rl_obstacle_avoidance_est_accel_bias();
    }
#else
    set_estimated_accel_bias = true;
#endif

    // Update measurements
    rl_obstacle_avoidance_update_measurements();

    if((set_estimated_k_over_m == false) && (flight_status == 25)){
        rl_obstacle_avoidance_est_k_over_m();
    }

    // Get state
    rl_obstacle_avoidance_state_estimator();

    if((flight_status >= 50) && (flight_status < 60) && (rl_episode_fail == false)){

        // Check for episode timeout
        if((rl_intervention_hover == false) && (rl_intervention_save == false) && (rl_episode_timeout == false)) {
            if (episode_time_rl > max_episode_length) {
                rl_episode_timeout = true;
                printf("Episode timeout at %d seconds\n", (int) episode_time_rl / 1000);
            }
        }

        // Check for end of exploring starts freeze
        if((rl_exploring_starts == true) && (rl_exploring_starts_frozen == true)){
            // Frozen, check height
//            printf("%.2f\n", enu_position_z);
            if(enu_position_z < rl_starting_height){
//                printf("test");
                rl_exploring_starts_frozen = false;
//                printf("Ended exploring starts at height: %.2f\n", enu_position_z);
            }
        }

        // Only pick and perform an action if we're not yet in another action
        if((rl_intervention_hover == false) && (rl_intervention_save == false) && (rl_episode_timeout == false) && (rl_exploring_starts_frozen == false)){
            // Prepare next timestep, get next action from policy
            action_policy = rl_obstacle_avoidance_get_action(current_state);

            // Exploration
            random_double = (double)rand() / (double)RAND_MAX;
//            printf("Random double: %f \n", random_double);
            if(random_double < rl_exploration_rate){
                // Random action
                action_chosen = action_space[rand() % 3];
                printf("Random action: %d\n",action_chosen);
            } else {
                // Normal action
//                printf("Normal action\n");
                action_chosen = action_policy;
            }

//             printf("[%d][%d] -> %u\n", current_state.discr_F_ext, current_state.discr_prev_action, action_chosen);

            // Check safety of action and need to abort

            if ((rl_obstacle_avoidance_check_crash() == true) || (rl_episode_fail == true)){
//                printf('%.2f', enu_position_z);
                action_performed = 4;
            } else {
                action_performed = action_chosen;
            }

            // Perform action
            rl_obstacle_avoidance_perform_action(action_performed);
        }
    }

    if(RL_OBSTACLE_AVOIDANCE_LOG){
        // Log measurements, states and actions to log file
        log_to_file_log_line(list_of_variables_to_log, number_of_variables);
    }


}

void rl_obstacle_avoidance_end_episode(){
    rl_episode_fail = false;
    rl_intervention_save = false;
    rl_intervention_hover = false;
    rl_episode_timeout = false;
    printf("Episode ended\n");
    rl_obstace_avoidance_new_log_file();
}

void rl_obstacle_avoidance_state_estimator(void){
    float F_ext;
//    float F_ext_rolling_sum = 0.0;

    int F_ext_discr;
    int prev_action_discr;
//    int F_ext_rolling_discr;
    int8_t i;

    // Get F_ext (set by update_measurements)
    F_ext = F_ext_onboard_est;

    // Get prev_action
    prev_action = action_chosen;

    // Update rolling, shift all elements in the array 1 place to the right
//    for (int8_t i=RL_ROLLING_WINDOW_SIZE-1; i > 0; i--){
//        current_state.F_ext_rolling[i] = current_state.F_ext_rolling[i-1];
//        F_ext_rolling_sum = F_ext_rolling_sum + current_state.F_ext_rolling[i];
//
//    }
//
//    // Replace the first value
//    current_state.F_ext_rolling[0]= F_ext;
//    F_ext_rolling_sum = F_ext_rolling_sum + F_ext;
//    F_ext_rolling_mean = F_ext_rolling_sum / RL_ROLLING_WINDOW_SIZE;

//    for (int8_t i=0; i < RL_ROLLING_WINDOW_SIZE; i++){
//        printf("%f +",current_state.F_ext_rolling[i]);
//    }
//    printf("= %f -> %f\n", F_ext_rolling_sum, F_ext_rolling_mean);

    // Discretize F_ext
    F_ext_discr = RL_DISCR_BINS_F_EXT-1;// Maximum value
    for (i=0; i < RL_DISCR_BINS_F_EXT-1; i++){
        if(F_ext < discr_state_F_ext_bounds[i+1]){
            F_ext_discr = i;
            break;
        }
    }

    // Discretize prev_action
    prev_action_discr = RL_DISCR_BINS_PREV_ACTION-1;// Maximum value
    if(prev_action <= 1){
        prev_action_discr = 0;
    } else{
        prev_action_discr = prev_action-1;
    }
//    printf("%u -> %u -> %d\n", action_chosen, prev_action, prev_action_discr);


    // Discretize F_ext_rolling
//    F_ext_rolling_discr = RL_DISCR_BINS_F_EXT_ROLLING-1; // Maximum value
//    for (i=0; i < RL_DISCR_BINS_F_EXT; i++){
//        if(F_ext_rolling_mean < discr_state_F_ext_rolling_bounds[i+1]){
////            printf("if %f < %f -> %d\n", F_ext_rolling_mean, discr_state_F_ext_rolling_bounds[i+1], i);
//            F_ext_rolling_discr = i;
//            break;
//        };
//    }

    // Save to state
    current_state.discr_F_ext = F_ext_discr;
//    current_state.discr_F_ext_rolling = F_ext_rolling_discr;
    current_state.discr_prev_action = prev_action_discr;
}

/*
 *  Function used to check if the quadrotor is 'crashed' and the episode should be aborted
 */
int rl_obstacle_avoidance_check_crash(void){
    if(enu_position_z <= rl_obstacle_avoidance_termination_dist)
        return true;
    else
        return false;
}

uint16_t rl_obstacle_avoidance_get_action(rl_state current_state){
    uint16_t action;

   // Get action from policy
    action = current_policy[current_state.discr_F_ext][current_state.discr_prev_action];
    return action;
}

void rl_obstacle_avoidance_perform_action(uint16_t action){
    if((action ==2) && (rl_intervention_save == false)){
        printf("Save at height %f! F_ext=%f [%d], prev_action=%u [%d]\n",enu_position_z, F_ext_onboard_est, current_state.discr_F_ext, prev_action, current_state.discr_prev_action);
        rl_intervention_save = true;
    } else if ((action == 3) && (rl_intervention_hover == false)){
        printf("Hover at height %f! F_ext=%f [%d], prev_action=%u [%d]\n",enu_position_z, F_ext_onboard_est, current_state.discr_F_ext, prev_action, current_state.discr_prev_action);
        rl_intervention_hover = true;
    } else if ((action == 4) && (rl_episode_fail == false)){
        rl_episode_fail = true;
        printf("Failed at height %f! F_ext=%f [%d], prev_action=%u [%d]\n",enu_position_z, \
            F_ext_onboard_est, current_state.discr_F_ext, prev_action, current_state.discr_prev_action);
    }
}

int rl_obstacle_avoidance_hover(void){
    return rl_intervention_hover;
}

int rl_obstacle_avoidance_save(void){
    return rl_intervention_save;
}

int rl_obstacle_avoidance_fail(void){
    return rl_episode_fail;
}

int rl_obstacle_avoidance_timeout(void){
    return rl_episode_timeout;
}

void rl_obstacle_avoidance_hover_concluded(void){
    rl_intervention_hover = false;
    printf("Hover concluded \n");
}

void rl_obstacle_avoidance_save_concluded(void){
    rl_intervention_save = false;
}

void rl_obstacle_avoidance_turn_on(void){
    rl_obstacle_avoidance_rl_obstacle_avoidance_periodic_status = MODULES_START;
}

void rl_obstacle_avoidance_turn_off(void){
    rl_obstacle_avoidance_rl_obstacle_avoidance_periodic_status = MODULES_STOP;
}

/** Function called when the module is stopped, performs the following functions:
 * -> Close log file
 * */
void rl_obstacle_avoidance_stop(void){

    // If logging is turned on, close file
    if(RL_OBSTACLE_AVOIDANCE_LOG){
        log_to_file_stop();
    }

    // Reset episode counter
    episode = 0;

    // Re-estimate acceleration bias and k_over_m
    set_estimated_accel_bias = false;
    set_estimated_k_over_m = false;
    estimate_accel_bias_counter = 0;
    estimate_k_over_m_counter = 0;
    for (int8_t i = 0; i < 4; i++) {
        estimated_k_over_m[i] = 0.0;
    }
    for (int8_t i = 0; i < 3; i++) {
        estimated_accel_bias[i] = 0.0;
    }

}

/** Functon used to set and store the flight status:
 * */
void rl_obstacle_avoidance_flight_status(int status){
    flight_status = status;
}

#ifdef USE_NPS

/** Function used to simulation F_ext, based on previous measurements
 * */
float estimate_F_ext_from_measurements(float height){
    float F_ext;
    float F_ext_noise_new;
//    float prop_radius = 0.05;
//    float F_ext_theory;
    float F_ext_fit;
    float noise_mu = 0.055;
    float noise_sigma = 0.348*4;

    // Estimate F_ext from theory
//    F_ext_theory = -9.81 * (1 / (1 - powf((prop_radius/ (4 * height)),2)) - 1);
    F_ext_fit = -5.58364265E-12*1/(height+5.03702366E-01)-1.26883748E-02*1/powf(height+7.44916373E-02,2)-1.40903748E-01;

    // Calculate low-pass filtered noise
    F_ext_noise_new = randn(noise_mu, noise_sigma);
    update_butterworth_2_low_pass(&filt_noise_F_ext_simulated, F_ext_noise_new);

    // Sum it to get an estimate for F_ext
    F_ext = F_ext_fit + filt_noise_F_ext_simulated.o[0];
//    printf("%f + %f = %f\n",F_ext_theory, filt_noise_F_ext_simulated.o[0], F_ext);
//    // Search the current height in the sorted list of measurements
//
//    height_id = binary_search(F_ext_measurements_z, 0,RL_NUMBER_OF_MEAUSUREMENTS, height);
//    if(height_id<0){
//        height_id = 0;
//    }
//    if(height_id>(RL_NUMBER_OF_MEAUSUREMENTS-1)){
//        height_id = RL_NUMBER_OF_MEAUSUREMENTS-1;
//    }
//
//    F_ext_estimate = F_ext_measurements_F_ext[height_id];

//    printf("%f -> %d -> %f\n",height, height_id, F_ext_estimate);

    return F_ext;
}

/*
 *  FUnction used to generate a random number from normal distribution
 */
float randn (float mu, float sigma)
{
  float U1, U2, W, mult;
  static float X1, X2;
  static int call = 0;

  if (call == 1)
    {
      call = 0;
      return (mu + sigma * (float) X2);
    }

  do
    {
      U1 = -1 + ((float) rand () / RAND_MAX) * 2;
      U2 = -1 + ((float) rand () / RAND_MAX) * 2;
      W = pow (U1, 2) + pow (U2, 2);
    }
  while (W >= 1 || W == 0);

  mult = sqrt ((-2 * log (W)) / W);
  X1 = U1 * mult;
  X2 = U2 * mult;

  call = !call;

  return (mu + sigma * (float) X1);
}


///** Function used to parse the measurements csv file
// * */
//void parseCSVmeasurements(void){
//    char buf[120];
//    char *item;
//    int32_t reccount = 0;
//
//    csv_measurements = fopen("/home/geart/Measurements/ground_F_ext_c.csv","r");
//
//    if (csv_measurements == NULL) {
//        printf("Error opening measurement file: %s\n", strerror(errno));
//    }
//
//    // Loop through file
//    while (fgets(buf,120,csv_measurements)) {
//
//        // Step
//        item = strtok(buf, ",");
//
//        // F_ext
//        item = strtok(NULL, ",");
//        F_ext_measurements_F_ext[reccount] = atof(item);
//
//        // z
//        item = strtok(NULL, " ");
//        F_ext_measurements_z[reccount] = atof(item);
//
//        reccount++;
//    }
//
//    // Close file
//    fclose(csv_measurements);
//}


/** Binary search algorithm
 * */

//float binary_search(float sorted_list[], int low, int high, float element)
//{
//
//    if (high < low){
//        int random_int = random_in_range(high-10,low+10);
//        return random_int;
//    }
//    int middle = low + (high - low)/2;
//    if (element < sorted_list[middle])
//        return binary_search(sorted_list, low, middle-1, element);
//    else if (element > sorted_list[middle])
//        return binary_search(sorted_list, middle+1, high, element);
//    else
//        return middle;
//}
//
//int random_in_range(int min, int max){
//    return min + rand() / (RAND_MAX / (max - min + 1) + 1);
//}

#endif


/*
 *  Generate random float within a specified range
 */

float random_float_in_range(float min, float max)
{
    float range = (max - min);
    float div = RAND_MAX / range;
    return min + (rand() / div);
}

/*
 *  Request new policy
 */

void rl_obstacle_avoidance_request_new_policy(void){
    char request[100] = "";
    char parameters[100] = "";

    // Start waiting for new policy
    rl_obstacle_avoidance_policy_received = false;

    // Send message
    strcpy(request, "request_new_policy");
    sprintf(parameters, "%s %d", rl_obstacle_avoidance_run_filename, episode);
    rl_obstacle_avoidance_send_message_down(request, parameters);

    printf("Waiting for new policy\n");
}

/*
 *  Function to send a message down
 */

void rl_obstacle_avoidance_send_message_down(char *_request, char *_parameters){
    uint16_t nb_request;
    uint16_t nb_parameters;
    struct pprzlink_msg msg;

    // Calculate length of strings
    nb_request = strlen(_request);
    nb_parameters = strlen(_parameters);

    // Setup message struct
    msg.trans = &(DefaultChannel).trans_tx;
    msg.dev = &(DefaultDevice).device;
    msg.sender_id = AC_ID;
    msg.receiver_id = 0;
    msg.component_id = 0;

    pprzlink_msg_v2_send_RL_TRAINING_DOWN(&msg, nb_request, _request, nb_parameters, _parameters);
}

/*
 * Parse the uplink messages, receive and implement new policy
 */
void rl_obstacle_avoidance_parse_uplink(void){
//    uint16_t ac_id;
    uint16_t index_1;
    uint16_t index_2;
    uint16_t value;
    uint16_t old_policy;

    // Get request and parameters from datalink buffer
//    ac_id = pprzlink_get_DL_RL_TRAINING_UP_ac_id(dl_buffer);
    index_1 = pprzlink_get_DL_RL_TRAINING_UP_index_1(dl_buffer);
    index_2 = pprzlink_get_DL_RL_TRAINING_UP_index_2(dl_buffer);
    value = pprzlink_get_DL_RL_TRAINING_UP_value(dl_buffer);

    // Update policy
    old_policy = current_policy[index_1][index_2];
    current_policy[index_1][index_2] = value;

    printf("Updated policy of state [%u][%u]: %u -> %u\n", index_1, index_2, old_policy, value);

    // Check if this was the last update
    if(index_1 == (POLICY_SIZE_1-1) && index_2 == (POLICY_SIZE_2-1)){
        printf("Policy completely updated\n");
        rl_obstacle_avoidance_policy_received = true;
    }

}


