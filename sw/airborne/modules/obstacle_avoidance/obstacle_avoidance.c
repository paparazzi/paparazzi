/*
 * Copyright (C) 2015 Roland + Clint
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/obstacle_avoidance/obstacle_avoidance.c
 *  @brief Obstacle avoidance methods
 */

#include <stdio.h>
#include <sys/fcntl.h>
#include <math.h>
#include <unistd.h>
#include <inttypes.h>
#include "state.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_int.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/stereocam/stereocam.h"
#include "modules/obstacle_avoidance/guidance_OA.h"
#include "modules/obstacle_avoidance/obstacle_avoidance.h"

#ifndef AVOIDANCES_DISTANCES_HOR_COUNT
#define AVOIDANCES_DISTANCES_HOR_COUNT 36
#endif

#ifndef AVOIDANCE_AMOUNT_OF_BOARDS
#define AVOIDANCE_AMOUNT_OF_BOARDS 6
#endif

#ifndef AVOIDANCE_HEIGHT_IN_MEASUREMENT_VALUES
#define AVOIDANCE_HEIGHT_IN_MEASUREMENT_VALUES 6
#endif

#ifndef AVOIDANCE_WIDTH_IN_MEASUREMENT_VALUES
#define AVOIDANCE_WIDTH_IN_MEASUREMENT_VALUES 6
#endif
//////////////SET BY USER!!!!//////////////////
//sensor info
uint16_t size_matrix[] = {AVOIDANCE_AMOUNT_OF_BOARDS, AVOIDANCE_HEIGHT_IN_MEASUREMENT_VALUES, AVOIDANCE_WIDTH_IN_MEASUREMENT_VALUES};
float stereo_fow[2] = {1.0018, 0.7767};//based on FOW of 57.4, by 44.5
float angle_hor_board[] = {0, 1.0472, 2.0944, 3.1416, -2.0944, -1.0472};
int16_t focal = 118 * 6;
float baseline = 60;
float ref_yaw = 0.0;
//tuning info
float reference_pitch = 0.1;
float reference_roll = 0.1;
float dist_treshold = 1.25;
float distances_hor[AVOIDANCES_DISTANCES_HOR_COUNT];

//////////////Variables CN///////////////////
//for messages
//general
uint8_t *READimageBuffer_old; //USED for butterworth filter
float heading_goal_f = 0;
float heading_goal_ref;
struct FloatVect2 pos_diff;
//target
struct NedCoor_f current_pos;
struct FloatVect2 target;
struct FloatVect2 init_target = {0, 0};
float dx_ref = 0;
float dy_ref = 0;
//Variables Kalman filter
float A_kal = 1;
float B_kal = 0;
float H_kal = 1;
float Q_kal = 0.05;//TODO: clean variables
float R_kal = 2;
float K_gain_send = 0;
float Pest_new[AVOIDANCE_AMOUNT_OF_BOARDS *AVOIDANCE_HEIGHT_IN_MEASUREMENT_VALUES
               *AVOIDANCE_WIDTH_IN_MEASUREMENT_VALUES];
float Xest_new[AVOIDANCE_AMOUNT_OF_BOARDS *AVOIDANCE_HEIGHT_IN_MEASUREMENT_VALUES
               *AVOIDANCE_WIDTH_IN_MEASUREMENT_VALUES];
//variables butterworth filter
struct FloatVect3 filter_repforce_old = {0.0, 0.0, 0.0};
struct FloatVect3 Repulsionforce_Kan_old = {0.0, 0.0, 0.0};
float butter_old[AVOIDANCE_AMOUNT_OF_BOARDS *AVOIDANCE_HEIGHT_IN_MEASUREMENT_VALUES
                 *AVOIDANCE_WIDTH_IN_MEASUREMENT_VALUES];
//Vector Method
float F1 = 0.1;
float F2 = 0.9;
float Cfreq = 1;
float Ko = 60;//1000;//10000
float Kg = 100;//100;//100
float Dist_offset = 0;//-0.125;
int8_t dis_treshold = 2;
struct FloatVect3 Repulsionforce_Used;
struct FloatVect3 Repulsionforce_Kan;
struct FloatVect3 Attractforce_goal;
//Potential Method
float  b_damp = 0;
float  K_goal = 1;
float  K_obst = 40;
float  c1_oa = 0.4;
float  c2_oa = 0.4;
float  c3_oa = 5.0;
float  c4_oa = 0.5;
float  c5_oa = 0.9;
float  kv = 0.05;
float  epsilon = 0.0;
float  vmin = 0.2;
uint8_t point_index = 0;
//variables escape method
uint8_t set_bias = 0;
uint8_t obstacle_flag = 0;
uint8_t direction = 0;
float waypoint_rot = 0;
float new_heading_old;
uint8_t escape_flag = 0;
float v_min = 0.3;
float v_max = 0.6;
//variables hysteris
float V_hys_low = 5;
float V_hys_high = 10;
uint8_t hysteris_flag;
//variables for logger
struct FloatVect3 Attractforce_goal_send;
struct FloatVect3 Repulsionforce_Kan_send;

//static void send_OA_DATA(void)
//{
//  //DOWNLINK_SEND_OA_DATA(DefaultChannel, DefaultDevice, &target.x, &target.y, &pos_diff.x, &pos_diff.y, &ref_pitch,
////                        &ref_roll, &Attractforce_goal_send.x, &Attractforce_goal_send.y, &Repulsionforce_Kan_send.x,
////                        &Repulsionforce_Kan_send.y);
//}

void serial_init(void)
{

  for (int i_fill = 0;
       i_fill < (AVOIDANCE_AMOUNT_OF_BOARDS * AVOIDANCE_HEIGHT_IN_MEASUREMENT_VALUES * AVOIDANCE_WIDTH_IN_MEASUREMENT_VALUES);
       i_fill++) {
    Pest_new[i_fill] = 1;
    Xest_new[i_fill] = 1;
    butter_old[i_fill] = 0;
  }
//  register_periodic_telemetry(DefaultPeriodic, "OA_DATA", send_OA_DATA);
}

void serial_start(void)
{
  //printf("serial start\n");
}

void setAnglesMeasurements(float *anglesMeasurements, float *centersensorRad, float *fieldOfViewRad,
                           uint16_t *size_matrix_local)
{

  for (int i1 = 0; i1 < size_matrix_local[0]; i1++) {
    for (int i3 = 0; i3 < size_matrix_local[2]; i3++) {
      anglesMeasurements[i1 * size_matrix_local[0] + i3] = centersensorRad[i1] - 0.5 * fieldOfViewRad[0] + fieldOfViewRad[0] /
          size_matrix_local[2] / 2 + (fieldOfViewRad[0] / size_matrix_local[2]) * i3;
    }
  }
}

void serial_update(void)
{
  if (stereocam_data.fresh) {
    printf("Stereo fresh: %i \n", stereocam_data.len);

    float distancesMeters[stereocam_data.len];
    float anglesMeasurements[stereocam_data.matrix_width];

    //stereocam_disparity_to_meters(stereocam_data.data,distancesMeters,stereocam_data.len);
    for (int i_print1 = 0; i_print1 < AVOIDANCE_AMOUNT_OF_BOARDS; i_print1++) {
      for (int i_print2 = 0; i_print2 < (stereocam_data.len / AVOIDANCE_AMOUNT_OF_BOARDS); i_print2++) {
        printf("%3d,", stereocam_data.data[i_print1 * (stereocam_data.len / AVOIDANCE_AMOUNT_OF_BOARDS) + i_print2]);
      }
      printf("\n");
    }

    ///////////////////calcuate control reference/////////////////////////////////////////////////////
    if (OA_method_flag == PINGPONG) {
      //Calculate angles + distances
      setAnglesMeasurements(anglesMeasurements, angle_hor_board, stereo_fow, size_matrix);
      stereocam_disparity_to_meters(stereocam_data.data, distancesMeters, stereocam_data.len);

      matrix_2_pingpong(distancesMeters, size_matrix, distances_hor);
      pingpong_euler(distances_hor, anglesMeasurements, stereocam_data.matrix_width, reference_pitch, reference_roll,
                     dist_treshold);
    }

    if (OA_method_flag == POT_HEADING) {
      CN_calculate_target();
      CN_potential_heading();
    }

    if (OA_method_flag == POT_VEL) {
      CN_calculate_target();
      CN_potential_velocity();
    }

    if (OA_method_flag == VECTOR) {
      CN_calculate_target();
      CN_vector_velocity();
    }
    if (OA_method_flag == SAFETYZONE) {
      CN_calculate_target();
      CN_vector_escape_velocity();
    }
    if (OA_method_flag == LOGICBASED) {
      CN_calculate_target();
      CN_escape_velocity();
    }

    stereocam_data.fresh = 0;
    if (stereocam_data.len > 50) {
      DOWNLINK_SEND_MULTIGAZE_METERS(DefaultChannel, DefaultDevice, 50, distancesMeters);

    } else {
      DOWNLINK_SEND_MULTIGAZE_METERS(DefaultChannel, DefaultDevice, stereocam_data.len, distancesMeters);

    }
  }

}
void matrix_2_pingpong(float *distancesMeters, uint16_t *size_matrix_local, float *distances_hor_local)
{
  float distances_hor_local_old[AVOIDANCES_DISTANCES_HOR_COUNT];
  float distances_hor_local_new[AVOIDANCES_DISTANCES_HOR_COUNT];

  for (int i_m = 0; i_m < size_matrix_local[0]; i_m++) {
    for (int i_m3 = 0; i_m3 < size_matrix_local[2]; i_m3++) {
      distances_hor_local[i_m * size_matrix_local[2] + i_m3] = 10000;
      distances_hor_local_new[i_m * size_matrix_local[2] + i_m3] = 10000;
      for (int i_m2 = 0; i_m2 < 4; i_m2++) {
        if (distancesMeters[i_m * size_matrix_local[1] + i_m2 * size_matrix_local[0]*size_matrix_local[2] + i_m3] <
            distances_hor_local[i_m * size_matrix_local[2] + i_m3]) {
          distances_hor_local_old[i_m * size_matrix_local[2] + i_m3] = distances_hor_local_new[i_m * size_matrix_local[2] + i_m3];
          distances_hor_local_new[i_m * size_matrix_local[2] + i_m3] = distancesMeters[i_m * size_matrix_local[1] + i_m2 *
              size_matrix_local[0] * size_matrix_local[2] + i_m3];

          distances_hor_local[i_m * size_matrix_local[2] + i_m3] = (distances_hor_local_old[i_m * size_matrix_local[2] + i_m3] +
              distances_hor_local_new[i_m * size_matrix_local[2] + i_m3]) / 2;
        }
      }
      //    printf("index: %i %i, %f",i_m,i_m3,distances_hor[i_m*size_matrix[2] + i_m3]);
    }
  }

}

void CN_matrix_Kalman_filter(void)
{
  //TODO implement kalman filter
  //Parameters for Kalman filter
  float Xpred_new[size_matrix[0]*size_matrix[1]*size_matrix[2]];
  float Xest_old[size_matrix[0]*size_matrix[1]*size_matrix[2]];
  float Ppred_new[size_matrix[0]*size_matrix[1]*size_matrix[2]];
  float Pest_old[size_matrix[0]*size_matrix[1]*size_matrix[2]];
  float K_gain[size_matrix[0]*size_matrix[1]*size_matrix[2]];

  //Kallman filter on disparity matrix
  for (int i_k = 0; i_k < (size_matrix[0]*size_matrix[1]*size_matrix[2]); i_k++) {
    Pest_old[i_k] = Pest_new[i_k];
    Xest_old[i_k] = Xest_new[i_k];

    //one step ahead prediction
    Xpred_new[i_k] = A_kal * Xest_old[i_k];

    //Covariance matrix of state prediction error
    Ppred_new[i_k] = A_kal * Pest_old[i_k] * A_kal + Q_kal;

    //Kalman gain calculation
    K_gain[i_k] = Ppred_new[i_k] * H_kal * 1.0 / (H_kal * Ppred_new[i_k] * H_kal + R_kal);

    //Measurement update
    Xest_new[i_k] = Xpred_new[i_k] + K_gain[i_k] * (float)(stereocam_data.data[i_k] - H_kal * Xpred_new[i_k]);

    //Covariance matrix of state estimation error
    Pest_new[i_k] = (1 - K_gain[i_k] * H_kal) * Ppred_new[i_k];
  }
}

void CN_matrix_butterworth(void)
{
  float butter[size_matrix[0]*size_matrix[1]*size_matrix[2]];
  float A_butter = -0.7265;
  float B_butter[2] = {0.1367, 0.1367};


  for (int i_k = 0; i_k < (size_matrix[0] * size_matrix[1] * size_matrix[2]); i_k++) {

    if ((READimageBuffer_old[i_k] - stereocam_data.data[i_k]) <= 1
        && (READimageBuffer_old[i_k] - stereocam_data.data[i_k]) > 0) {
      stereocam_data.data[i_k] = READimageBuffer_old[i_k];

    }

    butter[i_k] = B_butter[0] * (float)stereocam_data.data[i_k] + B_butter[1] * (float)READimageBuffer_old[i_k] - A_butter *
                  butter_old[i_k];
    butter_old[i_k] = butter[i_k];
  }

  for (int ifill = 0; ifill < (size_matrix[0] * size_matrix[1] * size_matrix[2]); ifill++) {
    READimageBuffer_old[ifill] = stereocam_data.data[ifill];
  }

}

void CN_calculate_target(void)
{
  current_pos = *stateGetPositionNed_f();

  float psi = stateGetNedToBodyEulers_f()->psi;
  float s_psi = sin(psi);
  float c_psi = cos(psi);
  float s_waypoint_rot = sin(waypoint_rot);
  float c_waypoint_rot = cos(waypoint_rot);
  float dx_ref_NED;
  float dy_ref_NED;

  if (OA_method_flag == 2) {
    dx_ref_NED = dx_ref;
    dy_ref_NED = dy_ref;

  } else {
    dx_ref_NED = c_psi * dx_ref - s_psi * dy_ref;
    dy_ref_NED = s_psi * dx_ref + c_psi * dy_ref;
  }

  target.x = init_target.x + dx_ref_NED;
  target.y = init_target.y + dy_ref_NED;

  //apply rotation of waypoint
  if (OA_method_flag == 6) {
    target.x = c_waypoint_rot * target.x - s_waypoint_rot * target.y;
    target.y = s_waypoint_rot * target.x + c_waypoint_rot * target.y;
  }

  pos_diff.x = target.x - current_pos.x;
  pos_diff.y = target.y - current_pos.y;
  heading_goal_f = atan2(pos_diff.y, pos_diff.x);

}

void pingpong_euler(float *distances_hor_local, float *horizontalAnglesMeasurements, int horizontalAmountOfMeasurements,
                    float attitude_reference_pitch, float attitude_reference_roll, float dist_treshold_local)
{

  //init
  float sumPitch = 0.0;
  float sumRoll = 0.0;

  float oa_pitch_angle[horizontalAmountOfMeasurements];
  float oa_roll_angle[horizontalAmountOfMeasurements];

  for (int horizontal_index = 0; horizontal_index < horizontalAmountOfMeasurements; horizontal_index++) {

    //  printf("index: %i,distance %f",horizontal_index, distances_hor[horizontal_index]);
    if (distances_hor_local[horizontal_index] < dist_treshold_local) {

      oa_pitch_angle[horizontal_index] = cos(horizontalAnglesMeasurements[horizontal_index]) * attitude_reference_pitch;
      oa_roll_angle[horizontal_index] = -sin(horizontalAnglesMeasurements[horizontal_index]) * attitude_reference_roll;
      sumPitch += oa_pitch_angle[horizontal_index];
      sumRoll += oa_roll_angle[horizontal_index];

    }
  }

  if (sumPitch > attitude_reference_pitch) {
    ref_pitch = attitude_reference_pitch;
  } else if (sumPitch < -attitude_reference_pitch) {
    ref_pitch = -attitude_reference_pitch;
  } else {
    ref_pitch = sumPitch;
  }


  if (sumRoll > attitude_reference_roll) {
    ref_roll = attitude_reference_roll;
  } else if (sumRoll < -attitude_reference_roll) {
    ref_roll = -attitude_reference_roll;
  } else {
    ref_roll = sumRoll;
  }


  printf("DegOfRad data %f %f\n", ref_pitch, ref_roll);


}

void CN_potential_heading(void)
{
  //float OF_Result_Vy = 0;
  //float OF_Result_Vx = 0

  //Initialize
  float potential_obst = 0;  //define potential field variables
  float potential_obst_temp = 0;
  float potential_obst_integrated = 0;
  float Distance_est;
  float angle_hor = - 0.5 * stereo_fow[0] - stereo_fow[0] / size_matrix[2] + stereo_fow[0] / size_matrix[2] /
                    2; //calculate position angle of stereo
  float angle_hor_abs;
  float obst_count = 0.0;
  //float total_vel;
  //float current_heading;
  //float fp_angle;

  //Tune
  //float dt = 0.5;  //define delta t for integration! check response under 0.1 and 1
  int8_t min_disparity = 40;

  //Current state of system;
  float r_old = stateGetBodyRates_f()->r;
  //current_heading = stateGetNedToBodyEulers_f()->psi; //check maybe cause of errors
  //current_heading = 0;

  //check if side slip is zero
  //total_vel = pow((OF_Result_Vy*OF_Result_Vy + OF_Result_Vx*OF_Result_Vx),0.5);

  //if (total_vel>vmin){
  //    fp_angle = atan2(OF_Result_Vx,OF_Result_Vy);
  //}
  //else{
  //    fp_angle = 0;
  //}
  //fp_angle = 0;

  heading_goal_ref = stateGetNedToBodyEulers_f()->psi - heading_goal_f;//
  FLOAT_ANGLE_NORMALIZE(heading_goal_ref);


  for (int i1 = 0; i1 < size_matrix[0]; i1++) {
    angle_hor = angle_hor_board[i1] - 0.5 * stereo_fow[0] - stereo_fow[0] / size_matrix[2] + stereo_fow[0] / size_matrix[2]
                / 2; //Check if bodyframe is correct with current_heading correction
    for (int i3 = 0; i3 < size_matrix[2]; i3++) {

      angle_hor = angle_hor + stereo_fow[0] / size_matrix[2];
      FLOAT_ANGLE_NORMALIZE(angle_hor);
      potential_obst_temp = 0.0;
      obst_count = 0;

      for (int i2 = 0; i2 < 4; i2++) {

        if (stereocam_data.data[i1 * size_matrix[1] + i2 * size_matrix[0]*size_matrix[2] + i3] > min_disparity) {
          Distance_est = ((baseline * (float)focal / (float)stereocam_data.data[i1 * size_matrix[1] + i2 * size_matrix[0] *
                           size_matrix[2] + i3] - 18.0)) / 1000;

          if (angle_hor < 0) {
            angle_hor_abs = -1 * angle_hor;
          } else {
            angle_hor_abs = angle_hor;
          }
          potential_obst_temp = potential_obst_temp - K_obst * (angle_hor) * exp(-c3_oa * angle_hor_abs) * exp(
                                  -c4_oa * Distance_est);//(tan(obst_width[i]+c5)-tan(c5));
          potential_obst_integrated = potential_obst_integrated + K_obst * c3_oa * (fabs(angle_hor) + 1) / (c3_oa * c3_oa) * exp(
                                        -c3_oa * fabs(angle_hor)) * exp(-c4_oa * Distance_est); // (tan(obst_width[i]+c5)-tan(c5));
          //printf("angle_hor: %f, angle_hor_abs: %f, c3: %f, potential_obst: %f", angle_hor, angle_hor_abs,c3_oa,potential_obst);
          obst_count++;

        } else {
          Distance_est = 2000;
        }

        //potential_obst = potential_obst + K_obst*(angle_hor);//*exp(-c3*abs(angle_hor))*exp(-c4*Distance_est);//(tan(obst_width[i]+c5)-tan(c5));
        //potential_obst_write = potential_obst;
        //potential_obst_integrated = potential_obst_integrated + K_obst*c3*(abs(angle_hor)+1)/(c3*c3)*exp(-c3*abs(angle_hor))*exp(-c4*Distance_est);// (tan(obst_width[i]+c5)-tan(c5));

      }
      if (obst_count != 0) {
        potential_obst = potential_obst + potential_obst_temp / ((float)obst_count);
      }
    }
  }

  //printf("current_heading, %f heading_goal_f: %f",current_heading, heading_goal_ref);
  //calculate angular accelaration from potential field

  r_dot_new = -b_damp * r_old - K_goal * (heading_goal_ref) * (exp(-c1_oa * sqrt(VECT2_NORM2(
                pos_diff))) + c2_oa) + potential_obst;

  // Calculate velocity from potential
  speed_pot = vref_max * exp(-kv * potential_obst_integrated) * erf(0.5 * sqrt(VECT2_NORM2(pos_diff))) - epsilon;
  if (speed_pot <= 0) {
    speed_pot = 0;
  }

  //calculate new_heading(ref_pitch)
  //ref_pitch = (current_state(3)+sideslip) + alpha_fil*r_dot_new;)

}

void CN_potential_velocity(void)
{
  float OF_Result_Vy = 0;
  float OF_Result_Vx = 0;

  //Constants
  float new_heading;
  float alpha_fil = 0.2;

  //Initialize
  float potential_obst = 0;  //define potential field variables
  float potential_obst_integrated = 0;
  float Distance_est;
  float fp_angle;
  float diff_angle;
  float total_vel;
  float current_heading;
  float angle_hor = - 0.5 * stereo_fow[0] - stereo_fow[0] / size_matrix[2] + stereo_fow[0] / size_matrix[2] /
                    2; //calculate position angle of stereo

  //Tune
  //float dt = 0.5;  //define delta t for integration! check response under 0.1 and 1
  int8_t min_disparity = 40;

  //Current state of system;
  float r_old = stateGetBodyRates_f()->r;
  current_heading = stateGetNedToBodyEulers_f()->psi; //check maybe cause of errors
  //current_heading = 0;

  total_vel = pow((OF_Result_Vy * OF_Result_Vy + OF_Result_Vx * OF_Result_Vx), 0.5);

  if (total_vel > vmin) {
    fp_angle = atan2(OF_Result_Vx, OF_Result_Vy);
  } else {
    fp_angle = 0;
  }

  heading_goal_ref = (fp_angle + current_heading) - heading_goal_f;
  FLOAT_ANGLE_NORMALIZE(heading_goal_ref);


  for (int i1 = 0; i1 < size_matrix[0]; i1++) {
    angle_hor = angle_hor_board[i1] - 0.5 * stereo_fow[0] - stereo_fow[0] / size_matrix[2] + stereo_fow[0] / size_matrix[2]
                / 2; //Check if bodyframe is correct with current_heading correction
    for (int i3 = 0; i3 < size_matrix[2]; i3++) {
      angle_hor = angle_hor + stereo_fow[0] / size_matrix[2];
      FLOAT_ANGLE_NORMALIZE(angle_hor);

      for (int i2 = 0; i2 < size_matrix[1]; i2++) {

        if (stereocam_data.data[i1 * size_matrix[1] + i2 * size_matrix[0]*size_matrix[2] + i3] > min_disparity) {
          Distance_est = ((baseline * (float)focal / (float)stereocam_data.data[i1 * size_matrix[1] + i2 * size_matrix[0] *
                           size_matrix[2] + i3] - 18.0)) / 1000;
        } else {
          Distance_est = 2000;
        }
        diff_angle = fp_angle - angle_hor;
        FLOAT_ANGLE_NORMALIZE(diff_angle);
        potential_obst = potential_obst + K_obst * (diff_angle) * exp(-c3_oa * fabs(diff_angle)) * exp(
                           -c4_oa * Distance_est); //(tan(obst_width[i]+c5)-tan(c5));
        potential_obst_integrated = potential_obst_integrated + K_obst * c3_oa * (fabs(diff_angle) + 1) / (c3_oa * c3_oa) * exp(
                                      -c3_oa * fabs(diff_angle)) * exp(-c4_oa * Distance_est); // (tan(obst_width[i]+c5)-tan(c5));

      }
    }
  }

  //calculate angular accelaration from potential field
  r_dot_new = -b_damp * r_old - K_goal * (heading_goal_ref) * (exp(-c1_oa * sqrt(VECT2_NORM2(
                pos_diff))) + c2_oa) + potential_obst;

  // Calculate velocity from potential
  speed_pot = vref_max * exp(-kv * potential_obst_integrated) * erf(0.5 * sqrt(VECT2_NORM2(pos_diff))) - epsilon;
  if (speed_pot <= 0) {
    speed_pot = 0;
  }

  //calculate new_ref_speeds
  new_heading = fp_angle + alpha_fil * r_dot_new;
  FLOAT_ANGLE_NORMALIZE(new_heading);

  ref_pitch = new_heading;
}

void CN_vector_velocity(void)
{

  //Constants
  //Parameters for Butterworth filter
  float A_butter = -0.8541;//-0.7265;
  float B_butter[2] = {0.0730 , 0.0730 };//{0.1367, 0.1367};

  //Initalize
  int8_t disp_count = 0;
  float escape_angle = 0;
  float y_goal_frame;
  //float total_vel;
  float Distance_est;
  float Ca;
  float Cv;
  float angle_ver = 0;
  float angle_hor = 0;
  //struct FloatVect3 Repulsionforce_Kan = {0,0,0};
  Repulsionforce_Kan.x = 0;
  Repulsionforce_Kan.y = 0;
  Repulsionforce_Kan.z = 0;
  //struct FloatVect3 Attractforce_goal = {0,0,0};
  //Attractforce_goal.x = 0;
  //Attractforce_goal.y = 0;
  //Attractforce_goal.z = 0;
  //struct FloatRMat T;
  //struct FloatEulers current_attitude = {0,0,0};
  struct FloatVect3 Total_Kan = {0, 0, 0};
  //Tuning variables
  //float force_max = 200;
  int8_t min_disparity = 45;

  //Flight plath angle calculation
  // TODO make algorithm dependent on angle of obstacle.....
  //     total_vel = pow((OF_Result_Vy*OF_Result_Vy + OF_Result_Vx*OF_Result_Vx),0.5);

  //       if (total_vel>vmin){
  //    fp_angle = atan2(OF_Result_Vx,OF_Result_Vy);
  //       }
  //       else{
  //    fp_angle = 0;
  //       }

  heading_goal_ref = heading_goal_f - stateGetNedToBodyEulers_f()->psi;
  //FLOAT_ANGLE_NORMALIZE(heading_goal_ref);

  //Calculate Attractforce_goal size = 1;
  Attractforce_goal.x = cos(heading_goal_ref) * erf(0.5 * sqrt(VECT2_NORM2(pos_diff)));
  Attractforce_goal.y = sin(heading_goal_ref) * erf(0.5 * sqrt(VECT2_NORM2(pos_diff)));
  Attractforce_goal.z = 0;


  //printf("current_heading, %f heading_goal_f: %f, heading_goal_ref: %f",stateGetNedToBodyEulers_f()->psi, heading_goal_f, heading_goal_ref);
  //Transform to body frame
  //current_attitude.psi = stateGetNedToBodyEulers_f()->psi;Attractforce_already in body frame, check when using data form Roland
  //float_rmat_of_eulers_312(&T, &current_attitude);
  //MAT33_VECT3_MUL(Attractforce_goal, T, Attractforce_goal);

  //Attractforce_goal_send.x = Attractforce_goal.x;
  //Attractforce_goal_send.y = Attractforce_goal.y;
  //Attractforce_goal_send.z = Attractforce_goal.z;


  for (int i1 = 0; i1 < size_matrix[0]; i1++) {
    angle_hor = angle_hor_board[i1] - 0.5 * stereo_fow[0] - stereo_fow[0] / size_matrix[2] + stereo_fow[0] / size_matrix[2]
                / 2; //Check if bodyframe is correct with current_heading correction

    for (int i3 = 0; i3 < size_matrix[2]; i3++) {
      angle_hor = angle_hor + stereo_fow[0] / size_matrix[2];
      angle_ver = 0.5 * stereo_fow[1] + stereo_fow[1] / size_matrix[1] - stereo_fow[1] / size_matrix[1] / 2;

      for (int i2 = 0; i2 < 4; i2++) {
        angle_ver = angle_ver - stereo_fow[1] / size_matrix[1];

        Ca = cos((angle_hor - heading_goal_ref) * Cfreq) * erf(1 * sqrt(VECT2_NORM2(pos_diff)));
        if (Ca < 0) {
          Ca = 0;
        }

        //TODO  make dependent on speed: total_vel/vref_max
        Cv = F1 + F2 * Ca;
        if (stereocam_data.data[i1 * size_matrix[1] + i2 * size_matrix[0]*size_matrix[2] + i3] > min_disparity) {
          Distance_est = (baseline * (float)focal / ((float)stereocam_data.data[i1 * size_matrix[1] + i2 * size_matrix[0] *
                          size_matrix[2] + i3]) - 18.0) / 1000;
          disp_count++;
          Repulsionforce_Kan.x = Repulsionforce_Kan.x - pow(Cv / (Distance_est + Dist_offset),
                                 2) * cos(angle_hor) * cos(angle_ver);
          Repulsionforce_Kan.y = Repulsionforce_Kan.y - pow(Cv / (Distance_est + Dist_offset),
                                 2) * sin(angle_hor) * cos(angle_ver);
          Repulsionforce_Kan.z = Repulsionforce_Kan.z - pow(Cv / (Distance_est + Dist_offset), 2) * sin(angle_ver);

          printf("rep.x  %f index %d %d %d disp: %d cv: %f angle_hor: %f angle_ver: %f \n", Repulsionforce_Kan.x, i1, i2, i3,
                 stereocam_data.data[i1 * size_matrix[1] + i2 * size_matrix[0]*size_matrix[2] + i3], Cv, angle_hor, angle_ver);

        }
//         else {
//          Distance_est = 2000;
//      }
//
//      if(pow(Cv/(READimageBuffer_offset[i1*size_matrix[1]+i2*size_matrix[0]*size_matrix[2] + i3]-0.2),2)<force_max){
//          Repulsionforce_Kan.x = Repulsionforce_Kan.x - pow(Cv/(Distance_est+Dist_offset),2)*cos(angle_hor)*cos(angle_ver);
//          Repulsionforce_Kan.y = Repulsionforce_Kan.y - pow(Cv/(Distance_est+Dist_offset),2)*sin(angle_hor)*cos(angle_ver);
//          Repulsionforce_Kan.z = Repulsionforce_Kan.z - pow(Cv/(Distance_est+Dist_offset),2)*sin(angle_ver);
//      }
//      else{
//          Repulsionforce_Kan.x = Repulsionforce_Kan.x - force_max*sin(angle_hor)*cos(angle_ver);
//          Repulsionforce_Kan.y = Repulsionforce_Kan.y - force_max*cos(angle_hor)*cos(angle_ver);
//          Repulsionforce_Kan.z = Repulsionforce_Kan.z - force_max*sin(angle_ver);
//      }

      }
    }
  }

  //Normalize for ammount entries in Matrix
  //Repulsionforce_Kan = Repulsionforce_Kan/(float)(size_matrix[1]*size_matrix[2]);
  VECT3_SMUL(Repulsionforce_Kan, Repulsionforce_Kan, 1.0 / (float)disp_count);
  printf("After multiplication: %f", Repulsionforce_Kan.x);

  if (repulsionforce_filter_flag == 1) {

    Repulsionforce_Used.x = B_butter[0] * Repulsionforce_Kan.x + B_butter[1] * Repulsionforce_Kan_old.x - A_butter *
                            filter_repforce_old.x;
    Repulsionforce_Used.y = B_butter[0] * Repulsionforce_Kan.y + B_butter[1] * Repulsionforce_Kan_old.y - A_butter *
                            filter_repforce_old.y;
    Repulsionforce_Used.z = B_butter[0] * Repulsionforce_Kan.z + B_butter[1] * Repulsionforce_Kan_old.z - A_butter *
                            filter_repforce_old.z;

    VECT3_COPY(Repulsionforce_Kan_old, Repulsionforce_Kan);
    VECT3_COPY(filter_repforce_old, Repulsionforce_Used);

  }

  else {
    VECT3_COPY(Repulsionforce_Used, Repulsionforce_Kan);
  }

  VECT3_SMUL(Repulsionforce_Used, Repulsionforce_Used, Ko);
  VECT3_SMUL(Attractforce_goal, Attractforce_goal, Kg);

  //Total force
  VECT3_ADD(Total_Kan, Repulsionforce_Used);
  VECT3_ADD(Total_Kan, Attractforce_goal);

  //set variable for stabilization_opticflow
  //printf("RepulsionforceX: %f AttractX: %f \n",Repulsionforce_Used.x,Attractforce_goal.x);
  //printf("RepulsionforceY: %f AttractY: %f \n",Repulsionforce_Used.y,Attractforce_goal.y);


  //hysteris
  if (sqrt(VECT2_NORM2(Total_Kan)) < V_hys_low && sqrt(VECT2_NORM2(pos_diff)) > 1) {
    hysteris_flag = 1;

    //x_goal_frame= cos() * Total_Kan.x + sin() * Total_Kan.y;
    y_goal_frame = -sin(heading_goal_ref) * Total_Kan.x + cos(heading_goal_ref) * Total_Kan.y;

    if (y_goal_frame >= 0) {
      escape_angle = 0.5 * M_PI;
    } else if (y_goal_frame < 0) {
      escape_angle = -0.5 * M_PI;
    }
  }


  if (sqrt(VECT2_NORM2(Total_Kan)) > V_hys_high || sqrt(VECT2_NORM2(pos_diff)) < 1) {
    hysteris_flag = 0;
  }

  if (hysteris_flag == 1) {

    Total_Kan.x = cos(heading_goal_ref + escape_angle) * V_hys_high;
    Total_Kan.y = -sin(heading_goal_ref + escape_angle) * V_hys_high;

  }


  ref_pitch = Total_Kan.x;
  ref_roll = Total_Kan.y;
  printf("ref_pitch:  %f ref_roll: %f disp_count: %d", ref_pitch, ref_roll, disp_count);
  //set write values for logger
  //Attractforce_goal_send.x = Attractforce_goal.x;
  //Attractforce_goal_send.y = Attractforce_goal.y;
  //Repulsionforce_Kan_send.x = Repulsionforce_Used.x;
  //Repulsionforce_Kan_send.y = Repulsionforce_Used.y;
}

void CN_vector_escape_velocity(void)
{

  /////VECTOR METHOD VARIABLES//////
  //Constants
  //Parameters for Butterworth filter
  float A_butter = -0.8541;//-0.7265;
  float B_butter[2] = {0.0730 , 0.0730 };//{0.1367, 0.1367};

  int8_t disp_count = 0;
  //Initalize
  //float fp_angle;
  //float total_vel;
  float Ca;
  float Cv;
  float angle_ver = 0;
  float angle_hor = 0;
  Repulsionforce_Kan.x = 0;
  Repulsionforce_Kan.y = 0;
  Repulsionforce_Kan.z = 0;
  //struct FloatRMat T;
  //struct FloatEulers current_attitude = {0,0,0};
  struct FloatVect3 Total_Kan = {0, 0, 0};
  //Tuning variables
  //float force_max = 200;

  /////ESCAPE METHOD VARIABLES//////
  //Constants
  int8_t min_disparity = 45;
  int8_t number_of_buffer = 20;
  float bias = 2 * M_PI;
  vref_max = 0.1; //CHECK!
  //init
  float available_heading[size_matrix[0] * size_matrix[2]];
  float diff_available_heading[size_matrix[0] * size_matrix[2]];
  float new_heading_diff = 1000;
  float new_heading = 1000;
  int8_t i = 0;
  int8_t i_buffer = 0;
  float Distance_est;
  float V_total = 0;
  ///////////////////////////////////

  heading_goal_ref = heading_goal_f - stateGetNedToBodyEulers_f()->psi;
  //FLOAT_ANGLE_NORMALIZE(heading_goal_ref);

  //Calculate Attractforce_goal
  Attractforce_goal.x = cos(heading_goal_ref) * erf(0.5 * sqrt(VECT2_NORM2(pos_diff)));
  Attractforce_goal.y = sin(heading_goal_ref) * erf(0.5 * sqrt(VECT2_NORM2(pos_diff)));
  Attractforce_goal.z = 0;

  for (int i1 = 0; i1 < size_matrix[0]; i1++) {
    angle_hor = angle_hor_board[i1] - 0.5 * stereo_fow[0] - stereo_fow[0] / size_matrix[2] + stereo_fow[0] / size_matrix[2]
                / 2; //Check if bodyframe is correct with current_heading correction

    for (int i3 = 0; i3 < size_matrix[2]; i3++) {
      angle_hor = angle_hor + stereo_fow[0] / size_matrix[2];
      angle_ver = 0.5 * stereo_fow[1] + stereo_fow[1] / size_matrix[1] - stereo_fow[1] / size_matrix[1] / 2;

      for (int i2 = 0; i2 < 4; i2++) {
        angle_ver = angle_ver - stereo_fow[1] / size_matrix[1];

        Ca = cos((angle_hor - heading_goal_ref) * Cfreq) * erf(1 * sqrt(VECT2_NORM2(pos_diff)));
        if (Ca < 0) {
          Ca = 0;
        }

        //TODO  make dependent on speed: total_vel/vref_max
        Cv = F1 + F2 * Ca;

        if (stereocam_data.data[i1 * size_matrix[1] + i2 * size_matrix[0]*size_matrix[2] + i3] > min_disparity) {
          Distance_est = (baseline * (float)focal / ((float)stereocam_data.data[i1 * size_matrix[1] + i2 * size_matrix[0] *
                          size_matrix[2] + i3]) - 18.0) / 1000;
          disp_count++;
          Repulsionforce_Kan.x = Repulsionforce_Kan.x - pow(Cv / (Distance_est + Dist_offset),
                                 2) * cos(angle_hor) * cos(angle_ver);
          Repulsionforce_Kan.y = Repulsionforce_Kan.y - pow(Cv / (Distance_est + Dist_offset),
                                 2) * sin(angle_hor) * cos(angle_ver);
          Repulsionforce_Kan.z = Repulsionforce_Kan.z - pow(Cv / (Distance_est + Dist_offset), 2) * sin(angle_ver);


        }

      }
    }
  }

  //Normalize for ammount entries in Matrix
  VECT3_SMUL(Repulsionforce_Kan, Repulsionforce_Kan, 1.0 / (float)disp_count);

  if (repulsionforce_filter_flag == 1) {

    Repulsionforce_Used.x = B_butter[0] * Repulsionforce_Kan.x + B_butter[1] * Repulsionforce_Kan_old.x - A_butter *
                            filter_repforce_old.x;
    Repulsionforce_Used.y = B_butter[0] * Repulsionforce_Kan.y + B_butter[1] * Repulsionforce_Kan_old.y - A_butter *
                            filter_repforce_old.y;
    Repulsionforce_Used.z = B_butter[0] * Repulsionforce_Kan.z + B_butter[1] * Repulsionforce_Kan_old.z - A_butter *
                            filter_repforce_old.z;

    VECT3_COPY(Repulsionforce_Kan_old, Repulsionforce_Kan);
    VECT3_COPY(filter_repforce_old, Repulsionforce_Used);

  }

  else {
    VECT3_COPY(Repulsionforce_Used, Repulsionforce_Kan);
  }

  VECT3_SMUL(Repulsionforce_Used, Repulsionforce_Used, Ko);
  VECT3_SMUL(Attractforce_goal, Attractforce_goal, Kg);

  //Total force
  VECT3_ADD(Total_Kan, Repulsionforce_Used);
  VECT3_ADD(Total_Kan, Attractforce_goal);

  //set variable for stabilization_opticflow
  printf("RepulsionforceX: %f AttractX: %f \n", Repulsionforce_Used.x, Attractforce_goal.x);
  printf("RepulsionforceY: %f AttractY: %f \n", Repulsionforce_Used.y, Attractforce_goal.y);

  //set write values for logger
  //Attractforce_goal_send.x = Attractforce_goal.x;
  //Attractforce_goal_send.y = Attractforce_goal.y;
  //Repulsionforce_Kan_send.x = Repulsionforce_Used.x;
  //Repulsionforce_Kan_send.y = Repulsionforce_Used.y;

  V_total = sqrt(pow(Total_Kan.x, 2) + pow(Total_Kan.y, 2));

  if (V_total < v_min && sqrt(VECT2_NORM2(pos_diff)) > 1) {
    escape_flag = 1;
  }

  else if (V_total > v_max && obstacle_flag == 0) {
    escape_flag = 0;
  }

  printf("V_total: %f", V_total);
  printf("escape_flag: %i, obstacle_flag: %i, set_bias: %i", escape_flag, obstacle_flag, set_bias);
  printf("escape_flag: %i, obstacle_flag: %i, set_bias: %i", escape_flag, obstacle_flag, set_bias);

  if (escape_flag == 0) {
    ref_pitch = Total_Kan.x;
    ref_roll = Total_Kan.y;
  } else if (escape_flag == 1) {
    heading_goal_ref = heading_goal_f - stateGetNedToBodyEulers_f()->psi;

    for (int i1 = 0; i1 < size_matrix[0]; i1++) {
      angle_hor = angle_hor_board[i1] - 0.5 * stereo_fow[0] - stereo_fow[0] / size_matrix[2];
      for (int i3 = 0; i3 < size_matrix[2]; i3++) {
        angle_hor = angle_hor + stereo_fow[0] / size_matrix[2];
        available_heading[i] = angle_hor;
        diff_available_heading[i] = heading_goal_ref - available_heading[i];
        FLOAT_ANGLE_NORMALIZE(diff_available_heading[i]);
        i++;

      }
    }

    i = 0;
    for (int i1 = 0; i1 < size_matrix[0]; i1++) {
      for (int i3 = 0; i3 < size_matrix[2]; i3++) {
        for (int i2 = 0; i2 < 4; i2++) {
          if (stereocam_data.data[i1 * size_matrix[1] + i2 * size_matrix[0]*size_matrix[2] + i3] > min_disparity) {
            //distance_est = (baseline*(float)focal/((float)READimageBuffer_offset[i1*size_matrix[1]+i2*size_matrix[0]*size_matrix[2] + i3])-18.0)/1000;

            diff_available_heading[i] = 100;
            for (int i_diff = -number_of_buffer; i_diff <= number_of_buffer + 1; i_diff++) {
              i_buffer = i + i_diff;


              if (i_buffer < 1) {
                i_buffer = i_buffer + size_matrix[0] * size_matrix[2];
              }

              if (i_buffer > size_matrix[0]*size_matrix[2]) {
                i_buffer = i_buffer - size_matrix[0] * size_matrix[2];
              }

              diff_available_heading[i_buffer] = 100;
            }
          }
        }
        i++;
      }
    }

    //set bias
    if (set_bias == 1) {
      for (int i100 = 0; i100 < (size_matrix[0]*size_matrix[2]); i100++) {
        if (diff_available_heading[i100] <= 0 && (fabs(diff_available_heading[i100]) < M_PI - 5.0 / 360.0 * 2.0 * M_PI)) {
          diff_available_heading[i100] = diff_available_heading[i100] - bias;
        }
      }
    } else if (set_bias == 2) {
      for (int i100 = 0; i100 < (size_matrix[0]*size_matrix[2]); i100++) {
        if (diff_available_heading[i100] > 0 && (fabs(diff_available_heading[i100]) < M_PI - 5.0 / 360.0 * 2.0 * M_PI)) {
          diff_available_heading[i100] = diff_available_heading[i100] + bias;
        }
      }
    }

    for (int i100 = 0; i100 < (size_matrix[0]*size_matrix[2]); i100++) {
      if (fabs(diff_available_heading[i100]) < fabs(new_heading_diff)) {
        new_heading_diff = diff_available_heading[i100];
        new_heading = available_heading[i100];
      }
    }

    if (obstacle_flag == 0 && fabs(new_heading_diff) > (2 * M_PI / 36.0)) {
      obstacle_flag = 1;
      if (new_heading_diff > 0) {
        set_bias = 1;
        direction = -1;
      } else if (new_heading_diff <= 0) {
        set_bias = 2;
        direction = 1;
      }
    }

    if (fabs(new_heading_diff) <= (2 * M_PI / 36.0)) {
      if (waypoint_rot == 0) {
        obstacle_flag = 0;
        set_bias = 0;
      } else {
        waypoint_rot = waypoint_rot - direction * 0.05 * M_PI;
      }
    }

    if (fabs(new_heading_diff) >= 0.5 * M_PI) {
      waypoint_rot = waypoint_rot + direction * 0.25 * M_PI;
    }

    //vref_max should be low
    speed_pot = vref_max * erf(0.5 * sqrt(VECT2_NORM2(pos_diff)));
    if (speed_pot <= 0) {
      speed_pot = 0;
    }

    new_heading_old = new_heading;

#if PRINT_STUFF
    for (int i100 = 0; i100 < (size_matrix[0] * size_matrix[2]); i100++) {
      printf("%i diff_available_heading: %f available_heading: %f \n", i100, diff_available_heading[i100],
             available_heading[i100]);
    }
    printf("new_heading: %f  current_heading: %f  speed_pot: %f\n", new_heading, stateGetNedToBodyEulers_f()->psi,
           speed_pot);
    printf("set_bias: %i  obstacle_flag: %i", set_bias, obstacle_flag);
#endif

    ref_pitch = cos(new_heading) * speed_pot;
    ref_roll = sin(new_heading) * speed_pot;

  }
}

void CN_escape_velocity(void)
{

  //Constants
  int8_t min_disparity = 45;
  int8_t number_of_buffer = 20;
  float bias = 2 * M_PI;
  vref_max = 0.1; //CHECK!

  //init variables
  float angle_hor = 0;
  float available_heading[size_matrix[0]*size_matrix[2]];
  float diff_available_heading[size_matrix[0]*size_matrix[2]];
  float new_heading_diff = 1000;
  float new_heading = 1000;
  int8_t i = 0;
  int8_t i_buffer = 0;
  float distance_est;
  float distance_heading = 0;

  //generate available_headings
  heading_goal_ref = heading_goal_f - stateGetNedToBodyEulers_f()->psi;

  for (int i1 = 0; i1 < size_matrix[0]; i1++) {
    angle_hor = angle_hor_board[i1] - 0.5 * stereo_fow[0] - stereo_fow[0] / size_matrix[2];
    for (int i3 = 0; i3 < size_matrix[2]; i3++) {
      angle_hor = angle_hor + stereo_fow[0] / size_matrix[2];
      available_heading[i] = angle_hor;
      diff_available_heading[i] = heading_goal_ref - available_heading[i];
      FLOAT_ANGLE_NORMALIZE(diff_available_heading[i]);
      i++;

    }
  }

  //set unavailable headings to 100;
  i = 0;
  for (int i1 = 0; i1 < size_matrix[0]; i1++) {
    for (int i3 = 0; i3 < size_matrix[2]; i3++) {
      for (int i2 = 0; i2 < 4; i2++) {
        if (stereocam_data.data[i1 * size_matrix[1] + i2 * size_matrix[0]*size_matrix[2] + i3] > min_disparity) {
          distance_est = (baseline * (float)focal / ((float)stereocam_data.data[i1 * size_matrix[1] + i2 * size_matrix[0] *
                          size_matrix[2] + i3]) - 18.0) / 1000;

          if (distance_est < distance_heading) {
            distance_heading = distance_est;
          }

          diff_available_heading[i] = 100;
          for (int i_diff = -number_of_buffer; i_diff <= number_of_buffer + 1; i_diff++) {
            i_buffer = i + i_diff;


            if (i_buffer < 1) {
              i_buffer = i_buffer + size_matrix[0] * size_matrix[2];
            }

            if (i_buffer > size_matrix[0]*size_matrix[2]) {
              i_buffer = i_buffer - size_matrix[0] * size_matrix[2];
            }

            diff_available_heading[i_buffer] = 100;
          }

        }
      }
      i++;
    }
  }

  //set bias
  if (set_bias == 1) {
    for (int i100 = 0; i100 < (size_matrix[0]*size_matrix[2]); i100++) {
      if (diff_available_heading[i100] <= 0 && (fabs(diff_available_heading[i100]) < M_PI - 5.0 / 360.0 * 2.0 * M_PI)) {
        diff_available_heading[i100] = diff_available_heading[i100] - bias;
      }
    }
  } else if (set_bias == 2) {
    for (int i100 = 0; i100 < (size_matrix[0]*size_matrix[2]); i100++) {
      if (diff_available_heading[i100] > 0 && (fabs(diff_available_heading[i100]) < M_PI - 5.0 / 360.0 * 2.0 * M_PI)) {
        diff_available_heading[i100] = diff_available_heading[i100] + bias;
      }
    }
  }

  // select minimum available heading
  for (int i100 = 0; i100 < (size_matrix[0]*size_matrix[2]); i100++) {
    if (fabs(diff_available_heading[i100]) < fabs(new_heading_diff)) {
      new_heading_diff = diff_available_heading[i100];
      new_heading = available_heading[i100];
    }
  }

  if (obstacle_flag == 0 && distance_heading > 2.0) {
    new_heading = new_heading_old;
  } else if (obstacle_flag == 0 && fabs(new_heading_diff) > (2 * M_PI / 36.0)) {
    obstacle_flag = 1;
    if (new_heading_diff > 0) {
      set_bias = 1;
      direction = -1;
    } else if (new_heading_diff <= 0) {
      set_bias = 2;
      direction = 1;
    }
  }

  // Rotate waypoint
  if (fabs(new_heading_diff) >= 0.5 * M_PI) {
    waypoint_rot = waypoint_rot + direction * 0.25 * M_PI;
  }

  if (fabs(new_heading_diff) <= (2 * M_PI / 36.0)) {
    if (waypoint_rot == 0) {
      obstacle_flag = 0;
      set_bias = 0;
    } else {
      waypoint_rot = waypoint_rot - direction * 0.05 * M_PI;
    }
  }

  //vref_max should be low
  speed_pot = vref_max * erf(0.5 * sqrt(VECT2_NORM2(pos_diff)));
  if (speed_pot <= 0) {
    speed_pot = 0;
  }

  ref_pitch = cos(new_heading) * speed_pot;
  ref_roll = sin(new_heading) * speed_pot;
  new_heading_old = new_heading;

#if PRINT_STUFF
  for (int i100 = 0; i100 < (size_matrix[0]*size_matrix[2]); i100++) {
    printf("%i diff_available_heading: %f available_heading: %f \n", i100, diff_available_heading[i100],
           available_heading[i100]);
  }
  printf("new_heading: %f  current_heading: %f  speed_pot: %f\n", new_heading, stateGetNedToBodyEulers_f()->psi,
         speed_pot);
  printf("set_bias: %i  obstacle_flag: %i", set_bias, obstacle_flag);
#endif

}


