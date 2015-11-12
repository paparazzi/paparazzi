/*
 * Copyright (C) ROland
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
 * @file "modules/read_matrix_serial/read_matrix_serial.c"
 * @author ROland
 * reads from the serial
 */


#include <stdio.h>
#include <sys/fcntl.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <inttypes.h>
#include "state.h"
#include "read_matrix_serial.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_int.h"
#include "modules/read_matrix_serial/read_matrix_serial.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/stereo_cam/stereocam.h"

int size_matrix[3] = {1, 6, 6};

float ref_pitch_angle = 0.2;
uint16_t matrix_sum[6]={0,0,0,0,0,0};
uint16_t matrix_sum2[6]={0,0,0,0,0,0};
uint16_t matrix_sum_treshold = 0;
float matrix_treshold = 7.0;
float ref_roll=0.0;
float ref_pitch=0.0;
float ref_yaw=0.0;

uint16_t OA_method_flag = 1;
struct FloatVect3 Repulsionforce_Kan; 
struct FloatVect3 Attractforce_goal; 
int16_t focal = 118*6;
float baseline = 60;

void serial_init(void) {

}

void serial_start(void)
{
	//printf("serial start\n");
}
void serial_update(void)
{
	 if(stereocam_data.fresh){
	      if(OA_method_flag==1){
		      cal_euler_pingpong();
	      }
		  
	      if(OA_method_flag==2){
		     nav_cal_vel_vector_pingpong();
	      }
	}

  
}

void cal_euler_pingpong(void){
  	
	float oa_pitch_angle[6]={0,0,0,0,0,0};
	float oa_roll_angle[6]={0,0,0,0,0,0};

	// Do something with: stereocam_data.data;
	for (int i_m=0;i_m<size_matrix[0];i_m++){
	    matrix_sum[i_m] = 0;
	    matrix_sum2[i_m] = 0;
	    oa_pitch_angle[i_m] = 0;
	    oa_roll_angle[i_m] = 0;

	      for(int i_m2=0;i_m2<size_matrix[2];i_m2++){
		    for(int i_m3=0;i_m3<size_matrix[1];i_m3++){
//						stereocam_data.data[i_m*size_matrix[1]+i_m2*size_matrix[0]*size_matrix[2] + i_m3] = stereocam_data.data[i_m*size_matrix[1]+i_m2*size_matrix[0]*size_matrix[2] + i_m3];

			  if(stereocam_data.data[i_m*size_matrix[1]+i_m2*size_matrix[0]*size_matrix[2] + i_m3]>matrix_treshold){
			      matrix_sum[i_m] = matrix_sum[i_m] + 1;
			  }
// 					if((stereocam_data.data[i_m*size_matrix[1]+i_m2*size_matrix[0]*size_matrix[2] + i_m3]>(matrix_treshold-1))&& (stereocam_data.data[i_m*size_matrix[1]+i_m2*size_matrix[0]*size_matrix[2] + i_m3]<=matrix_treshold))
// 					   matrix_sum2[i_m] = matrix_sum2[i_m] + 1;
		    }
	      }
	}

	if (matrix_sum[0]>matrix_sum_treshold){
	      oa_pitch_angle[0] = ref_pitch_angle;
	      oa_roll_angle[0] = 0;
	  }

	  //limiter of control action
	  if((oa_pitch_angle[0] + oa_pitch_angle[1] + oa_pitch_angle[2] + oa_pitch_angle[3]+ oa_pitch_angle[4] + oa_pitch_angle[5])>ref_pitch_angle){
	    ref_pitch = ref_pitch_angle;
	  }
	  else if((oa_pitch_angle[0] + oa_pitch_angle[1] + oa_pitch_angle[2] + oa_pitch_angle[3]+ oa_pitch_angle[4] + oa_pitch_angle[5])<-ref_pitch_angle){
	    ref_pitch = -ref_pitch_angle;
	  }
	  else{
	    ref_pitch = oa_pitch_angle[0] + oa_pitch_angle[1] + oa_pitch_angle[2] + oa_pitch_angle[3]+ oa_pitch_angle[4] + oa_pitch_angle[5];
	  }


	  if((oa_roll_angle[0] + oa_roll_angle[1] + oa_roll_angle[2] + oa_roll_angle[3] + oa_roll_angle[4] + oa_roll_angle[5])>ref_pitch_angle)
	    {
	      ref_roll = ref_pitch_angle;
	    }
	    else if((oa_roll_angle[0] + oa_roll_angle[1] + oa_roll_angle[2] + oa_roll_angle[3] + oa_roll_angle[4] + oa_roll_angle[5])<-ref_pitch_angle)
	    {
	      ref_roll = -ref_pitch_angle;
	    }
	  else
	    {
	      ref_roll = oa_roll_angle[0] + oa_roll_angle[1] + oa_roll_angle[2] + oa_roll_angle[3] + oa_roll_angle[4] + oa_roll_angle[5];
	    }

}


 void nav_cal_vel_vector_pingpong(void){
  
      //follow variables;
      float follow_distance = 1.5;
      int8_t obst_count = 0;
      float obst_total = 0;
      int8_t column_count  =0;
      float column_total = 0;
      float min_measured_distance = 0;
      float min_measured_distance_old = 0;
      float measured_distance = 0;
      float avg_collumn = 0; 
      float yaw_ref = 0;
      
      //Constants
      float stereo_fow[2] = {1.0018, 0.7767};//based on FOW of 57.4, by 44.5
      float angle_hor_board [] = {0, 1.0472, 2.0944, 3.1416, -2.0944, -1.0472}; //init angle values in input matrix in body frame  
      //float angle_hor = - 0.5*stereo_fow[0] - stereo_fow[0]/size_matrix[2] + stereo_fow[0]/size_matrix[2]/2;
      
      //Initalize
      float Distance_est;
      float angle_ver = 0;
      float angle_hor = 0;
      Repulsionforce_Kan.x = 0;
      Repulsionforce_Kan.y = 0;
      Repulsionforce_Kan.z = 0;
      Attractforce_goal.x = 0;
      Attractforce_goal.y = 0;
      Attractforce_goal.z = 0;
      
      //Tuning variables
      int8_t min_distance = 0.2;
      int8_t max_distance = 2.5;
      
      
      for (int i1=0;i1<size_matrix[0];i1++){
	  angle_hor = angle_hor_board[i1] - 0.5*stereo_fow[0] - stereo_fow[0]/size_matrix[2] + stereo_fow[0]/size_matrix[2]/2;//Check if bodyframe is correct with current_heading correction
	  
	  for (int i3=0;i3<size_matrix[2];i3++){
	      angle_hor = angle_hor + stereo_fow[0]/size_matrix[2];
	      angle_ver = 0.5*stereo_fow[1] + stereo_fow[1]/size_matrix[1] - stereo_fow[1]/size_matrix[1]/2;
	      
	      column_count = 0;
	      column_total = 0;
	      for (int i2=0;i2<4;i2++){
		  angle_ver = angle_ver - stereo_fow[1]/size_matrix[1];
		  Distance_est = (baseline*(float)focal/((float)stereocam_data.data[i1*size_matrix[1]+i2*size_matrix[0]*size_matrix[2] + i3])-18.0)/1000;
		    
		      if(Distance_est>min_distance && Distance_est<max_distance){
			
			    column_count++;
			    column_total = column_total + Distance_est;
			    
			    obst_count++;
			    obst_total = obst_total + Distance_est;
			    
			    if(Distance_est<min_measured_distance){
			      min_measured_distance_old = min_measured_distance;
			      min_measured_distance = Distance_est;
			    }
		      }

	      }
	      if(avg_collumn>(column_total/(float)column_count)){
		    avg_collumn = (column_total/(float)column_count);
		    yaw_ref = angle_hor;
	      }
	  }
      }
      
      measured_distance = obst_total/(float)obst_count;
      Repulsionforce_Kan.x = 0.2*erf(3*(follow_distance-measured_distance));
      
      if(yaw_ref>stereo_fow[0]/size_matrix[2]){
	  Repulsionforce_Kan.y = stateGetNedToBodyEulers_f()->psi + 0.1*yaw_ref;
      }
      else{
	  Repulsionforce_Kan.y = stateGetNedToBodyEulers_f()->psi; 
      }  
      Repulsionforce_Kan.z  = 0;
      
      ref_pitch = Repulsionforce_Kan.x;
      ref_yaw = Repulsionforce_Kan.y;
      ref_roll = Repulsionforce_Kan.z;
     
      
      /*if(repulsionforce_filter_flag == 1){
	  				
	    Repulsionforce_Used.x = B_butter[0]*Repulsionforce_Kan.x + B_butter[1]*Repulsionforce_Kan_old.x - A_butter*filter_repforce_old.x;
	    Repulsionforce_Used.y = B_butter[0]*Repulsionforce_Kan.y + B_butter[1]*Repulsionforce_Kan_old.y - A_butter*filter_repforce_old.y;
	    Repulsionforce_Used.z = B_butter[0]*Repulsionforce_Kan.z + B_butter[1]*Repulsionforce_Kan_old.z - A_butter*filter_repforce_old.z;
	    
	    VECT3_COPY(Repulsionforce_Kan_old,Repulsionforce_Kan);
	    VECT3_COPY(filter_repforce_old,Repulsionforce_Used);
		
      }
      
      else{
	    VECT3_COPY(Repulsionforce_Used,Repulsionforce_Kan);
      }*/   
}





 
