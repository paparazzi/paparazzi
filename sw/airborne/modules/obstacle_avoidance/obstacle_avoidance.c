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
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_int.h"
#include "modules/obstacle_avoidance/obstacle_avoidance.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/stereo_cam/stereocam.h"

//TODO: clean variables
float ref_roll=0.0;
float ref_pitch=0.0;
float ref_yaw=0.0;
uint16_t OA_method_flag = 1;
struct FloatVect3 Repulsionforce_Kan; 
struct FloatVect3 Attractforce_goal; 
int16_t focal = 118*6;
float baseline = 60;

//////////////SET BY USER!!!!//////////////////

//sensor info
uint16_t size_matrix[] = {5,6,6};
float stereo_fow[2] = {1.0018, 0.7767};//based on FOW of 57.4, by 44.5
float angle_hor_board[] = {0, 1.0472, 2.0944, 3.1416, -2.0944, -1.0472}; 

//tuning info
float attitude_reference_pitch = 0.2;
float attitude_reference_roll = 0.2;
float dist_treshold = 0.75;

float distances_hor[36];

//////////////////////////////////////////////

void serial_init(void) {

}

void serial_start(void)
{
	//printf("serial start\n");
}

void setAnglesMeasurements(float *anglesMeasurements,float* centersensorRad, float* fieldOfViewRad, uint16_t* size_matrix){

	 for (int i1=0;i1<size_matrix[0];i1++){	   
	    for (int i3=0;i3<size_matrix[2];i3++){
	      anglesMeasurements[i1*size_matrix[0]+i3] = centersensorRad[i1] - 0.5*fieldOfViewRad[0] + fieldOfViewRad[0]/size_matrix[2]/2 + (fieldOfViewRad[0]/size_matrix[2])*i3;
	      }
	 }	 
}

void serial_update(void)
{
	 if(stereocam_data.fresh){
		 printf("Stereo fresh: %i \n",stereocam_data.len);

		 float distancesMeters[stereocam_data.len];
		 float anglesMeasurements[stereocam_data.matrix_width];
		 float centerRad = 3.4415926;
		 float fieldOfViewRadHorizontal=6.282;
		 int x=0;
		 float sumDistances=0.0;
		 
		 //OA_method 1
		 float forward_speed;
		 float heading;
		 
		 
		 //Calculate angles + distances 
		 setAnglesMeasurements(anglesMeasurements,angle_hor_board,stereo_fow,size_matrix);
		 stereocam_disparity_to_meters(stereocam_data.data,distancesMeters,stereocam_data.len);
		
		 for(int i_print1=0; i_print1<6; i_print1++){
		      for(int i_print2=0; i_print2<(stereocam_data.len/6); i_print2++){
			//  printf("%2.3f,",distancesMeters[i_print1*(stereocam_data.len/6) + i_print2]);
		      }
		  //    printf("\n");
		  }
		 
		 

		 		 
		 for(x=0; x < stereocam_data.len; x++){
			 sumDistances+=distancesMeters[x];
		 }
		// printf("Distance: %f\n",(sumDistances/stereocam_data.len));

		 
		 //calcuate control reference
		 if(OA_method_flag==1){
		      matrix_2_pingpong(distancesMeters, size_matrix, distances_hor);
		      cal_euler_pingpong(distances_hor, anglesMeasurements, stereocam_data.matrix_width, attitude_reference_pitch, attitude_reference_roll, dist_treshold);
	         }
		  
		 //if(OA_method_flag==2){
		 //     nav_cal_vel_vector_pingpong(distancesMeters,anglesMeasurements,stereocam_data.len,&forward_speed,&heading);
		 //}
		 
		stereocam_data.fresh=0;
		
		DOWNLINK_SEND_CLINT_AVOID(DefaultChannel, DefaultDevice, &ref_roll,&ref_pitch,&ref_yaw);
		DOWNLINK_SEND_MULTIGAZE_METERS(DefaultChannel, DefaultDevice, stereocam_data.len, distancesMeters);
	}

}

void matrix_2_pingpong(float* distancesMeters, int16_t* size_matrix, float* distances_hor){
 
	for(int i_m=0;i_m<size_matrix[0];i_m++){
	    for(int i_m3=0;i_m3<size_matrix[2];i_m3++){
	        distances_hor[i_m*size_matrix[2] + i_m3] = 10000;
		for(int i_m2=0;i_m2<4;i_m2++){
		   if(distancesMeters[i_m*size_matrix[1]+i_m2*size_matrix[0]*size_matrix[2] + i_m3]<distances_hor[i_m*size_matrix[2] + i_m3]){
		     distances_hor[i_m*size_matrix[2] + i_m3] = distancesMeters[i_m*size_matrix[1]+i_m2*size_matrix[0]*size_matrix[2] + i_m3];
		   }     
		}
	//    printf("index: %i %i, %f",i_m,i_m3,distances_hor[i_m*size_matrix[2] + i_m3]);	
	    }
	}
  
}


void cal_euler_pingpong(float* distances_hor,float *horizontalAnglesMeasurements,int horizontalAmountOfMeasurements, float attitude_reference_pitch, float attitude_reference_roll, float dist_treshold){
   
	//init
	float sumPitch=0.0;
	float sumRoll=0.0;
  
	float oa_pitch_angle[horizontalAmountOfMeasurements];
	float oa_roll_angle[horizontalAmountOfMeasurements];

        for(int horizontal_index=0;horizontal_index<horizontalAmountOfMeasurements;horizontal_index++){
	  
	  //  printf("index: %i,distance %f",horizontal_index, distances_hor[horizontal_index]);
	    if(distances_hor[horizontal_index]<dist_treshold){
	              
	      
		      
		      oa_pitch_angle[horizontal_index] = cos(horizontalAnglesMeasurements[horizontal_index])*attitude_reference_pitch;
		      oa_roll_angle[horizontal_index] = -sin(horizontalAnglesMeasurements[horizontal_index])*attitude_reference_roll;
		      sumPitch+=oa_pitch_angle[horizontal_index];
		      sumRoll+=oa_roll_angle[horizontal_index];
			
	   }
	}

	if(sumPitch>attitude_reference_pitch){
		ref_pitch=attitude_reference_pitch;
	}
	else if(sumPitch<-attitude_reference_pitch){
		ref_pitch=-attitude_reference_pitch;
	}
	else{
		ref_pitch=sumPitch;
	}


	if(sumRoll>attitude_reference_roll){
		ref_roll=attitude_reference_roll;
	}
	else if(sumRoll<-attitude_reference_roll){
		ref_roll=-attitude_reference_roll;
	}
	else{
		ref_roll=sumRoll;
	}

	ref_pitch=DegOfRad(ref_pitch);
	ref_roll=DegOfRad(ref_roll);

	printf("DegOfRad data %f %f\n",ref_pitch,ref_roll);
	

}


 void nav_cal_vel_vector_pingpong(float *distancesMeters,float *anglesMeasurements,int lengthMeasurements,float* forward_speed,float *heading){

      int size_matrix[3] = {1, 6, 6};

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





 
