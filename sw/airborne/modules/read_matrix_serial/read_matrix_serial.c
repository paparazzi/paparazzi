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
#include "read_matrix_serial.h"
#include "math/pprz_geodetic_int.h"
#include "modules/read_matrix_serial/read_matrix_serial.h"
#include "modules/computer_vision/stabilization_practical.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/stereo_cam/stereocam.h"

int size_matrix[3] = {1, 6, 6};

float ref_pitch_angle = 0.2;
uint16_t matrix_sum[6]={0,0,0,0,0,0};
uint16_t matrix_sum2[6]={0,0,0,0,0,0};
uint16_t matrix_sum_treshold = 0;
float matrix_treshold = 7.0;

void serial_init(void) {

}

void serial_start(void)
{
	//printf("serial start\n");
}
void serial_update(void)
{

	float oa_pitch_angle[6]={0,0,0,0,0,0};
    float oa_roll_angle[6]={0,0,0,0,0,0};

	if(stereocam_data.fresh){
		// Do something with:
		stereocam_data.data;
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

//			     for(int i_fill2=0;i_fill2<size_matrix[2];i_fill2++){
//				  for(int i_fill3=0;i_fill3<size_matrix[1];i_fill3++){
//				      SendREADimageBuffer[i_fill2*size_matrix[1] + i_fill3] =  stereocam_data.data[messageArrayLocation*size_matrix[1]+i_fill2*size_matrix[0]*size_matrix[2] + i_fill3];
//				  }
//			      }




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
}





 
