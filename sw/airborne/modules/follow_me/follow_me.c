/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/follow_me/follow_me.c"
 * @author Roland
 * follows based on stereo
 */

#include "modules/follow_me/follow_me.h"
#include "modules/stereo_cam/stereocam.h"

#include "subsystems/abi.h"

#include "guidance.h"
// Paparazzi Data
#include "state.h"

#include "subsystems/datalink/telemetry.h"
// Interact with navigation
#include "navigation.h"
#include "modules/computer_vision/opticflow_module.h"

int far_away_threshold = 28;
float lastVelocityReference=0.0;
float ref_pitch=0.0;
float ref_roll=0.0;
void setVelocityReference(float velocity){
	lastVelocityReference=velocity;
	float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));
	int newPosX = POS_BFP_OF_REAL(sin_heading * velocity);
	int newPosY = POS_BFP_OF_REAL(cos_heading * velocity);
	navigation_carrot.x=newPosX;
	navigation_carrot.y=newPosY;
}

void searchSpaceHeadingDrone(uint8_t* histogram,uint8_t *requiredHeading, uint8_t *valueThere){

    uint8_t x = 5;
    int maxFound=0;
    int width=5;
	 for(x=10; x < 110; x++){
		 int index=0;
		 int sumFound=0;
		 for(index=x; index < x+width; index++){
			 if(histogram[index]==120){
				 break;
			 }
			 sumFound+= histogram[index];

		 }
		 if(sumFound>maxFound){
			 maxFound=sumFound;
			 *valueThere=histogram[x+1];
			 *requiredHeading=x+1;
		 }
	}
}
void follow_me_init() {

}
void follow_me_periodic() {
	setVelocityReference(lastVelocityReference);

	if(stereocam_data.fresh){
		stereocam_data.fresh=0;

		uint8_t headingToFollow=0;
		uint8_t valueThere=0;
		 int indexRight;
			    int highValuesRightCount=0;
			    for(indexRight=0; indexRight < 120; indexRight++){
					if(stereocam_data.data[indexRight] > 60){
						highValuesRightCount++;
					}
				}

		searchSpaceHeadingDrone(stereocam_data.data,&headingToFollow,&valueThere);
		float heading_change = (float) (headingToFollow-65.0)*0.012; // convert pixel location to radians
		int differenceCenter = headingToFollow-65;
		if(differenceCenter > 0){
			ref_roll=0.1*abs(differenceCenter);
		}
		else{
			ref_roll=-0.1*abs(differenceCenter);
		}
		//float newHeading =stateGetNedToBodyEulers_f()->psi+heading_change;
		//nav_set_heading_rad(newHeading);
		printf("THE CURRENT VALUE AT THE PLACE I WANT TO LOOK AT IS %d with rotation %f %d\n",valueThere,heading_change,headingToFollow);
		if(fabs(heading_change) < 0.42){

			if(valueThere<40){
				setVelocityReference(1.1);
				ref_pitch=-3.0;
			}
			else{
				ref_pitch=3.0;
				setVelocityReference(-0.5);

			}

		}
		else{
			setVelocityReference(0.0);
			ref_pitch=0.0;
		}
		if(highValuesRightCount>20){
			ref_pitch=5.0;
		}
	}
}




/*
 *
int searchSpaceHeadingDrone(uint8_t* histogram){

    int x = 5;
    int closeGoodCount=0;
	 for(x=5; x < 120; x++){
		if(histogram[x] < far_away_threshold && histogram[x]>0 ){
			closeGoodCount++;
			if(closeGoodCount > 3){
				return x;
			}
		}
		else{
			closeGoodCount=0;
		}
	}
	return 120;
}

void set_heading_following_egg() {
	setVelocityReference(lastVelocityReference);

	if(stereocam_data.fresh){
		stereocam_data.fresh=0;

	    int headingToFollow = searchSpaceHeadingDrone(stereocam_data.data);


	    int indexRight;
	    int highValuesRightCount=0;
	    for(indexRight=40; indexRight < 120; indexRight++){
			if(stereocam_data.data[indexRight] > 60){
				highValuesRightCount++;
			}
		}

	    int indexAll=0;
	    int totalSum=0;
	    int totalSumCount=0;
	    for(indexAll=0; indexAll < 120; indexAll++){
	    	if(stereocam_data.data[indexAll]>5 && stereocam_data.data[indexAll] < 110){
	    		totalSum+=stereocam_data.data[indexAll];
	    		totalSumCount++;
	    	}
		}
	    float averageClose=0.0;
	    if(totalSumCount>0){
	      averageClose=(totalSum/totalSumCount);
	    }

	    float heading_change = (float) (headingToFollow-55.0)*0.012; // convert pixel location to radians
	    DOWNLINK_SEND_FOLLOWEGG(DefaultChannel, DefaultDevice, &headingToFollow, &heading_change,&highValuesRightCount,&averageClose);

		float newHeading =stateGetNedToBodyEulers_f()->psi+heading_change;
		nav_set_heading_rad(newHeading);
		if(fabs(heading_change) < 0.22){
			setVelocityReference(1.1);
			printf("Dont turn %f , let's go!\n",heading_change);
		}
		else{
			setVelocityReference(-0.2);

			printf("Turn, wait pleaze!\n");
		}
	}
}*/

