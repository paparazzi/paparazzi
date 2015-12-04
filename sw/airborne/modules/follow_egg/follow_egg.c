/*
 * Copyright (C) Roland and Sjoerd
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/follow_egg/follow_egg.c"
 * @author Roland and Sjoerd
 * follows the egg next to the mavlab
 */

#include "modules/follow_egg/follow_egg.h"
#include "modules/stereo_cam/stereocam.h"

#include "subsystems/abi.h"
#ifndef VISION_VELOCITY_ESTIMATE_ID
#define VISION_VELOCITY_ESTIMATE_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(VISION_VELOCITY_ESTIMATE_ID)
static abi_event velocity_est_ev;

#include "guidance.h"
// Paparazzi Data
#include "state.h"

#include "subsystems/datalink/telemetry.h"
// Interact with navigation
#include "navigation.h"
#include "modules/computer_vision/opticflow_module.h"

float lastKnownVelX;
float lastKnownVelY;
int far_away_threshold = 28;
static void new_velocity_estimate(uint8_t sender_id __attribute__((unused)),
    uint32_t stamp, float vel_x, float vel_y, float vel_z, float noise)
{
	lastKnownVelX= vel_x;
	lastKnownVelY= vel_y;
}
void follow_egg_init() {
	printf("Egg follow init");
	 AbiBindMsgVELOCITY_ESTIMATE(VISION_VELOCITY_ESTIMATE_ID, &velocity_est_ev, new_velocity_estimate);

}
float lastVelocityReference=0.0;

void setVelocityReference(float velocity){
	lastVelocityReference=velocity;
	float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));
	int newPosX = POS_BFP_OF_REAL(sin_heading * velocity);
	int newPosY = POS_BFP_OF_REAL(cos_heading * velocity);
	navigation_carrot.x=newPosX;
	navigation_carrot.y=newPosY;
}

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
}

