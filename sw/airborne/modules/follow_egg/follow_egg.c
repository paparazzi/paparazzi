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
int knowsHeadingFresh=0;
float headingToFollow=0.0;
float follow_egg_pitch=0.0;
float follow_egg_roll=0.0;

float lastKnownVelX;
float lastKnownVelY;

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
void set_heading_following_egg() {
	float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  int newPosX = POS_BFP_OF_REAL(sin_heading * 1.5);
	  int newPosY = POS_BFP_OF_REAL(cos_heading * 1.5);
	  printf("Total vel: %f\n",(lastKnownVelX*lastKnownVelX+lastKnownVelY*lastKnownVelY));
		navigation_carrot.x=newPosX;
		navigation_carrot.y=newPosY;
		float newHeading =stateGetNedToBodyEulers_f()->psi+0.01;
		nav_set_heading_rad(newHeading);

		if(stereocam_data.fresh){
		printf("Set heading Egg follow");
		stereocam_data.fresh=0;

	    int far_away_threshold = 18;
	    int x = 5;
	    uint8_t valueClosest=0;
	    for(x=5; x < 120; x++){
	    	if(stereocam_data.data[x] < far_away_threshold &&stereocam_data.data[x]>0 ){
	    		valueClosest=stereocam_data.data[x];
	    		break;
	    	}
	    }
	    int indexRight;
	    int highValuesRightCount=0;
	    for(indexRight=40; indexRight < 120; indexRight++){
			if(stereocam_data.data[indexRight] > 60){
				highValuesRightCount++;
			}
		}
	    /*
	    if(highValuesRightCount>0){
	    	follow_egg_roll=-3.0;
	    }
	    else{
	    	follow_egg_roll=0.0;
	    }*/

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

		/*if(averageClose>50){
			follow_egg_pitch=4.0;
		}
		else{

			follow_egg_pitch=-1.0;
		}*/
	    follow_egg_pitch=-2.5;
	    uint8_t toSend=x;

	    float heading_change = (float) (x-55.0)*0.012; // convert pixel location to radians

	    follow_egg_roll=-heading_change;
	    DOWNLINK_SEND_FOLLOWEGG(DefaultChannel, DefaultDevice, &toSend, &heading_change,&highValuesRightCount,&averageClose);

		float newHeading =stateGetNedToBodyEulers_f()->psi+heading_change;
		knowsHeadingFresh=1;
		headingToFollow=newHeading;
		nav_set_heading_rad(headingToFollow);
	}
}
float getHeadingForFollowingEggRad(){
	printf("Get heading follow\n");
	return headingToFollow ;
}
int ackEgg=0;
int getKnowsHeadingEgg(){
	//return 3.14;
	if(knowsHeadingFresh && ackEgg){
		knowsHeadingFresh=0;
		ackEgg=0;
		return 1;
	}
	return 0;
}
void setAckEgg(){
	ackEgg=1;
}
