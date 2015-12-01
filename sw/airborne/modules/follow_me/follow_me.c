/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/follow_me/follow_me.c"
 * @author Roland
 * follows a person on the stereo histogram image. It searches for the highest peak and adjusts its roll and pitch to hover at a nice distance.
 */

#include "modules/follow_me/follow_me.h"
#include "modules/stereo_cam/stereocam.h"
#include "state.h"
#include "navigation.h"

#include "subsystems/datalink/telemetry.h"

float selfie_ref_pitch = 0.0;
float ref_roll = 0.0;
float selfie_alt = 2.0;


void follow_me_init()
{

}


int breaksPoints=0;
int isRollPhase=0;
int isYawPhase=0;
int phaseCounter=0;
void changeRollYawPhase(int *phaseCounter,int *isRollPhase,int *isYawPhase){
	(*phaseCounter)++;

	if(*isRollPhase){
		if(*phaseCounter>15){
			*phaseCounter=0;
			*isRollPhase=0;
			*isYawPhase=1;
		}
	}
	else{
		if(*phaseCounter>5){
			*phaseCounter=0;
			*isRollPhase=1;
			*isYawPhase=0;
		}
	}
}
void follow_me_periodic()
{
  if (stereocam_data.fresh) {
    stereocam_data.fresh = 0;
    changeRollYawPhase(&phaseCounter,&isRollPhase,&isYawPhase);
    uint8_t headingToFollow = stereocam_data.data[0];
	float heading_change = 0.0;
	if(abs(headingToFollow - 65)>10){
		if(headingToFollow>65){
			heading_change=0.25;
		}
		else{
			heading_change=-0.25;
		}
	}
	if(isYawPhase){
//		float heading_change = (float)(headingToFollow - 65.0) * 0.002; // convert pixel location to radians
		if(abs(nav_heading - stateGetNedToBodyEulers_i()->psi)<150){
			float turnFactor=2.3;
			float currentHeading=stateGetNedToBodyEulers_f()->psi;
			float newHeading2 =currentHeading+turnFactor*heading_change;
			//printf("Heading now: %f new heading: %f\n",currentHeading,newHeading2);
			nav_set_heading_rad(newHeading2);
		}
		ref_roll=0.0;
	}
	else{
		ref_roll=32*heading_change;
	}


	uint8_t distanceToObject = stereocam_data.data[2];
	uint8_t heightObject = stereocam_data.data[1];

//	printf("Distance object %d heading to follow: %d height object: %d current heigt: %f\n",distanceToObject,headingToFollow,heightObject,selfie_alt);
	float heightGain = 3.0;
	if(heightObject>50 && heightObject !=100){
		selfie_alt-=heightGain*0.01;
	}
	else if(heightObject<20){
		selfie_alt+=heightGain*0.01;
	}
	if (distanceToObject < 55) {
		selfie_ref_pitch = 13.0;
	}
	else if(distanceToObject < 90){
		selfie_ref_pitch= 3.0;
	}
	else{
		selfie_ref_pitch=0.0;
	}

  }
}
