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

#include "modules/stereocam/stereocam_follow_me/follow_me.h"
#include "modules/stereocam/stereocam.h"
#include "state.h"
#include "navigation.h"

#include "subsystems/datalink/telemetry.h"

float ref_pitch = 0.0;
float ref_roll = 0.0;
float selfie_alt = 1.0;


void follow_me_init()
{

}


int breaksPoints = 0;
int isRollPhase = 0;
int isYawPhase = 0;
int phaseCounter = 0;

void changeRollYawPhase(int *phaseCounterArg, int *isRollPhaseArg, int *isYawPhaseArg);
void changeRollYawPhase(int *phaseCounterArg, int *isRollPhaseArg, int *isYawPhaseArg)
{
  (*phaseCounterArg)++;

  if (*isRollPhaseArg) {
    if (*phaseCounterArg > 15) {
      *phaseCounterArg = 0;
      *isRollPhaseArg = 0;
      *isYawPhaseArg = 1;
    }
  } else {
    if (*phaseCounterArg > 15) {
      *phaseCounterArg = 0;
      *isRollPhaseArg = 1;
      *isYawPhaseArg = 0;
    }
  }
}
uint8_t distanceToObject;
uint8_t heightObject;

void increase_nav_heading(int32_t *heading, int32_t increment);
void increase_nav_heading(int32_t *heading, int32_t increment)
{
  *heading = *heading + increment;
}
void follow_me_periodic()
{
  if (stereocam_data.fresh) {
    stereocam_data.fresh = 0;
    if (stereocam_data.data[0] == 0 || stereocam_data.data[1] == 85 || stereocam_data.data[2] == 0) {
      return;
    }
    changeRollYawPhase(&phaseCounter, &isRollPhase, &isYawPhase);
    uint8_t headingToFollow = stereocam_data.data[0];
    float heading_change = 0.0;
    int headingChangePerIt = 260;
    if (abs(headingToFollow - 65) > 10) {
      if (headingToFollow > 65) {
        heading_change = 0.25;
        if (isYawPhase) {
          increase_nav_heading(&nav_heading, headingChangePerIt);
        }
      } else if (headingToFollow < 65) {
        heading_change = -0.25;
        if (isYawPhase) {
          increase_nav_heading(&nav_heading, -1 * headingChangePerIt);
        }
      } else {
        heading_change = 0.0;
      }

    } else {
      heading_change = 0.0;
    }

    float turnFactor = 1.3;
    float currentHeading = stateGetNedToBodyEulers_f()->psi;

    ref_roll = 0.0;
    if (isRollPhase) {
      ref_roll = 30 * heading_change;
    }


    distanceToObject = stereocam_data.data[2];
    heightObject = stereocam_data.data[1];
    float heightGain = 3.0;
    if (nav_is_in_flight()) {
      if (heightObject > 50 && heightObject != 100) {
        selfie_alt -= heightGain * 0.01;
      } else if (heightObject < 20) {
        selfie_alt += heightGain * 0.01;
      }
    }/*
  if(selfie_alt-state.alt_agl_f>0.5){
    selfie_alt=state.alt_agl_f+0.5;
  }
  if(selfie_alt-state.alt_agl_f<-0.5){
      selfie_alt=state.alt_agl_f-0.5;
    }*/
    selfie_alt = 2.5;
    if (distanceToObject < 35) {
      ref_pitch = 13.0;
    } else if (distanceToObject < 60) {
      ref_pitch = 5.0;
    } else {
      ref_pitch = -2.0;
    }
  }
}
