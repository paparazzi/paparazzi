/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file modules/stereocam/stereocam_follow_me/follow_me.c
 * @author Roland
 * Follows a person using the reference given by the stereocam.
 * This module does so by changing the yaw angle and roll angle alternatively.
 * This way the drone does not drift away, and keeps looking at the person it tries to follow.
 */

#include "modules/stereocam/stereocam_follow_me/follow_me.h"

#include "state.h"
#include "navigation.h"
#include "modules/datalink/telemetry.h"
#include "generated/flight_plan.h"

#ifndef STEREOCAM_FOLLOW_ME_USE_OPTITRACK
#define STEREOCAM_FOLLOW_ME_USE_OPTITRACK FALSE
#endif

#define HEADING_CHANGE_PER_MEASUREMENT 0.063f
#define CENTER_IMAGE_HOR 65
#define MAXIMUM_ALTITUDE_FOLLOWING 3.0
#define MINIMUM_ALTITUDE_FOLLOWING 1.0

float ref_pitch = 0.0;
float ref_roll = 0.0;
float selfie_alt = 1.0;

static void changeRollYawPhase(int *phaseCounterArg, int *isRollPhaseArg, int *isYawPhaseArg)
{
  static const int amountOfRollPhaseTime = 15;
  static const int amountOfYawPhaseTime = 15;

  (*phaseCounterArg)++;

  if (*isRollPhaseArg) {
    if (*phaseCounterArg > amountOfRollPhaseTime) {
      *phaseCounterArg = 0;
      *isRollPhaseArg = 0;
      *isYawPhaseArg = 1;
    }
  } else {
    if (*phaseCounterArg > amountOfYawPhaseTime) {
      *phaseCounterArg = 0;
      *isRollPhaseArg = 1;
      *isYawPhaseArg = 0;
    }
  }
}

static void increase_nav_heading(float *heading, float increment)
{
  *heading = *heading + increment;
}

void follow_me(uint8_t headingToFollow, uint8_t heightObject, uint8_t distanceToObject)
{
  static int phaseCounter=0, isRollPhase=0, isYawPhase=0;
  static float heightGain=0.3;

  // If we don't use GPS we alternate a phase where we roll and where we yaw.
  // This way we don't drift sideways AND we don't lose the user out of our sight
  changeRollYawPhase(&phaseCounter, &isRollPhase, &isYawPhase);

  // Change our heading if the user is starting to get out of sight.
  float heading_change = 0.0;
  float headingChangePerIt = HEADING_CHANGE_PER_MEASUREMENT;
  if (abs(headingToFollow - CENTER_IMAGE_HOR) > 10) {
    if (headingToFollow > CENTER_IMAGE_HOR) {
      heading_change = 0.25;
      if (isYawPhase || STEREOCAM_FOLLOW_ME_USE_OPTITRACK) {
        increase_nav_heading(&nav.heading, headingChangePerIt);
      }
    } else if (headingToFollow < CENTER_IMAGE_HOR) {
      heading_change = -0.25;
      if (isYawPhase || STEREOCAM_FOLLOW_ME_USE_OPTITRACK) {
        increase_nav_heading(&nav.heading, -1 * headingChangePerIt);
      }
    } else {
      heading_change = 0.0;
    }

  } else {
    heading_change = 0.0;
  }

  // If we have our roll phase we take the value of the change we need to have in heading and use it to go sideways
  ref_roll = 0.0;
  if (isRollPhase) {
    ref_roll = 30 * heading_change;
  }


  // If we are in flight we want to adjust our height based on the place where we see our object
  if (nav_is_in_flight()) {
    if (heightObject > 50) {
      selfie_alt -= heightGain;
    } else if (heightObject < 20) {
      selfie_alt += heightGain;
    }
  }

  // Bound the altitude to normal values
  if (selfie_alt > MAXIMUM_ALTITUDE_FOLLOWING) {
    selfie_alt = MAXIMUM_ALTITUDE_FOLLOWING;
  }
  if (selfie_alt < MINIMUM_ALTITUDE_FOLLOWING) {
    selfie_alt = MINIMUM_ALTITUDE_FOLLOWING;
  }

  // If using GPS we set the location of the waypoint to our desired altitude
#if STEREOCAM_FOLLOW_ME_USE_OPTITRACK
  waypoint_set_alt(WP_STDBY, selfie_alt);
#endif

  // Set a pitch if the person we follow is too close
  if (distanceToObject < 35) {
    ref_pitch = 13.0;
  } else if (distanceToObject < 60) {
    ref_pitch = 5.0;
  } else {
    ref_pitch = -2.0;
  }
}
