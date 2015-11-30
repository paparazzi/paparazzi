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

#include "modules/stereocam/follow_me/follow_me.h"
#include "modules/stereocam/stereocam.h"

int far_away_threshold = 28;
float ref_pitch = 0.0;
float ref_roll = 0.0;

void searchSpaceHeadingDrone(uint8_t *histogram, uint8_t *requiredHeading, uint8_t *valueThere)
{
  uint8_t x = 5;
  int maxFound = 0;
  int width = 5;
  for (x = 10; x < 110; x++) {
    int index = 0;
    int sumFound = 0;
    for (index = x; index < x + width; index++) {
      if (histogram[index] == 120) {
        break;
      }
      sumFound += histogram[index];

    }
    if (sumFound > maxFound) {
      maxFound = sumFound;
      *valueThere = histogram[x + 1];
      *requiredHeading = x + 1;
    }
  }
}
void follow_me_init()
{

}
void follow_me_periodic()
{
  if (stereocam_data.fresh) {
    stereocam_data.fresh = 0;

    uint8_t headingToFollow = 0;
    uint8_t valueThere = 0;
    int indexRight;
    int highValuesRightCount = 0;
    for (indexRight = 0; indexRight < 120; indexRight++) {
      if (stereocam_data.data[indexRight] > 60) {
        highValuesRightCount++;
      }
    }

    searchSpaceHeadingDrone(stereocam_data.data, &headingToFollow, &valueThere);
    float heading_change = (float)(headingToFollow - 65.0) * 0.012; // convert pixel location to radians
    int differenceCenter = headingToFollow - 65;
    if (differenceCenter > 0) {
      ref_roll = 0.1 * abs(differenceCenter);
    } else {
      ref_roll = -0.1 * abs(differenceCenter);
    }
    if (fabs(heading_change) < 0.42) {

      if (valueThere < 40) {
        ref_pitch = -3.0;
      } else {
        ref_pitch = 3.0;
      }

    } else {
      ref_pitch = 0.0;
    }
    if (highValuesRightCount > 20) {
      ref_pitch = 5.0;
    }
  }
}
