/*
 * Copyright (C) Roland and Sjoerd
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/follow_egg/follow_egg.c"
 * @author Roland and Sjoerd
 * follows the egg the MAV lab is housed in. It follows a corridor while assuming that the corridor always goes left.
 */

#include "modules/stereocam/follow_egg/follow_egg.h"
#include "modules/stereocam/stereocam.h"
#include "navigation.h"

void follow_egg_init()
{

}
float lastVelocityReference = 0.0;
void setVelocityReference(float velocity)
{
  lastVelocityReference = velocity;
  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));
  int newPosX = POS_BFP_OF_REAL(sin_heading * velocity);
  int newPosY = POS_BFP_OF_REAL(cos_heading * velocity);
  navigation_carrot.x = newPosX;
  navigation_carrot.y = newPosY;
}

int searchSpaceHeadingDrone(uint8_t *histogram)
{

  int x = 5;
  int closeGoodCount = 0;
  for (x = 5; x < 120; x++) {
    if (histogram[x] < far_away_threshold && histogram[x] > 0) {
      closeGoodCount++;
      if (closeGoodCount > 3) {
        return x;
      }
    } else {
      closeGoodCount = 0;
    }
  }
  return 120;
}

void set_heading_following_egg()
{
  setVelocityReference(lastVelocityReference);

  if (stereocam_data.fresh) {
    stereocam_data.fresh = 0;
    int headingToFollow = searchSpaceHeadingDrone(stereocam_data.data);
    float heading_change = (float)(headingToFollow - 55.0) * 0.012; // convert pixel location to radians
    float newHeading = stateGetNedToBodyEulers_f()->psi + heading_change;
    nav_set_heading_rad(newHeading);
    if (fabs(heading_change) < 0.22) {
      setVelocityReference(1.1);
    } else {
      setVelocityReference(-0.1);
    }
  }
}

