/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file modules/stereocam/stereocam_follow_me/follow_me.h
 * @author Roland
 * Follows a person using the reference given by the stereocam.
 * This module does so by changing the yaw angle and roll angle alternatively.
 * This way the drone does not drift away, and keeps looking at the person it tries to follow.
 */

#ifndef FOLLOW_ME_H
#define FOLLOW_ME_H

extern float ref_pitch;
extern float ref_roll;
extern float selfie_alt;

extern void follow_me(uint8_t headingToFollow, uint8_t heightObject, uint8_t distanceToObject);

#endif

