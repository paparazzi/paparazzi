/*
 * Copyright (C) Roland and Sjoerd
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/follow_egg/follow_egg.h"
 * @author Roland and Sjoerd
 * follows the egg next to the mavlab
 */

#ifndef FOLLOW_EGG_H
#define FOLLOW_EGG_H
float follow_egg_roll;

float follow_egg_pitch;
extern void follow_egg_init(void);
extern void set_heading_following_egg(void);
extern float getHeadingForFollowingEggRad(void);
extern int getKnowsHeadingEgg(void);
extern void setAckEgg(void);
#endif

