/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/stereocam/stereocam_follow_me/stereocam_follow_me.h"
 * @author Roland
 * follows a person using the reference given by the stereocam.
 */

#ifndef FOLLOW_ME_H
#define FOLLOW_ME_H
extern float ref_pitch;
extern float ref_roll;
extern float selfie_alt;
extern void follow_me_periodic(void);

#endif

