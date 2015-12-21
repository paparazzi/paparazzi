/*
 * Copyright (C) Kimberly McGuire
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/stereocam/stereocam2state/stereocam2state.h"
 * @author Kimberly McGuire
 * This module sends the data retreived from an external stereocamera modules, to the state filter of the drone. This is done so that the guidance modules can use that information for couadcopter
 */

#ifndef STEREOCAM2STATE_H
#define STEREOCAM2STATE_H

#include "state.h"

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"
#include <std.h>
#include "modules/stereocam/stereocam.h"

struct GpsStereoCam {
  struct EcefCoor_i ecef_vel;
};

extern struct GpsStereoCam gps_stereocam;

extern void stereo_to_state_init(void);
extern void stereo_to_state_periodic(void);


#endif

