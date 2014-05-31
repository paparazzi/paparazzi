#ifndef AHRS_MAGNETIC_FIELD_MODEL_H
#define AHRS_MAGNETIC_FIELD_MODEL_H

#include "generated/airframe.h"

#if !USE_MAGNETOMETER && !defined(AHRS_H_X) && !defined(AHRS_H_Y)
#define AHRS_H_X 1
#define AHRS_H_Y 0
#define AHRS_H_Z 0
#endif

#endif
