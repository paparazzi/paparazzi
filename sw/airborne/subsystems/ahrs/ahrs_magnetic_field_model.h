#ifndef AHRS_MAGNETIC_FIELD_MODEL_H
#define AHRS_MAGNETIC_FIELD_MODEL_H

#include "generated/airframe.h"

// for complete INS filters, magnetic field can be defined with INS_H_[XYZ]
#if defined(INS_H_X) && defined(INS_H_Y) && defined(INS_H_Z)
#if defined(AHRS_H_X) || defined(AHRS_H_Y) || defined(AHRS_H_Z)
#warning Magnetic field model both defined by AHRS_H_[XYZ] and INS_H_[XYZ]
#else
#define AHRS_H_X INS_H_X
#define AHRS_H_Y INS_H_Y
#define AHRS_H_Z INS_H_Z
#endif
#endif

#if !USE_MAGNETOMETER && !defined(AHRS_H_X) && !defined(AHRS_H_Y)
#define AHRS_H_X 1
#define AHRS_H_Y 0
#define AHRS_H_Z 0
#endif

#endif
