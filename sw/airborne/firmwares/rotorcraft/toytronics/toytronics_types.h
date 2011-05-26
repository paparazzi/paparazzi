// toytronics_types.h
// Greg Horn, Joby Robotics 2011

#ifndef __TOYTRONICS_TYPES_H__
#define __TOYTRONICS_TYPES_H__

#include "firmwares/rotorcraft/toytronics/mathlib/misc_math.h"
#include "firmwares/rotorcraft/toytronics/mathlib/filters.h"
#include "firmwares/rotorcraft/toytronics/mathlib/quat.h"
#include "firmwares/rotorcraft/toytronics/mathlib/xyz.h"
#include "firmwares/rotorcraft/toytronics/mathlib/spatial_rotations.h"

typedef struct setpoint_t {
  quat_t q_n2sp;
  quat_t q_b2sp;
  quat_t q_pitch_yaw_setpoint;
  quat_t q_pitch_yaw_estimated;
  double setpoint_heading;
  double estimated_heading;
} setpoint_t;

typedef struct rc_t {
  double throttle;
  double pitch;
  double roll;
  double yaw;
  /* uint8_t gear; */
  /* uint8_t enable; */
  /* uint8_t aux2; */
  /* uint8_t mode; */
} rc_t;

#endif // __TOYTRONICS_TYPES_H__
