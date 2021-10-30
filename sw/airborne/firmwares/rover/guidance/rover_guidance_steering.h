/*  rover_guidance_steering.h */

#ifndef ROVER_GUIDANCE_STEERING_H
#define ROVER_GUIDANCE_STEERING_H

/** Generated airframe.h from airframe.xml
 * - fun: SetActuatorsFromCommands
 * - var: commands
 * - var: hardware and construction parameters
 **/

#include "math/pprz_algebra_float.h"
#include "std.h"

#include "generated/airframe.h"

// Check servo name
#ifndef SERVO_MOTOR_THROTTLE
#error "Where is the servo MOTOR_THROTTLE?"
#endif

#ifndef SERVO_MOTOR_STEERING
#error "Where is the servo MOTOR_STEERING?"
#endif

// MIN_DELTA, MAX_DELTA: Min and max wheels turning angle
#ifndef MAX_DELTA 
#define MAX_DELTA 90
#endif

#ifndef MIN_DELTA 
#define MIN_DELTA -MAX_DELTA
#endif

// DRIVE_SHAFT_DISTANCE: Distance between front and rear wheels
#ifndef DRIVE_SHAFT_DISTANCE
#define DRIVE_SHAFT_DISTANCE 0.5
#warning "Construction variable DRIVE_SHAFT_DISTANCE for steer wheels rover not defined"
#endif

/** Steering rover guidance variables **/
typedef struct {
  float speedNorm;
  float speedDir;
  float delta;
  float omega;
} rover_ctrl;

extern rover_ctrl guidance_control;

extern void rover_guidance_steering_init(void);
extern void rover_guidance_steering_periodic(void);
extern bool rover_guidance_steering_set_delta(float delta);

// Bound delta
#define BoundDelta(delta) (delta <  MIN_DELTA ? MIN_DELTA : \
                          (delta >  MAX_DELTA ? MAX_DELTA : \
                           delta))

// Set steering command from delta
# define SetSteeringCmdFromDelta(delta){ \
  if (delta > 0) { \
    float dif = (SERVO_MOTOR_STEERING_MAX - SERVO_MOTOR_STEERING_NEUTRAL) * delta/MAX_DELTA; \
    commands[COMMAND_STEERING] = SERVO_MOTOR_STEERING_NEUTRAL + dif; \
  } else { \
    float dif = (SERVO_MOTOR_STEERING_NEUTRAL - SERVO_MOTOR_STEERING_MIN) * delta/MAX_DELTA;; \
    commands[COMMAND_STEERING] = SERVO_MOTOR_STEERING_NEUTRAL - dif; \
  } \
}

// Set AP throttle value
#define SetAPThrottleFromCommands(void) { \
    autopilot.throttle = MAX_PPRZ * (commands[COMMAND_SPEED] - SERVO_MOTOR_THROTTLE_NEUTRAL)/SERVO_MOTOR_THROTTLE_MAX; \
  }

#endif // ROVER_GUIDANCE_STEERING_H
