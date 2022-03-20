/*  rover_guidance_steering.h */

#ifndef ROVER_GUIDANCE_STEERING_H
#define ROVER_GUIDANCE_STEERING_H

/** Generated airframe.h from airframe.xml
 * - fun: SetActuatorsFromCommands
 * - var: commands
 * - var: hardware and construction parameters
 **/

#include "std.h"
#include <math.h>

#include "generated/airframe.h"

// Check critical global definitiones
#ifndef SERVO_MOTOR_THROTTLE
#error "Steering rover firmware requires the servo MOTOR_THROTTLE"
#endif

#ifndef SERVO_MOTOR_STEERING
#error "Steering rover firmware requires the servo MOTOR_STEERING"
#endif

#ifndef COMMAND_THROTTLE
#error "Steering rover firmware requires the command COMMAND_THROTTLE"
#endif

#ifndef COMMAND_STEERING
#error "Steering rover firmware requires the command COMMAND_STEERING"
#endif


/** Global variables definitions **/

// MIN_DELTA, MAX_DELTA: Min and max wheels turning angle (deg)
// You should measure this angle if you want to have an
// efficient control in your steering
#ifndef MAX_DELTA 
#define MAX_DELTA 15.0
#endif
#ifndef MIN_DELTA 
#define MIN_DELTA MAX_DELTA
#endif

// This variables allow you to configurate de max and min
// steering actuator signal. There is a mecanic limitation
// for the actuator in the steering of our rover, so we have
// to limit the actuator travel.
#ifndef MAX_CMD_SHUT
#define MAX_CMD_SHUT 0
#endif
#ifndef MIN_CMD_SHUT 
#define MIN_CMD_SHUT 0
#endif

// MIN_SPEED, MAX_SPEED: Min and max state speed (m/s)
#ifndef MAX_SPEED 
#define MAX_SPEED 999.0 //We don't really use that variable
#endif
#ifndef MIN_SPEED 
#define MIN_SPEED 0.2 //But this one is mandatory because we have
#endif                //to deal with GPS noise (and 1/v in guidance control).

// DRIVE_SHAFT_DISTANCE: Distance between front and rear wheels (m)
#ifndef DRIVE_SHAFT_DISTANCE
#define DRIVE_SHAFT_DISTANCE 0.25
#warning "Construction variable DRIVE_SHAFT_DISTANCE for steering wheel rover not defined"
#endif

// SR_MEASURED_KF: Lineal feed forward control constant (have to be measured in new servos)
#ifndef SR_MEASURED_KF
#define SR_MEASURED_KF 10
#warning "Construction constant SR_MEASURED_KF for steering wheel rover not defined"
#endif


/** Steering rover guidance STRUCTURES **/
// High level commands
typedef struct {
  float speed;
  float delta;
} sr_cmd_t;

// Main structure
typedef struct {
  sr_cmd_t cmd;
  float state_speed;
  float gvf_omega;
  float throttle;

  float speed_error;
  float kf;
  float kp;
  float ki;
} rover_ctrl;

extern rover_ctrl guidance_control;

/** Steering rover guidance EXT FUNCTIONS **/
extern void rover_guidance_steering_init(void);
extern void rover_guidance_steering_periodic(void);
extern bool rover_guidance_steering_set_delta(float delta);


/** MACROS **/
// Bound delta
#define BoundDelta(delta) (delta < -MIN_DELTA ? -MIN_DELTA : \
                          (delta >  MAX_DELTA ?  MAX_DELTA : \
                           delta));

// Bound speed
#define BoundSpeed(speed) (speed <  MIN_SPEED ? MIN_SPEED : \
                          (speed >  MAX_SPEED ? MAX_SPEED : \
                           speed));

// Bound throttle
#define BoundThrottle(throttle) TRIM_PPRZ((int)throttle);

// Set low level commands from high level commands
#define GetCmdFromDelta(delta) (delta >= 0 ? -delta/MAX_DELTA * (MAX_PPRZ - (int)MAX_CMD_SHUT) : \
                                             -delta/MIN_DELTA * (MAX_PPRZ - (int)MIN_CMD_SHUT));

// This macro is for NAV state
#define GetCmdFromThrottle(throttle) TRIM_PPRZ((int)throttle);

// Set AP throttle value
#define SetAPThrottleFromCommands(void) { \
    autopilot.throttle = commands[COMMAND_THROTTLE]; \
  }

#endif // ROVER_GUIDANCE_STEERING_H
