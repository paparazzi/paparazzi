/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file firmwares/rover/guidance/rover_guidance.h
 *  Basic guidance for rover.
 *  Implement standard PID control loop to track a navigation target.
 *  Guidance "modes" are using the autopilot generation with the "guidance"
 *  state machine.
 */

#ifndef ROVER_GUIDANCE_H
#define ROVER_GUIDANCE_H


#include "math/pprz_algebra_float.h"
#include "generated/airframe.h"
#include "std.h"

struct RoverGuidanceSetpoint {
  /** position setpoint in NED.
   */
  struct FloatVect2 pos;    ///< position setpoint
  struct FloatVect2 speed;  ///< speed setpoint
  float heading;            ///< heading setpoint
  uint8_t mask;             ///< bit 5: vx & vy, bit 6: vz, bit 7: vyaw
};

// TODO write a generic PID somewhere else ?
struct RoverGuidancePID {
  float p;
  float i;
  float d;
  float err;
  float d_err;
  float sum_err;
};

struct RoverGuidanceControl {
  float motor_speed;
  float motor_turn;
};

struct RoverGuidance {
  struct RoverGuidanceSetpoint sp;    ///< setpoints
  struct RoverGuidanceControl cmd;    ///< commands
  struct RoverGuidancePID speed_pid;  ///< motor speed controller
  struct RoverGuidancePID turn_pid;   ///< turn rate controller
};

extern struct RoverGuidance rover_guidance;

extern void rover_guidance_init(void);
extern void rover_guidance_periodic(void); // call state machine
extern void rover_guidance_run(float *heading_sp); // run control loop
extern void rover_guidance_enter(void);

extern void rover_guidance_set_speed_igain(uint32_t igain);
extern void rover_guidance_set_turn_igain(uint32_t igain);

#ifdef GUIDANCE_MODE_GUIDED
/** Run GUIDED mode control
 */
extern void rover_guidance_guided_run(bool in_flight);

/** Set horizontal position setpoint in GUIDED mode.
 * @param x North position (local NED frame) in meters.
 * @param y East position (local NED frame) in meters.
 * @return TRUE if setpoints were set (currently in GUIDANCE_H_MODE_GUIDED)
 */
extern bool rover_guidance_set_guided_pos(float x, float y);

/** Set heading setpoint in GUIDED mode.
 * @param heading Setpoint in radians.
 * @return TRUE if setpoint was set (currently in GUIDANCE_H_MODE_GUIDED)
 */
extern bool rover_guidance_set_guided_heading(float heading);

/** Set body relative horizontal velocity setpoint in GUIDED mode.
 * @param vx forward velocity (body frame) in meters/sec.
 * @param vy right velocity (body frame) in meters/sec.
 * @return TRUE if setpoints were set (currently in GUIDANCE_H_MODE_GUIDED)
 */
extern bool rover_guidance_set_guided_body_vel(float vx, float vy);

/** Set horizontal velocity setpoint in GUIDED mode.
 * @param vx North velocity (local NED frame) in meters/sec.
 * @param vy East velocity (local NED frame) in meters/sec.
 * @return TRUE if setpoints were set (currently in GUIDANCE_H_MODE_GUIDED)
 */
extern bool rover_guidance_set_guided_vel(float vx, float vy);

/** Set heading rate setpoint in GUIDED mode.
 * @param rate Heading rate in radians.
 * @return TRUE if setpoints were set (currently in GUIDANCE_H_MODE_GUIDED)
 */
extern bool rover_guidance_set_guided_heading_rate(float rate);
#endif

//static inline void rover_guidance_SetMaxSpeed(float speed)
//{
//  gh_set_max_speed(speed);
//}
//
//static inline void rover_guidance_SetOmega(float omega)
//{
//  gh_set_omega(omega);
//}
//
//static inline void rover_guidance_SetZeta(float zeta)
//{
//  gh_set_zeta(zeta);
//}
//
//static inline void rover_guidance_SetTau(float tau)
//{
//  gh_set_tau(tau);
//}

#endif /* ROVER_GUIDANCE_H */
