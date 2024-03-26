/*
 * Copyright (C) 2012 Felix Ruess <felix.ruess@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file stabilization_attitude_rc_setpoint.h
 *  Read an attitude setpoint from the RC.
 */

#ifndef STABILIZATION_ATTITUDE_RC_SETPOINT_H
#define STABILIZATION_ATTITUDE_RC_SETPOINT_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "modules/radio_control/radio_control.h"

/** Attitude (and Rate) Remote Control input
 */
struct AttitudeRCInput {
  struct FloatQuat    rc_quat;    ///< RC input in quaternion
  struct FloatEulers  rc_eulers;  ///< RC input in eulers (needed even for quat for yaw integration)
  float care_free_heading;        ///< care_free heading
  float transition_theta_offset;  ///< pitch offset for hybrids, add when in forward mode
  float last_ts;                  //< last timestamp (in seconds)
};

/** Init rc input
 * @param[out] rc_sp pointer to rc input structure
 */
extern void stabilization_attitude_rc_setpoint_init(struct AttitudeRCInput *rc_sp);

/** Read attitude setpoint from RC as quaternion
 * Interprets the stick positions as axes.
 * Both eulers and quaternion format are updated.
 * @param[out] rc_sp             pointer to rc input structure
 * @param[in]  in_flight         true if in flight
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  rc                pointer to radio control structure
 */
extern void stabilization_attitude_read_rc_setpoint(struct AttitudeRCInput *rc_sp, bool in_flight,
    bool in_carefree, bool coordinated_turn, struct RadioControl *rc);

/** Read attitude setpoint from RC as quaternion in earth bound frame
 * Interprets the stick positions as axes.
 * Both eulers and quaternion format are updated.
 * @param[out] rc_sp             pointer to rc input structure
 * @param[in]  in_flight         true if in flight
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  rc                pointer to radio control structure
 */
extern void stabilization_attitude_read_rc_setpoint_earth_bound(struct AttitudeRCInput *rc_sp, bool in_flight,
    bool in_carefree, bool coordinated_turn, struct RadioControl *rc);

/** Read attitude setpoint from RC as euler angles
 * Only the euler format is updated and returned
 * @param[out] rc_sp             pointer to rc input structure
 * @param[in]  in_flight         true if in flight
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  rc                pointer to radio control structure
 * @return attitude setpoint in eulers (int)
 */
extern struct Int32Eulers stabilization_attitude_read_rc_setpoint_eulers(struct AttitudeRCInput *rc_sp, bool in_flight,
    bool in_carefree, bool coordinated_turn, struct RadioControl *rc);

/** Read attitude setpoint from RC as float euler angles
 * Only the euler format is updated and returned
 * @param[out] rc_sp             pointer to rc input structure
 * @param[in]  in_flight         true if in flight
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  rc                pointer to radio control structure
 * @return attitude setpoint in eulers (float)
 */
extern struct FloatEulers stabilization_attitude_read_rc_setpoint_eulers_f(struct AttitudeRCInput *rc_sp, bool in_flight,
    bool in_carefree, bool coordinated_turn, struct RadioControl *rc);

/** Reset rc input to current state
 * @param[in/out] rc_sp   pointer to rc input structure
 */
extern void stabilization_attitude_reset_rc_setpoint(struct AttitudeRCInput *rc_sp);

/** Reset care free heading to current heading
 * @param[in/out] rc_sp   pointer to rc input structure
 */
extern void stabilization_attitude_reset_care_free_heading(struct AttitudeRCInput *rc_sp);

/** Read RC roll and pitch only
 * @param[out] q quaternion representing the RC roll/pitch input
 * @param[in] rc pointer to radio control structure
 */
extern void stabilization_attitude_read_rc_roll_pitch_quat_f(struct FloatQuat *q, struct RadioControl *rc);

/** Read RC roll and pitch only, in earth bounded frame
 * @param[out] q quaternion representing the RC roll/pitch input
 * @param[in] theta_offset pitch offset for forward flight
 * @param[in] rc pointer to radio control structure
 */
extern void stabilization_attitude_read_rc_roll_pitch_earth_quat_f(struct FloatQuat *q, float theta_offset, struct RadioControl *rc);

/** Get attitude heading as int (avoiding jumps)
 */
extern int32_t stabilization_attitude_get_heading_i(void);

/** Get attitude heading as float (avoiding jumps)
 */
extern float stabilization_attitude_get_heading_f(void);

#endif /* STABILIZATION_ATTITUDE_RC_SETPOINT_H */

