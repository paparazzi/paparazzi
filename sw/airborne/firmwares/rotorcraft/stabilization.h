/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

/** @file firmwares/rotorcraft/stabilization.h
 *  General stabilization interface for rotorcrafts.
 */

#ifndef STABILIZATION_H
#define STABILIZATION_H

#include "std.h"

#include "generated/airframe.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"

/** Stabilization modes
 */
#define STABILIZATION_MODE_NONE     0
#define STABILIZATION_MODE_DIRECT   1
#define STABILIZATION_MODE_RATE     2
#define STABILIZATION_MODE_ATTITUDE 3

/** Stabilization sub-modes for attitude
 */
#define STABILIZATION_ATT_SUBMODE_HEADING   0 // direct heading control
#define STABILIZATION_ATT_SUBMODE_CARE_FREE 1 // care free heading mode
#define STABILIZATION_ATT_SUBMODE_FORWARD   2 // forward flight for hybrid-like

/** Stabilization setpoint.
 *  Struture to store the desired attitude with different
 *  frames and representations
 */
struct StabilizationSetpoint {
  enum {
    STAB_SP_QUAT,     ///< LTP to Body orientation in unit quaternion
    STAB_SP_EULERS,   ///< LTP to Body orientation in euler angles
    STAB_SP_LTP,      ///< banking and heading in LTP (NED) frame
    STAB_SP_RATES,     ///< body rates
    STAB_SP_QUAT_FF_RATE  ///< LTP to Body orientation in unit quaternion with precomputed feedforward rates
  } type;
  enum {
    STAB_SP_INT,
    STAB_SP_FLOAT
  } format;
  union {
    struct Int32Quat quat_i;
    struct FloatQuat quat_f;
    struct Int32Eulers eulers_i;
    struct FloatEulers eulers_f;
    struct { struct Int32Vect2 vect; int32_t heading; } ltp_i;
    struct { struct FloatVect2 vect; float heading; } ltp_f;
  } sp;
  union {
    struct Int32Rates rates_i;
    struct FloatRates rates_f;
  } r_sp;
};

/** Thrust setpoint // TODO to a setpoint header
 *  Structure to store the desired thrust vector with different format
 */
struct ThrustSetpoint {
  enum {
    THRUST_SP,        ///< absolute thrust setpoint
    THRUST_INCR_SP    ///< thrust increment
  } type;
  enum {
    THRUST_SP_INT,    ///< int is assumed to be normalized in [0:MAX_PPRZ]
    THRUST_SP_FLOAT   ///< float is assumed to be normalized in [0.:1.]
  } format;
  union {
    int32_t thrust_i[3];
    float thrust_f[3];
    int32_t th_incr_i[3];
    float th_incr_f[3];
  } sp;
};


/** Stabilization structure
 */
struct Stabilization {
  uint8_t mode;                       ///< current mode
  uint8_t att_submode;                ///< current attitude sub-mode
  struct AttitudeRCInput rc_in;       ///< RC input
  struct StabilizationSetpoint rc_sp; ///< Keep it ? FIXME
  struct StabilizationSetpoint sp;    ///< current attitude setpoint (store for messages)
  int32_t cmd[COMMANDS_NB];           ///< output command vector, range from [-MAX_PPRZ:MAX_PPRZ] (store for messages)
  float transition_ratio;             ///< transition percentage for hybrids (0.: hover; 1.: forward)
};

extern struct Stabilization stabilization;

/** Init function
 */
extern void stabilization_init(void);

/** Check mode change
 */
extern void stabilization_mode_changed(uint8_t new_mode, uint8_t submode);

/** Call default stabilization control
 * @param[in] in_flight true if rotorcraft is flying
 * @param[in] sp pointer to the stabilization setpoint, computed in guidance or from RC
 * @param[in] thrust pointer to thrust setpoint computed by vertical guidance
 * @param[out] cmd pointer to the output command vector
 */
extern void stabilization_run(bool in_flight, struct StabilizationSetpoint *sp, struct ThrustSetpoint *thrust, int32_t *cmd);

/** Get stabilization setpoint for failsafe
 */
extern struct StabilizationSetpoint stabilization_get_failsafe_sp(void);

/** Command filter for vibrating airframes
 */
extern void stabilization_filter_commands(void);

// helper convert functions
extern struct Int32Quat stab_sp_to_quat_i(struct StabilizationSetpoint *sp);
extern struct FloatQuat stab_sp_to_quat_f(struct StabilizationSetpoint *sp);
extern struct Int32Eulers stab_sp_to_eulers_i(struct StabilizationSetpoint *sp);
extern struct FloatEulers stab_sp_to_eulers_f(struct StabilizationSetpoint *sp);
extern struct Int32Rates stab_sp_to_rates_i(struct StabilizationSetpoint *sp);
extern struct FloatRates stab_sp_to_rates_f(struct StabilizationSetpoint *sp);
// thrust setpoint helper functions
// - first param is the thrust setpoint structure
// - second param is the current thrust (expected in [0:MAX_PPRZ])
extern int32_t th_sp_to_thrust_i(struct ThrustSetpoint *th, int32_t thrust, uint8_t axis);
extern float th_sp_to_thrust_f(struct ThrustSetpoint *th, int32_t thrust, uint8_t axis);
extern int32_t th_sp_to_incr_i(struct ThrustSetpoint *th, int32_t thrust, uint8_t axis);
extern float th_sp_to_incr_f(struct ThrustSetpoint *th, int32_t thrust, uint8_t axis);

// helper make functions
extern struct StabilizationSetpoint stab_sp_from_quat_i(struct Int32Quat *quat);
extern struct StabilizationSetpoint stab_sp_from_quat_f(struct FloatQuat *quat);
extern struct StabilizationSetpoint stab_sp_from_quat_ff_rates_f(struct FloatQuat *quat, struct FloatRates *rates);
extern struct StabilizationSetpoint stab_sp_from_eulers_i(struct Int32Eulers *eulers);
extern struct StabilizationSetpoint stab_sp_from_eulers_f(struct FloatEulers *eulers);
extern struct StabilizationSetpoint stab_sp_from_ltp_i(struct Int32Vect2 *vect, int32_t heading);
extern struct StabilizationSetpoint stab_sp_from_ltp_f(struct FloatVect2 *vect, float heading);
extern struct StabilizationSetpoint stab_sp_from_rates_i(struct Int32Rates *rates);
extern struct StabilizationSetpoint stab_sp_from_rates_f(struct FloatRates *rates);
extern struct ThrustSetpoint th_sp_from_thrust_i(int32_t thrust, uint8_t axis);
extern struct ThrustSetpoint th_sp_from_thrust_f(float thrust, uint8_t axis);
extern struct ThrustSetpoint th_sp_from_incr_i(int32_t th_increment, uint8_t axis);
extern struct ThrustSetpoint th_sp_from_incr_f(float th_increment, uint8_t axis);
extern struct ThrustSetpoint th_sp_from_thrust_vect_i(int32_t thrust[3]);
extern struct ThrustSetpoint th_sp_from_thrust_vect_f(float thrust[3]);
extern struct ThrustSetpoint th_sp_from_incr_vect_i(int32_t th_increment[3]);
extern struct ThrustSetpoint th_sp_from_incr_vect_f(float th_increment[3]);

#define THRUST_AXIS_X 0
#define THRUST_AXIS_Y 1
#define THRUST_AXIS_Z 2

#define STAB_SP_SET_EULERS_ZERO(_sp) { \
  _sp.type = STAB_SP_EULERS;  \
  _sp.format = STAB_SP_INT;   \
  _sp.sp.eulers_i.phi = 0;    \
  _sp.sp.eulers_i.theta = 0;  \
  _sp.sp.eulers_i.psi = 0;    \
}

#define THRUST_SP_SET_ZERO(_sp) { \
  _sp.type = THRUST_SP;       \
  _sp.format = THRUST_SP_INT; \
  _sp.sp.thrust_i[0] = 0;     \
  _sp.sp.thrust_i[1] = 0;     \
  _sp.sp.thrust_i[2] = 0;     \
}

#endif /* STABILIZATION_H */
