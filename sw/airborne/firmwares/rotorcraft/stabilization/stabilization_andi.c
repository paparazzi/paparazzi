/*
 *
 * Copyright (C) 2025 Justin Dubois <j.p.g.dubois@student.tudelft.nl>
 *
 * This file is part of paparazzi
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
 *
 */

/**
 * @file sw/airborne/firmwares/rotorcraft/stabilization/stabilization_andi.c
 * @brief ANDI stabilization controller for tiltbody rotorcraft.
 *
 * Implements Adaptive Nonlinear Dynamic Inversion (ANDI) for attitude and thrust control
 * with on-board model compensation.
 *
 * Assumptions:
 * - Airframe is a tiltbody with 2 elevons (indices 0,1) and 2 motors (indices 2,3) in tractor configuration.
 * - ESC neutral value equals zero thrust (usually 0).
 * - Motors controlled in squared RPM to linearize thrust curve.
 *
 * FIXME: WLS allocation normalization depends on control effectiveness scaling (state-dependent).
 * Consider normalizing costs by max achievable derivatives per actuator/output.
 *
 * @author Justin Dubois <j.p.g.dubois@student.tudelft.nl>
 */


#include "firmwares/rotorcraft/stabilization/stabilization_andi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "math/pprz_algebra_float.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
#include "math/wls/wls_alloc.h"
#include "modules/nav/nav_rotorcraft_hybrid.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/rotwing_drone/rotwing_state.h"
#include "modules/core/commands.h"
#include "autopilot.h"
#include "filters/low_pass_filter.h"
#include "filters/low_pass_filter_types.h"
#include "filters/complementary_filter.h"
#include "filters/complementary_filter_types.h"
#include "filters/transport_delay.h"
#include "filters/transport_delay_types.h"
#include "filters/low_pass_zoh_filter.h"
#include "filters/low_pass_zoh_filter_types.h"

#include <stdio.h>
#if INS_EXT_POSE
  #include "modules/ins/ins_ext_pose.h"
#endif

#ifdef STABILIZATION_ANDI_SCHEDULE_EFF
  const bool SCHEDULE_EFF = STABILIZATION_ANDI_SCHEDULE_EFF;
#else
  const bool SCHEDULE_EFF = false;
#endif

#ifdef STABILIZATION_ANDI_USE_STATE_DYNAMICS
  const bool USE_STATE_DYNAMICS = STABILIZATION_ANDI_USE_STATE_DYNAMICS;
#else
  const bool USE_STATE_DYNAMICS = false;
#endif

#ifdef STABILIZATION_ANDI_RELAX_OBM
  const float ANDI_RELAX_OBM = STABILIZATION_ANDI_RELAX_OBM;
#else
  const float ANDI_RELAX_OBM = 1.0f;
#endif

#ifdef STABILIZATION_ANDI_ACT_IS_SERVO
const bool ACTUATOR_IS_SERVO[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_IS_SERVO;
#else
const bool ACTUATOR_IS_SERVO[ANDI_NUM_ACT] = {0};
#endif

#ifdef STABILIZATION_ANDI_ACT_DYNAMICS
  const float ACTUATOR_DYNAMICS[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_DYNAMICS; // rad/s
#else
  #error "You must specify the actuator dynamics"
#endif

#ifdef STABILIZATION_ANDI_ACT_DELAY
const uint8_t ACTUATOR_DELAY[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_DELAY;
#else
const uint8_t ACTUATOR_DELAY[ANDI_NUM_ACT] = {0};
#endif

#if defined(STABILIZATION_ANDI_ACT_MAX) && defined(STABILIZATION_ANDI_ACT_MIN)
  const float ACTUATOR_MAX[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_MAX;
  const float ACTUATOR_MIN[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_MIN;
#else
  #error "You must specify the actuator limits: STABILIZATION_ANDI_ACT_MAX and STABILIZATION_ANDI_ACT_MIN"
#endif

#if defined(STABILIZATION_ANDI_ACT_RATE_MAX) && defined(STABILIZATION_ANDI_ACT_RATE_MIN)
  const float ACTUATOR_D_MAX[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_RATE_MAX;
  const float ACTUATOR_D_MIN[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_RATE_MIN;
#else
  #error "You must specify the actuator limits: STABILIZATION_ANDI_ACT_RATE_MAX and STABILIZATION_ANDI_ACT_RATE_MIN"
#endif

#if defined STABILIZATION_ANDI_ACT_PREF
const float ACTUATOR_PREF[ANDI_NUM_ACT] = STABILIZATION_ANDI_ACT_PREF;
#else
const float ACTUATOR_PREF[ANDI_NUM_ACT] = {0.0f};
#endif

#if defined STABILIZATION_ANDI_RC_RATE_MAX
const float RC_RATE_MAX[ANDI_OUTPUTS] = STABILIZATION_ANDI_RC_RATE_MAX;
#else
const float RC_RATE_MAX[ANDI_OUTPUTS] = {[0 ... ANDI_OUTPUTS - 1] = 5.0f};
#endif

#ifdef STABILIZATION_ANDI_THRUST_MIN
  const float THRUST_MIN = STABILIZATION_ANDI_THRUST_MIN;
#else
  const float THRUST_MIN = 0.0f;
#endif

#ifdef STABILIZATION_ANDI_THRUST_MAX
  const float THRUST_MAX = STABILIZATION_ANDI_THRUST_MAX;
#else
  #error "You must specify maximum specific thrust: STABILIZATION_ANDI_THRUST_MAX"
#endif

#ifdef PERIODIC_FREQUENCY
  const float SAMPLE_TIME = 1.0f / PERIODIC_FREQUENCY;
#else
  #error "Periodic frequency is not defined."
#endif

#if ANDI_NUM_ACT > WLS_N_U_MAX
  #error Matrix-WLS_N_U_MAX too small or not defined: define WLS_N_U_MAX >= ANDI_NUM_ACT in airframe file
#endif
#if ANDI_OUTPUTS > WLS_N_V_MAX
  #error Matrix-WLS_N_V_MAX too small or not defined: define WLS_N_V_MAX >= ANDI_OUTPUTS in airframe file
#endif

#ifdef STABILIZATION_ANDI_WLS_WV
const float WLS_WV[ANDI_OUTPUTS] = STABILIZATION_ANDI_WLS_WV;
#else
const float WLS_WV[ANDI_OUTPUTS] = {[0 ... ANDI_OUTPUTS - 1] = 1.0f};
#endif

/**
 * Normalized actuator cost for WLS allocation.
 * Each value corresponds to the relative cost of using each actuator, normalized over
 * the possible range of u_dot for that actuator.
 * FIXME: This normalization is not ideal or constant, as it depends on the scaling of the control.
 */
#ifdef STABILIZATION_ANDI_WLS_WU
float WLS_WU[ANDI_NUM_ACT] = STABILIZATION_ANDI_WLS_WU;
#else
float WLS_WU[ANDI_NUM_ACT] = {[0 ... ANDI_NUM_ACT - 1] = 1.0f};
#endif

#ifdef STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA
const struct FloatVect3 CUTOFF_FREQ_OMEGA = {
  .x = STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA,
  .y = STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA,
  .z = STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA
}; // rad/s
#else
#error "You must specify the cutoff frequency for omega: STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA"
#endif

#ifdef STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA_DOT
const struct FloatVect3 CUTOFF_FREQ_OMEGA_DOT = {
  .x = STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA_DOT,
  .y = STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA_DOT,
  .z = STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA_DOT
}; // rad/s
const float CUTOFF_FREQ_ACTUATOR[ANDI_NUM_ACT] = {[0 ... ANDI_NUM_ACT - 1] = STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA_DOT}; // rad/s
const float CUTOFF_FREQ_THRUST = STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA_DOT; // rad/s
#else
#error "You must specify the cutoff frequency for omega_dot: STABILIZATION_ANDI_CUTOFF_FREQ_OMEGA_DOT"
#endif

#ifdef STABILIZATION_ANDI_CUTOFF_FREQ_ACCEL
const struct FloatVect3 CUTOFF_FREQ_ACCEL = {
  .x = STABILIZATION_ANDI_CUTOFF_FREQ_ACCEL,
  .y = STABILIZATION_ANDI_CUTOFF_FREQ_ACCEL,
  .z = STABILIZATION_ANDI_CUTOFF_FREQ_ACCEL
};
#else
#error "You must specify the cutoff frequency for accel: STABILIZATION_ANDI_CUTOFF_FREQ_ACCEL"
#endif

#ifdef STABILIZATION_ANDI_CUTOFF_FREQ_VEL
const struct FloatVect3 CUTOFF_FREQ_VEL = {
  .x = STABILIZATION_ANDI_CUTOFF_FREQ_VEL,
  .y = STABILIZATION_ANDI_CUTOFF_FREQ_VEL,
  .z = STABILIZATION_ANDI_CUTOFF_FREQ_VEL
}; // rad/s
#else
#error "You must specify the cutoff frequency for vel: STABILIZATION_ANDI_CUTOFF_FREQ_VEL"
#endif


// Function prototypes
static void actuators_t4_in_callback(uint8_t sender_id, struct ActuatorsT4In *actuators_t4_in_ptr,
                                     float *actuators_t4_extra_data_in_ptr);
static void fetch_actuators_t4(float actuator_meas[ANDI_NUM_ACT], const struct ActuatorsT4In *actuators_t4_in_ptr);
static struct GainsOrder3Vect3 compute_reference_gains_order_3_vect_3(const struct PolesOrder3Vect3 *poles);
static struct GainsOrder2Vect3 compute_reference_gains_order_2_vect_3(const struct PolesOrder2Vect3 *poles);
static struct GainsOrder3Vect3 compute_error_gains_order_3_vect_3(const struct PolesOrder3Vect3 *poles);
static struct GainsOrder2Vect3 compute_error_gains_order_2_vect_3(const struct PolesOrder2Vect3 *poles);
static void generate_reference_rate(float dt, const struct FloatRates *rate_des,
                                    const struct GainsOrder2Vect3 *k_rate_rm, const struct AttQuat *bounds,
                                    struct AttQuat *att_ref) __attribute__((unused));
static void generate_reference_attitude(float dt, const struct FloatQuat *att_des,
                                        const struct GainsOrder3Vect3 *k_att_rm, const struct AttQuat *bounds, struct AttQuat *att_ref);
static void generate_reference_thrust(float dt, float thrust_des, const float k_thrust_rm,
                                      const struct ThrustRef *bounds_min, const struct ThrustRef *bounds_max, struct ThrustRef *thrust_ref);
static struct FloatVect3 control_error_rate(const struct AttQuat *att_ref, const struct AttStateQuat *att_state,
    const struct GainsOrder2Vect3 *k_rate_ec) __attribute__((unused));
static struct FloatVect3 control_error_attitude(const struct AttQuat *att_ref, const struct AttStateQuat *att_state,
    const struct GainsOrder3Vect3 *k_att_ec);
static float control_error_thrust(const struct ThrustRef *thrust_ref, const float thrust_state,
                                  const float k_thrust_ec);
static void compute_wls_upper_bounds(float u_d_max[ANDI_NUM_ACT], const float act_state[ANDI_NUM_ACT],
                                     const float act_max[ANDI_NUM_ACT], const float act_rate_max[ANDI_NUM_ACT],
                                     const float actuator_bandwidth[ANDI_NUM_ACT]);
static void compute_wls_lower_bounds(float u_d_min[ANDI_NUM_ACT], const float act_state[ANDI_NUM_ACT],
                                     const float act_min[ANDI_NUM_ACT], const float act_rate_min[ANDI_NUM_ACT],
                                     const float actuator_bandwidth[ANDI_NUM_ACT]);
static void compute_wls_u_scaler(float u_scaler[ANDI_NUM_ACT], const float act_min[ANDI_NUM_ACT],
                                 const float act_max[ANDI_NUM_ACT]);
static void compute_wls_v_scaler(float v_scaler[ANDI_NUM_ACT], const float v[ANDI_NUM_ACT]) __attribute__((unused));

static inline float ec_k1_order3_f(const float omega_n, const float zeta, const float omega_a) { return (omega_n * omega_n * (omega_a - 2 * zeta * omega_n)); }
static inline float ec_k2_order3_f(const float omega_n, const float zeta, const float omega_a) { return (omega_n * omega_n + 2.0f * zeta * omega_n * (omega_a - 2 * zeta * omega_n)); }
static inline float ec_k3_order3_f(const float omega_n __attribute__((unused)),
                                   const float zeta __attribute__((unused)), const float omega_a) { return omega_a; }
static inline float rm_k1_order3_f(const float omega_n, const float zeta, const float omega_a) { return (omega_n * omega_n * omega_a) / (omega_n * omega_n + 2 * zeta * omega_n * omega_a); }
static inline float rm_k2_order3_f(const float omega_n, const float zeta, const float omega_a) { return (omega_n * omega_n + 2 * zeta * omega_n * omega_a) / (2 * zeta * omega_n + omega_a); }
static inline float rm_k3_order3_f(const float omega_n, const float zeta, const float omega_a) { return 2 * zeta * omega_n + omega_a; }

static inline float ec_k1_order2_f(const float omega_a, const float zeta) { return omega_a * omega_a / (4 * zeta * zeta); }
static inline float ec_k2_order2_f(const float omega_a, const float zeta __attribute__((unused))) { return omega_a; }
static inline float rm_k1_order2_f(const float omega_a, const float zeta) { return ec_k2_order2_f(omega_a, zeta) / ec_k2_order2_f(omega_a, zeta); }
static inline float rm_k2_order2_f(const float omega_a, const float zeta) { return ec_k2_order2_f(omega_a, zeta); }

struct PolesOrder2Vect3 andi_p_rate_ec = {
  .omega_a = {
    .x = STABILIZATION_ANDI_POLE_RATE_EC_OMEGA_A_X,
    .y = STABILIZATION_ANDI_POLE_RATE_EC_OMEGA_A_Y,
    .z = STABILIZATION_ANDI_POLE_RATE_EC_OMEGA_A_Z
  },
  .zeta = {
    .x = STABILIZATION_ANDI_POLE_RATE_EC_ZETA_X,
    .y = STABILIZATION_ANDI_POLE_RATE_EC_ZETA_Y,
    .z = STABILIZATION_ANDI_POLE_RATE_EC_ZETA_Z
  }
};
struct PolesOrder2Vect3 andi_p_rate_rm = {
  .omega_a = {
    .x = STABILIZATION_ANDI_POLE_RATE_RM_OMEGA_A_X,
    .y = STABILIZATION_ANDI_POLE_RATE_RM_OMEGA_A_Y,
    .z = STABILIZATION_ANDI_POLE_RATE_RM_OMEGA_A_Z
  },
  .zeta = {
    .x = STABILIZATION_ANDI_POLE_RATE_RM_ZETA_X,
    .y = STABILIZATION_ANDI_POLE_RATE_RM_ZETA_Y,
    .z = STABILIZATION_ANDI_POLE_RATE_RM_ZETA_Z
  }
};
struct PolesOrder3Vect3 andi_p_att_ec = {
  .omega_n = {
    .x = STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_N_X,
    .y = STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_N_Y,
    .z = STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_N_Z
  },
  .zeta = {
    .x = STABILIZATION_ANDI_POLE_ATT_EC_ZETA_X,
    .y = STABILIZATION_ANDI_POLE_ATT_EC_ZETA_Y,
    .z = STABILIZATION_ANDI_POLE_ATT_EC_ZETA_Z
  },
  .omega_a = {
    .x = STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_A_X,
    .y = STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_A_Y,
    .z = STABILIZATION_ANDI_POLE_ATT_EC_OMEGA_A_Z
  }
};
struct PolesOrder3Vect3 andi_p_att_rm = {
  .omega_n = {
    .x = STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_N_X,
    .y = STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_N_Y,
    .z = STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_N_Z
  },
  .zeta = {
    .x = STABILIZATION_ANDI_POLE_ATT_RM_ZETA_X,
    .y = STABILIZATION_ANDI_POLE_ATT_RM_ZETA_Y,
    .z = STABILIZATION_ANDI_POLE_ATT_RM_ZETA_Z
  },
  .omega_a = {
    .x = STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_A_X,
    .y = STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_A_Y,
    .z = STABILIZATION_ANDI_POLE_ATT_RM_OMEGA_A_Z
  }
};
float andi_p_thrust_ec = STABILIZATION_ANDI_POLE_THRUST_EC;
float andi_p_thrust_rm = STABILIZATION_ANDI_POLE_THRUST_RM;


// WLS allocation variables
struct WLS_t wls_stab_p = {
  .nu = ANDI_NUM_ACT,
  .nv = ANDI_OUTPUTS,
  .gamma_sq = 100000.0,
  .u_pref = {0.0f},
  .u_min = {0.0f},
  .u_max = {0.0f},
  .PC = 0.0f,
  .SC = 0.0f,
  .iter = 0
};

// Controller gains
struct GainsOrder2Vect3 andi_k_rate_ec;
struct GainsOrder2Vect3 andi_k_rate_rm;
struct GainsOrder3Vect3 andi_k_att_ec;
struct GainsOrder3Vect3 andi_k_att_rm;
float andi_k_thrust_ec;
float andi_k_thrust_rm;

// Complementary filter instances for state dependent contribution
struct FirstOrderComplementaryVect3 attitude_rates_cf;
struct Butterworth2ComplementaryVect3 attitude_accel_cf;
struct FloatRates angular_rates_obm;

struct FirstOrderComplementaryVect3 linear_vel_cf;
struct Butterworth2ComplementaryVect3 linear_accel_cf;
struct FloatVect3 linear_velocity_obm;

// Actuator and delay
struct FirstOrderZOHLowPass actuator_obm_zohlpf[ANDI_NUM_ACT]; // ZOH low pass filter for actuator model
struct TransportDelay actuator_obm_delay[ANDI_NUM_ACT]; // transport delay for actuator model
float actuator_state[ANDI_NUM_ACT]; // from feedback or model
float actuator_state_lpf[ANDI_NUM_ACT]; // delayed actuator state

// T4 Actuator feedback handling
struct ActuatorsT4In actuators_t4_obs;
abi_event actuators_t4_in_event;
float actuator_meas[ANDI_NUM_ACT]; // measured actuator feedback

// Raw state measurement variables
struct FloatRates rates_prev;

// State variables
struct AttStateQuat attitude_state_cf;  // undelayed attitude state for feedforward
struct LinState linear_state_cf;        // undelayed linear state for feedforward
float thrust_state;                     // delayed thrust state for feedback

// Reference model variables
struct AttQuat attitude_ref;
struct ThrustRef thrust_ref;

// Setpoints
struct FloatRates rates_des;
struct FloatQuat attitude_des;
float thrust_des;

// Bounds
struct AttQuat attitude_bounds;
struct ThrustRef thrust_bounds_min;
struct ThrustRef thrust_bounds_max;

// Controller variables
float ce_mat[ANDI_NUM_ACT * ANDI_OUTPUTS];
float du_min[ANDI_NUM_ACT];
float du_max[ANDI_NUM_ACT];
float du_cmd[ANDI_NUM_ACT];
float u_cmd[ANDI_NUM_ACT];

// Pseudo command variables
float nu_obj[ANDI_OUTPUTS];           // Total pseudo command allocated to the actuators
float nu_ec[ANDI_OUTPUTS];            // Pseudo command from the error controller
float nu_obm[ANDI_OUTPUTS];           // Pseudo command from the on board model state dependent term
// float nu_reconstructed[ANDI_OUTPUTS]; // Reconstructed angular acceleration from the actuator commands (for model verification)

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_wls_v_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  char *name = "andi";
  pprz_msg_send_WLS_V(trans, dev, AC_ID,
                      strlen(name), name,
                      &wls_stab_p.gamma_sq, // Does this need scaling as a function of scaling factor?
                      (uint8_t *)&wls_stab_p.iter,
                      ANDI_OUTPUTS, nu_obj,
                      ANDI_OUTPUTS, (float *)WLS_WV);
}

static void send_wls_u_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  char *name = "andi";
  float zero_array[ANDI_NUM_ACT] = {0.0f};
  pprz_msg_send_WLS_U(trans, dev, AC_ID,
                      strlen(name), name,
                      ANDI_NUM_ACT, (float *)WLS_WU,
                      ANDI_NUM_ACT, zero_array,
                      ANDI_NUM_ACT, du_min,
                      ANDI_NUM_ACT, du_max,
                      ANDI_NUM_ACT, du_cmd);
}

static void send_eff_mat_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  float zero = 0.0f;
  pprz_msg_send_EFF_MAT_STAB(trans, dev, AC_ID,
                             ANDI_NUM_ACT, &ce_mat[0 * ANDI_NUM_ACT],
                             ANDI_NUM_ACT, &ce_mat[1 * ANDI_NUM_ACT],
                             ANDI_NUM_ACT, &ce_mat[2 * ANDI_NUM_ACT],
                             ANDI_NUM_ACT, &ce_mat[3 * ANDI_NUM_ACT],
                             1,  &zero);
}

static void send_stab_attitude_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_ATTITUDE(trans, dev, AC_ID,
                              4, (float *)&attitude_des,
                              4, (float *)&attitude_state_cf.att,
                              4, (float *)&attitude_ref.att,
                              3, (float *)&attitude_state_cf.att_d,
                              3, (float *)&attitude_ref.att_d,
                              3, (float *)&attitude_state_cf.att_2d,
                              3, (float *)&attitude_ref.att_2d,
                              3, (float *)&attitude_ref.att_3d,
                              ANDI_OUTPUTS, actuator_state);
}
// For debug only
// static void send_stab_thrust_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
// {
//   pprz_msg_send_STAB_THRUST(trans, dev, AC_ID,
//                             &thrust_des,
//                             &thrust_ref.thrust,
//                             &thrust_state,
//                             &thrust_ref.thrust_d);
// }

// static void send_on_board_model_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
// {
//   pprz_msg_send_ON_BOARD_MODEL(trans, dev, AC_ID,
//                                3, (float *)&linear_state_cf.vel,
//                                3, (float *)&linear_state_cf.acc,
//                                ANDI_NUM_ACT, (float *)&actuator_meas,
//                                ANDI_OUTPUTS, (float *)&nu_obm);
// }

// static void send_stab_pseudo_command_stabilization_andi(struct transport_tx *trans, struct link_device *dev)
// {
//   /**
//    * nu_obj: Total pseudo command sent to the actuators
//    * nu_ec: Pseudo command from the error controller
//    * nu_obm: Pseudo command from the on board model state dependent term
//    * nu_reconstructed: Reconstructed angular acceleration from the actuator commands.
//    * This is used to verify the on board model. If the OBM is perfect, then nu_reconstructed should be
//    * equal to the measured state plus any external disturbances.
//    *
//    * nu_obj = nu_ec + nu_obm
//    * nu_reconstructed = Ce * u_meas - nu_obm NOTE: This is wrong equation, fix it.
//    *
//    * FIXME: nu_reconstructed has been repurposed and no longer is the reconstructed value. Update the telemetry message accordingly.
//    */
//   pprz_msg_send_STAB_PSEUDO_COMMAND(trans, dev, AC_ID,
//                                     ANDI_OUTPUTS, nu_obj, // Total pseudo command
//                                     ANDI_OUTPUTS, nu_ec, // From error controller
//                                     ANDI_OUTPUTS, nu_obm, // From on board model state dependent term
//                                     ANDI_OUTPUTS, nu_reconstructed); // Reconstructed from actuator commands
// }

#endif // PERIODIC_TELEMETRY

static void actuators_t4_in_callback(uint8_t sender_id __attribute__((unused)),
                                     struct ActuatorsT4In *actuators_t4_in_ptr, float *actuators_t4_extra_data_in_ptr __attribute__((unused)));

/**
 * @brief Callback function to handle incoming actuator telemetry data.
 *
 * This function copies the data from the input struct pointer `actuators_t4_in_ptr`
 * into a local static instance `actuator_obs` for further processing or monitoring.
 *
 * @param sender_id Identifier of the sender of the telemetry data.
 * @param actuators_t4_in_ptr Pointer to the incoming ActuatorsT4In struct containing
 *                            ESC telemetry, servo angles, loads, and other actuator info.
 * @param actuators_t4_extra_data_in_ptr Pointer to additional extra actuator data (unused).
 */
static void actuators_t4_in_callback(uint8_t sender_id __attribute__((unused)),
                                     struct ActuatorsT4In *actuators_t4_in_ptr, float *actuators_t4_extra_data_in_ptr __attribute__((unused)))
{
  // Copy the entire ActuatorsT4In struct from the pointer to actuator_obs
  memcpy(&actuators_t4_obs, actuators_t4_in_ptr, sizeof(struct ActuatorsT4In));
}

/**
 * @brief Extract actuator states from ActuatorsT4In struct.
 *
 * This function converts selected actuator telemetry data from the
 * input struct into floating-point values representing angles in radians
 * and rotational speeds in radians per second, storing them in the
 * provided actuator_state array.
 *
 * FIXME: Avoid hardcoding indices, conversion factors, and inversions; This requires additional functionality in actuators_t4.
 * FIXME: Put actuator feedback / state management in a separate module.
 * FIXME: All ESC type actuators will be squared rpm for thrust linearization.
 *
 * @param[out] actuator_state Array of floats with size ANDI_NUM_ACT where the
 *                           converted actuator states will be stored. The caller
 *                           must allocate this.
 * @param[in] actuators_t4_in_ptr Pointer to the input struct containing actuator telemetry,
 *                                including servo angles (in 1e-2 degrees) and ESC RPM.
 */
static void fetch_actuators_t4(float actuator_meas[ANDI_NUM_ACT], const struct ActuatorsT4In *actuators_t4_in_ptr)
{
  actuator_meas[0] = RadOfCentiDeg((float)actuators_t4_in_ptr->servo_1_angle);
  actuator_meas[1] = -RadOfCentiDeg((float)actuators_t4_in_ptr->servo_6_angle);
  actuator_meas[2] = (float)actuators_t4_in_ptr->esc_1_rpm * 2 * M_PI / 60;      // Convert rpm to rad/s
  actuator_meas[2] *= actuator_meas[2];                                          // square motor rpm
  actuator_meas[3] = (float)actuators_t4_in_ptr->esc_2_rpm * 2 * M_PI / 60;
  actuator_meas[3] *= actuator_meas[3]; // square motor rpm
}

/**
 * Compute reference-model gains for a 3rd-order 3D system.
 * Each axis gain (x, y, z) is computed using rm_k*_order3_f() with omega_n, zeta, and p1 parameters.
 * @param[in] poles Pointer to PolesOrder3Vect3 containing omega_n, zeta, and p1 for each axis.
 * @return Struct containing k1, k2, k3 gains for x, y, z.
 */
static struct GainsOrder3Vect3 compute_reference_gains_order_3_vect_3(const struct PolesOrder3Vect3 *poles)
{
  struct GainsOrder3Vect3 gains;
  gains.k1.x = rm_k1_order3_f(poles->omega_n.x, poles->zeta.x, poles->omega_a.x);
  gains.k1.y = rm_k1_order3_f(poles->omega_n.y, poles->zeta.y, poles->omega_a.y);
  gains.k1.z = rm_k1_order3_f(poles->omega_n.z, poles->zeta.z, poles->omega_a.z);

  gains.k2.x = rm_k2_order3_f(poles->omega_n.x, poles->zeta.x, poles->omega_a.x);
  gains.k2.y = rm_k2_order3_f(poles->omega_n.y, poles->zeta.y, poles->omega_a.y);
  gains.k2.z = rm_k2_order3_f(poles->omega_n.z, poles->zeta.z, poles->omega_a.z);

  gains.k3.x = rm_k3_order3_f(poles->omega_n.x, poles->zeta.x, poles->omega_a.x);
  gains.k3.y = rm_k3_order3_f(poles->omega_n.y, poles->zeta.y, poles->omega_a.y);
  gains.k3.z = rm_k3_order3_f(poles->omega_n.z, poles->zeta.z, poles->omega_a.z);
  return gains;
}

/**
 * Compute reference-model gains for a 2nd-order 3D system.
 * Each axis gain (x, y, z) is computed using rm_k*_order2_f() with omega_n and zeta parameters.
 * @param[in] poles Pointer to PolesOrder2Vect3 containing omega_n and zeta for each axis.
 * @return Struct containing k1, k2 gains for x, y, z.
 */
static struct GainsOrder2Vect3 compute_reference_gains_order_2_vect_3(const struct PolesOrder2Vect3 *poles)
{
  struct GainsOrder2Vect3 gains;
  gains.k1.x = rm_k1_order2_f(poles->omega_a.x, poles->zeta.x);
  gains.k1.y = rm_k1_order2_f(poles->omega_a.y, poles->zeta.y);
  gains.k1.z = rm_k1_order2_f(poles->omega_a.z, poles->zeta.z);

  gains.k2.x = rm_k2_order2_f(poles->omega_a.x, poles->zeta.x);
  gains.k2.y = rm_k2_order2_f(poles->omega_a.y, poles->zeta.y);
  gains.k2.z = rm_k2_order2_f(poles->omega_a.z, poles->zeta.z);
  return gains;
}

/**
 * Compute error-compensation gains for a 3rd-order 3D system.
 * Each axis gain (x, y, z) is computed using ec_k*_order3_f() with omega_n, zeta, and p1 parameters.
 * @param[in] poles Pointer to PolesOrder3Vect3 containing omega_n, zeta, and p1 for each axis.
 * @return Struct containing k1, k2, k3 gains for x, y, z.
 */
static struct GainsOrder3Vect3 compute_error_gains_order_3_vect_3(const struct PolesOrder3Vect3 *poles)
{
  struct GainsOrder3Vect3 gains;
  gains.k1.x = ec_k1_order3_f(poles->omega_n.x, poles->zeta.x, poles->omega_a.x);
  gains.k1.y = ec_k1_order3_f(poles->omega_n.y, poles->zeta.y, poles->omega_a.y);
  gains.k1.z = ec_k1_order3_f(poles->omega_n.z, poles->zeta.z, poles->omega_a.z);

  gains.k2.x = ec_k2_order3_f(poles->omega_n.x, poles->zeta.x, poles->omega_a.x);
  gains.k2.y = ec_k2_order3_f(poles->omega_n.y, poles->zeta.y, poles->omega_a.y);
  gains.k2.z = ec_k2_order3_f(poles->omega_n.z, poles->zeta.z, poles->omega_a.z);

  gains.k3.x = ec_k3_order3_f(poles->omega_n.x, poles->zeta.x, poles->omega_a.x);
  gains.k3.y = ec_k3_order3_f(poles->omega_n.y, poles->zeta.y, poles->omega_a.y);
  gains.k3.z = ec_k3_order3_f(poles->omega_n.z, poles->zeta.z, poles->omega_a.z);
  return gains;
}

/**
 * Compute error-compensation gains for a 2nd-order 3D system.
 * Each axis gain (x, y, z) is computed using ec_k*_order2_f() with omega_n and zeta parameters.
 * @param[in] poles Pointer to PolesOrder2Vect3 containing omega_n and zeta for each axis.
 * @return Struct containing k1, k2 gains for x, y, z.
 */
static struct GainsOrder2Vect3 compute_error_gains_order_2_vect_3(const struct PolesOrder2Vect3 *poles)
{
  struct GainsOrder2Vect3 gains;
  gains.k1.x = ec_k1_order2_f(poles->omega_a.x, poles->zeta.x);
  gains.k1.y = ec_k1_order2_f(poles->omega_a.y, poles->zeta.y);
  gains.k1.z = ec_k1_order2_f(poles->omega_a.z, poles->zeta.z);

  gains.k2.x = ec_k2_order2_f(poles->omega_a.x, poles->zeta.x);
  gains.k2.y = ec_k2_order2_f(poles->omega_a.y, poles->zeta.y);
  gains.k2.z = ec_k2_order2_f(poles->omega_a.z, poles->zeta.z);
  return gains;
}

/**
 * @brief Generates a bounded second-order reference signal for attitude rate control.
 *
 * This function computes smooth angular rate, acceleration, and jerk references based on the desired rates,
 * applying bounded limits to ensure stability and safe dynamic behavior. It updates the reference states
 * in place within the provided `att_ref` structure, which is a quaternion-based reference with associated rate states.
 *
 * @param[in] dt         The sampling interval in seconds.
 * @param[in] rate_des   Desired angular rates (p, q, r) as a `FloatRates` structure.
 * @param[in] k_rate_rm  Gain parameters as `GainsOrder2Vect3`, for the proportional and derivative terms.
 * @param[in] bounds     Limits for the desired rates, rates derivatives, and accelerations, in `AttRefEulers`.
 * @param[in,out] att_ref  The reference model states (attitude quaternion, rates, and derivatives),
 *                         updated in place to produce the reference command.
 */
static void generate_reference_rate(
  float dt,
  const struct FloatRates *rate_des,
  const struct GainsOrder2Vect3 *k_rate_rm,
  const struct AttQuat *bounds,
  struct AttQuat *att_ref)
{
  float p_des = rate_des->p;
  float q_des = rate_des->q;
  float r_des = rate_des->r;

  BoundAbs(p_des, bounds->att_d.p);
  BoundAbs(q_des, bounds->att_d.q);
  BoundAbs(r_des, bounds->att_d.r);

  float p_d_des = k_rate_rm->k1.x * (p_des - att_ref->att_d.p);
  float q_d_des = k_rate_rm->k1.y * (q_des - att_ref->att_d.q);
  float r_d_des = k_rate_rm->k1.z * (r_des - att_ref->att_d.r);

  BoundAbs(p_d_des, bounds->att_2d.x);
  BoundAbs(q_d_des, bounds->att_2d.y);
  BoundAbs(r_d_des, bounds->att_2d.z);

  float p_2d_des = k_rate_rm->k2.x * (p_d_des - att_ref->att_2d.x);
  float q_2d_des = k_rate_rm->k2.y * (q_d_des - att_ref->att_2d.y);
  float r_2d_des = k_rate_rm->k2.z * (r_d_des - att_ref->att_2d.z);

  BoundAbs(p_2d_des, bounds->att_3d.x);
  BoundAbs(q_2d_des, bounds->att_3d.y);
  BoundAbs(r_2d_des, bounds->att_3d.z);

  att_ref->att_3d.x = p_2d_des;
  att_ref->att_3d.y = q_2d_des;
  att_ref->att_3d.z = r_2d_des;

  att_ref->att_2d.x += p_2d_des * dt;
  att_ref->att_2d.y += q_2d_des * dt;
  att_ref->att_2d.z += r_2d_des * dt;

  att_ref->att_d.p += att_ref->att_2d.x * dt;
  att_ref->att_d.q += att_ref->att_2d.y * dt;
  att_ref->att_d.r += att_ref->att_2d.z * dt;

  float_quat_identity(&att_ref->att);
}

/**
 * @brief Generates a bounded second-order reference signal for quaternion-based attitude control.
 *
 * Computes smooth angular rate, acceleration, and jerk references using quaternion error feedback.
 * Bounds are applied to limit reference values and maintain system stability. The function updates
 * the reference states in place within the provided `att_ref` structure, which includes quaternion attitude
 * and associated rate states.
 *
 * @param[in] dt         Sampling interval in seconds.
 * @param[in] att_des    Desired attitude as a unit quaternion.
 * @param[in] k_att_rm   Gain parameters (proportional and derivative gains for rate control).
 * @param[in] bounds     Limits on rate references and derivatives (angular velocity and higher).
 * @param[in,out] att_ref Reference model state containing attitude quaternion and rate state, updated in place.
 *
 * FIXME: The bounds introduce nonlinearities that may destabilize the reference model for large changes in attitude.
 * Currently removed, consider revising the bounding strategy.
 * FIXME: Quaternion error calculation assumes small angle errors; may need to be revised for large attitude changes.
 */
static void generate_reference_attitude(
  float dt,
  const struct FloatQuat *att_des,
  const struct GainsOrder3Vect3 *k_att_rm,
  const struct AttQuat *bounds,
  struct AttQuat *att_ref)
{
  struct FloatQuat att_err;  // rotation from ref to des
  float_quat_inv_comp_norm_shortest(&att_err, &att_ref->att, att_des);
  float p_des = k_att_rm->k1.x * att_err.qx * 2;
  float q_des = k_att_rm->k1.y * att_err.qy * 2;
  float r_des = k_att_rm->k1.z * att_err.qz * 2;

  BoundAbs(p_des, bounds->att_d.p);
  BoundAbs(q_des, bounds->att_d.q);
  BoundAbs(r_des, bounds->att_d.r);

  float p_d_des = k_att_rm->k2.x * (p_des - att_ref->att_d.p);
  float q_d_des = k_att_rm->k2.y * (q_des - att_ref->att_d.q);
  float r_d_des = k_att_rm->k2.z * (r_des - att_ref->att_d.r);

  BoundAbs(p_d_des, bounds->att_2d.x);
  BoundAbs(q_d_des, bounds->att_2d.y);
  BoundAbs(r_d_des, bounds->att_2d.z);

  float p_2d_des = k_att_rm->k3.x * (p_d_des - att_ref->att_2d.x);
  float q_2d_des = k_att_rm->k3.y * (q_d_des - att_ref->att_2d.y);
  float r_2d_des = k_att_rm->k3.z * (r_d_des - att_ref->att_2d.z);

  BoundAbs(p_2d_des, bounds->att_3d.x);
  BoundAbs(q_2d_des, bounds->att_3d.y);
  BoundAbs(r_2d_des, bounds->att_3d.z);

  att_ref->att_3d.x = p_2d_des;
  att_ref->att_3d.y = q_2d_des;
  att_ref->att_3d.z = r_2d_des;

  att_ref->att_2d.x += p_2d_des * dt;
  att_ref->att_2d.y += q_2d_des * dt;
  att_ref->att_2d.z += r_2d_des * dt;

  att_ref->att_d.p += att_ref->att_2d.x * dt;
  att_ref->att_d.q += att_ref->att_2d.y * dt;
  att_ref->att_d.r += att_ref->att_2d.z * dt;

  float_quat_integrate(&att_ref->att, &att_ref->att_d, dt);
  float_quat_normalize(&att_ref->att);
}


/**
 * @brief Generates a bounded second-order reference signal for thrust control.
 *
 * Applies bounded limits to a desired thrust input and computes smoothed
 * thrust rate and integrated thrust references. The reference states are
 * updated in place inside the provided \p thrust_ref structure.
 *
 * @param[in] dt           Sampling interval in seconds.
 * @param[in] thrust_des   Desired thrust input value.
 * @param[in] k_thrust_rm  Gain parameter for thrust rate control (proportional gain).
 * @param[in] bounds       Maximum allowable thrust magnitude.
 * @param[in,out] thrust_ref  Pointer to ThrustRef struct holding thrust reference states.
 */
static void generate_reference_thrust(
  float dt,
  float thrust_des,
  const float k_thrust_rm,
  const struct ThrustRef *bounds_min,
  const struct ThrustRef *bounds_max,
  struct ThrustRef *thrust_ref)
{
  Bound(thrust_des, bounds_min->thrust, bounds_max->thrust);
  thrust_ref->thrust_d = k_thrust_rm * (thrust_des - thrust_ref->thrust);

  Bound(thrust_ref->thrust_d, bounds_min->thrust_d, bounds_max->thrust_d);
  thrust_ref->thrust += thrust_ref->thrust_d * dt;
}

/**
 * @brief Computes the control error command for attitude rate regulation.
 *
 * This function calculates the virtual control input vector \c nu required to reduce the error
 * between the reference and current angular rate states. It uses proportional and derivative gains
 * on the differences of rate and rate derivative components, respectively, and adds feedforward jerk.
 *
 * @param[in] att_ref   Pointer to the reference attitude and rate state structure.
 * @param[in] att_state Pointer to the current attitude and rate state structure.
 * @param[in] k_rate_ec  Pointer to gain parameters structure, containing proportional and derivative gains.
 *
 * @return A \c FloatVect3 structure representing the computed virtual control input vector.
 */
static struct FloatVect3 control_error_rate(
  const struct AttQuat *att_ref,
  const struct AttStateQuat *att_state,
  const struct GainsOrder2Vect3 *k_rate_ec)
{
  struct FloatVect3 nu = att_ref->att_3d;

  nu.x += k_rate_ec->k2.x * (att_ref->att_2d.x - att_state->att_2d.x);
  nu.y += k_rate_ec->k2.y * (att_ref->att_2d.y - att_state->att_2d.y);
  nu.z += k_rate_ec->k2.z * (att_ref->att_2d.z - att_state->att_2d.z);

  nu.x += k_rate_ec->k1.x * (att_ref->att_d.p - att_state->att_d.p);
  nu.y += k_rate_ec->k1.y * (att_ref->att_d.q - att_state->att_d.q);
  nu.z += k_rate_ec->k1.z * (att_ref->att_d.r - att_state->att_d.r);

  return nu;
}

/**
 * @brief Computes the attitude rate error control command.
 *
 * This function calculates the control input vector \c nu to correct the attitude error and
 * the associated angular rate errors using proportional-derivative gains. The quaternion error
 * between the reference and current attitudes is also factored in the control law.
 *
 * @param[in] att_ref   Pointer to the reference attitude and rate states (quaternion-based).
 * @param[in] att_state Pointer to the current attitude and rate states.
 * @param[in] k_att_ec  Pointer to gain parameters struct containing proportional, derivative, and jerk gains.
 *
 * @return The computed virtual control input vector.
 */
static struct FloatVect3 control_error_attitude(
  const struct AttQuat *att_ref,
  const struct AttStateQuat *att_state,
  const struct GainsOrder3Vect3 *k_att_ec)
{
  struct FloatVect3 nu = att_ref->att_3d;

  nu.x += k_att_ec->k3.x * (att_ref->att_2d.x - att_state->att_2d.x);
  nu.y += k_att_ec->k3.y * (att_ref->att_2d.y - att_state->att_2d.y);
  nu.z += k_att_ec->k3.z * (att_ref->att_2d.z - att_state->att_2d.z);

  nu.x += k_att_ec->k2.x * (att_ref->att_d.p - att_state->att_d.p);
  nu.y += k_att_ec->k2.y * (att_ref->att_d.q - att_state->att_d.q);
  nu.z += k_att_ec->k2.z * (att_ref->att_d.r - att_state->att_d.r);

  struct FloatQuat att_err; // rotation from state to reference
  float_quat_inv_comp_norm_shortest(&att_err, &att_state->att, &att_ref->att);
  // Multiplication by 2 needed to convert quaternion difference to angular rate.
  nu.x += k_att_ec->k1.x * att_err.qx * 2;
  nu.y += k_att_ec->k1.y * att_err.qy * 2;
  nu.z += k_att_ec->k1.z * att_err.qz * 2;

  return nu;
}

/**
 * @brief Computes the thrust control command based on desired and current thrust.
 *
 * This function calculates a corrected thrust command using a simple proportional
 * feedback law. The correction term is scaled by the thrust error gain k_thrust_ec
 * to reduce the difference between the desired thrust (thrust_ref->thrust)
 * and the current thrust (thrust_state). The resulting command is based on
 * the desired thrust feedforward input thrust_ref->thrust_d.
 *
 * @param[in] thrust_ref Pointer to a structure containing the desired thrust values.
 * @param[in] thrust_state Current measured thrust value.
 * @param[in] k_thrust_ec Proportional gain applied to the thrust error correction.
 *
 * @return The computed virtual control thrust input.
 */
static float control_error_thrust(
  const struct ThrustRef *thrust_ref,
  const float thrust_state,
  const float k_thrust_ec)
{
  float nu = thrust_ref->thrust_d;
  nu += k_thrust_ec * (thrust_ref->thrust - thrust_state);
  return nu;
}

/**
 * @brief Compute upper bounds for actuator rate commands based on actuator state and constraints.
 *
 * This function calculates the maximum allowable actuator rate commands (`u_d_max`) for each actuator to ensure that:
 * 1. The actuator position does not exceed its specified maximum (`act_max`) in the next timestep.
 * 2. The actuator rate does not exceed its maximum allowed rate (`act_rate_max`).
 * The computed upper bound is the minimum of the position-based rate limit and the maximum rate limit.
 *
 * @param[out] u_d_max Array to store the computed upper bounds for actuator rates.
 * @param[in] act_state Current states (positions) of the actuators.
 * @param[in] act_max Maximum allowable positions for the actuators.
 * @param[in] act_rate_max Maximum allowable rates for the actuators.
 * @param[in] actuator_bandwidth Bandwidth of the actuators used to compute rate limits.
 */
static void compute_wls_upper_bounds(float u_d_max[ANDI_NUM_ACT], const float act_state[ANDI_NUM_ACT],
                                     const float act_max[ANDI_NUM_ACT], const float act_rate_max[ANDI_NUM_ACT], const float actuator_bandwidth[ANDI_NUM_ACT])
{
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++) {
    // Calculate max rate allowed to avoid exceeding actuator max position in one timestep
    float rate_limit_pos = (act_max[i] - act_state[i]) * actuator_bandwidth[i];
    u_d_max[i] = (rate_limit_pos < act_rate_max[i]) ? rate_limit_pos : act_rate_max[i];
    Bound(u_d_max[i], 0.0f, INFINITY);
  }
}

/**
 * @brief Compute lower bounds for actuator rate commands based on actuator state and constraints.
 *
 * This function calculates the minimum allowable actuator rate commands (`u_d_min`) for each actuator to ensure that:
 * 1. The actuator position does not go below its specified minimum (`act_min`) in the next timestep.
 * 2. The actuator rate does not go below its minimum allowed rate (`act_rate_min`).
 * The computed lower bound is the maximum of the position-based rate limit and the minimum rate limit.
 * Additionally, if the computed lower bound is greater than zero (i.e., actuator cannot reverse direction), it is clamped to zero.
 *
 * @param[out] u_d_min Array to store the computed lower bounds for actuator rates.
 * @param[in] act_state Current states (positions) of the actuators.
 * @param[in] act_min Minimum allowable positions for the actuators.
 * @param[in] act_rate_min Minimum allowable rates for the actuators.
 * @param[in] actuator_bandwidth Bandwidth of the actuators used to compute rate limits.
 */
static void compute_wls_lower_bounds(float u_d_min[ANDI_NUM_ACT], const float act_state[ANDI_NUM_ACT],
                                     const float act_min[ANDI_NUM_ACT], const float act_rate_min[ANDI_NUM_ACT], const float actuator_bandwidth[ANDI_NUM_ACT])
{
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++) {
    // Calculate min rate allowed to avoid going below actuator min position in one timestep
    float rate_limit_pos = (act_min[i] - act_state[i]) * actuator_bandwidth[i];
    u_d_min[i] = (rate_limit_pos > act_rate_min[i]) ? rate_limit_pos : act_rate_min[i];
    Bound(u_d_min[i], -INFINITY, 0.0f);
  }
}

/**
 * @brief Compute input scaling factors for normalizing weighted least squares.
 *
 * This function calculates the input scaling factors (`u_scaler`) for each actuator
 * based on the provided minimum (`act_min`) and maximum (`act_max`) actuator values.
 * The scaling factor is computed as the inverse of the range (max - min). If the
 * range is zero, the scaling factor is set to 1.0 to avoid division by zero.
 *
 * u_norm = u_scaler * u
 *
 * @param[out] u_scaler Array to store the computed input scaling factors.
 * @param[in] act_min Array of minimum actuator values.
 * @param[in] act_max Array of maximum actuator values.
 */
static void compute_wls_u_scaler(float u_scaler[ANDI_NUM_ACT], const float act_min[ANDI_NUM_ACT],
                                 const float act_max[ANDI_NUM_ACT])
{
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++) {
    float range = fabs(act_max[i] - act_min[i]);
    if (range <= FLT_EPSILON) {
      u_scaler[i] = 1.0f;
    } else {
      u_scaler[i] = 1.0f / range;
    }
  }
}

/**
 * @brief Compute output scaling factors for normalizing weighted least squares outputs.
 *
 * For each output i this sets v_scaler[i] = 1.0f / abs(v[i]) when v[i] is non-zero; otherwise v_scaler[i] is set
 * to 1.0f to avoid a division-by-zero. The normalized output used in WLS is then
 *
 * v_norm = v_scaler * v.
 *
 * @param[out] v_scaler Array of length ANDI_NUM_ACT to store the computed inverse scaling factors.
 * @param[in]  v        Array of length ANDI_NUM_ACT containing reference/scaling values used to compute the inverse.
 */
static void compute_wls_v_scaler(float v_scaler[ANDI_NUM_ACT], const float v[ANDI_NUM_ACT])
{
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++) {
    if (fabs(v[i]) <= FLT_EPSILON) {
      v_scaler[i] = 1.0f;
    } else {
      v_scaler[i] = 1.0f / fabs(v[i]);
    }
  }
}

void stabilization_andi_init(void)
{
  // Compute gains
  andi_k_rate_ec = compute_error_gains_order_2_vect_3(&andi_p_rate_ec);
  andi_k_rate_rm = compute_reference_gains_order_2_vect_3(&andi_p_rate_rm);
  andi_k_att_ec = compute_error_gains_order_3_vect_3(&andi_p_att_ec);
  andi_k_att_rm = compute_reference_gains_order_3_vect_3(&andi_p_att_rm);
  andi_k_thrust_ec = andi_p_thrust_ec;
  andi_k_thrust_rm = andi_p_thrust_rm;

  // Initialize state variables
  rates_prev.p = 0.0f;
  rates_prev.q = 0.0f;
  rates_prev.r = 0.0f;

  float_quat_identity(&attitude_state_cf.att);
  attitude_state_cf.att_d.p = 0.0f;
  attitude_state_cf.att_d.q = 0.0f;
  attitude_state_cf.att_d.r = 0.0f;
  attitude_state_cf.att_2d.x = 0.0f;
  attitude_state_cf.att_2d.y = 0.0f;
  attitude_state_cf.att_2d.z = 0.0f;

  linear_state_cf.vel.x = 0.0f;
  linear_state_cf.vel.y = 0.0f;
  linear_state_cf.vel.z = 0.0f;
  linear_state_cf.acc.x = 0.0f;
  linear_state_cf.acc.y = 0.0f;
  linear_state_cf.acc.z = 0.0f;

  thrust_state = 0.0f;
  float_vect_zero(actuator_state, ANDI_NUM_ACT);
  float_vect_zero(u_cmd, ANDI_NUM_ACT);
  float_vect_zero(du_cmd, ANDI_NUM_ACT);
  float_vect_zero(nu_obj, ANDI_NUM_ACT);
  float_vect_zero(nu_ec, ANDI_NUM_ACT);
  float_vect_zero(nu_obm, ANDI_NUM_ACT);
  // float_vect_zero(nu_reconstructed, ANDI_NUM_ACT);

  // Initialize reference variables
  float_quat_identity(&attitude_ref.att);
  attitude_ref.att_d.p = 0.0f;
  attitude_ref.att_d.q = 0.0f;
  attitude_ref.att_d.r = 0.0f;
  attitude_ref.att_2d.x = 0.0f;
  attitude_ref.att_2d.y = 0.0f;
  attitude_ref.att_2d.z = 0.0f;
  attitude_ref.att_3d.x = 0.0f;
  attitude_ref.att_3d.y = 0.0f;
  attitude_ref.att_3d.z = 0.0f;

  thrust_ref.thrust = 0.0f;
  thrust_ref.thrust_d = 0.0f;

  // FIXME: These bounds should be set via parameters
  // Initialize attitude bounds (symmetric bounds on abs values)
  float_quat_identity(&attitude_bounds.att);
  attitude_bounds.att_d.p = 100.0f;
  attitude_bounds.att_d.q = 100.0f;
  attitude_bounds.att_d.r = 25.0f;
  attitude_bounds.att_2d.x = 100.0f;
  attitude_bounds.att_2d.y = 100.0f;
  attitude_bounds.att_2d.z = 20.0f;
  attitude_bounds.att_3d.x = 100.0f;
  attitude_bounds.att_3d.y = 100.0f;
  attitude_bounds.att_3d.z = 100.0f;

  // Limit thrust bounds (asymmetric bounds)
  thrust_bounds_min.thrust = THRUST_MIN;
  thrust_bounds_max.thrust = THRUST_MAX;
  thrust_bounds_min.thrust_d = -1000.0f;
  thrust_bounds_max.thrust_d = 1000.0f;

  // Initial control effectiveness matrix
  evaluate_obm_f_stb_u(ce_mat, &attitude_state_cf.att_d, &linear_state_cf.vel, ACTUATOR_PREF);

  // Initialize filters
  init_first_order_complementary_vect3(&attitude_rates_cf, &CUTOFF_FREQ_OMEGA, SAMPLE_TIME);
  init_butterworth_2_complementary_vect3(&attitude_accel_cf, &CUTOFF_FREQ_OMEGA_DOT, SAMPLE_TIME);
  angular_rates_obm.p = 0.0f;
  angular_rates_obm.q = 0.0f;
  angular_rates_obm.r = 0.0f;

  init_first_order_complementary_vect3(&linear_vel_cf, &CUTOFF_FREQ_VEL, SAMPLE_TIME);
  init_butterworth_2_complementary_vect3(&linear_accel_cf, &CUTOFF_FREQ_ACCEL, SAMPLE_TIME);
  linear_velocity_obm.x = 0.0f;
  linear_velocity_obm.y = 0.0f;
  linear_velocity_obm.z = 0.0f;

  // Bind T4 actuator feedback abi message
  AbiBindMsgACTUATORS_T4_IN(ABI_BROADCAST, &actuators_t4_in_event, actuators_t4_in_callback);

  // Precompute discrete-time actuator dynamics coefficients
  init_first_order_zoh_low_pass_array(ANDI_NUM_ACT, actuator_obm_zohlpf, ACTUATOR_DYNAMICS, SAMPLE_TIME);
  init_transport_delay_array(ANDI_NUM_ACT, actuator_obm_delay, ACTUATOR_DELAY, actuator_state);

// Start telemetry
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_stab_attitude_stabilization_andi);
  // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_THRUST, send_stab_thrust_stabilization_andi); // For debug only, dont forget to add your message.
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_MAT_STAB, send_eff_mat_stabilization_andi);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_V, send_wls_v_stabilization_andi);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WLS_U, send_wls_u_stabilization_andi);
  // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_PSEUDO_COMMAND, send_stab_pseudo_command_stabilization_andi);
  // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ON_BOARD_MODEL, send_on_board_model_stabilization_andi);
#endif
}

/**
 * @brief Initializes the ANDI stabilization controller state upon entering stabilization mode.
 *
 * This function resets all relevant state variables, filters, and actuator states to match the current
 * measurements when entering the ANDI stabilization mode. It ensures that the controller starts from a
 * consistent state when entering stabilization.
 *
 * NOTE: This function currently assumes that the actuator feedback message is being received.
 * FIXME: Transient free initialization for the cascaded complementary filters is not implemented.
 */
void stabilization_andi_enter(void)
{
  // Reset actuator states to current measurements
  fetch_actuators_t4(actuator_state, &actuators_t4_obs);
  reset_first_order_zoh_low_pass_array(ANDI_NUM_ACT, actuator_obm_zohlpf, actuator_state);
  reset_transport_delay_array(ANDI_NUM_ACT, actuator_obm_delay, actuator_state);

  // Clear previous rates
  rates_prev.p = 0.0f;
  rates_prev.q = 0.0f;
  rates_prev.r = 0.0f;

  // Reset state to current measurements, higher order derivatives are set to zero
  float_quat_vmult(&linear_state_cf.vel, stateGetNedToBodyQuat_f(), (struct FloatVect3 *)stateGetSpeedNed_f());
  linear_state_cf.acc.x = 0.0f;
  linear_state_cf.acc.y = 0.0f;
  linear_state_cf.acc.z = 0.0f;
  reset_first_order_complementary_vect3(&linear_vel_cf, &linear_state_cf.vel);
  reset_butterworth_2_complementary_vect3(&linear_accel_cf, &linear_state_cf.acc);
  linear_velocity_obm = linear_state_cf.vel;

  attitude_state_cf.att = *stateGetNedToBodyQuat_f();
  attitude_state_cf.att_d = *stateGetBodyRates_f();
  attitude_state_cf.att_2d.x = 0.0f;
  attitude_state_cf.att_2d.y = 0.0f;
  attitude_state_cf.att_2d.z = 0.0f;
  reset_first_order_complementary_rates(&attitude_rates_cf, &attitude_state_cf.att_d);
  reset_butterworth_2_complementary_vect3(&attitude_accel_cf, &attitude_state_cf.att_2d);
  angular_rates_obm = attitude_state_cf.att_d;

  attitude_ref.att = attitude_state_cf.att;
  attitude_ref.att_d = attitude_state_cf.att_d;
  attitude_ref.att_2d = attitude_state_cf.att_2d;
  attitude_ref.att_3d.x = 0.0f;
  attitude_ref.att_3d.y = 0.0f;
  attitude_ref.att_3d.z = 0.0f;

  thrust_state = evaluate_obm_thrust_z(actuator_state);

  thrust_ref.thrust = thrust_state;
  thrust_ref.thrust_d = 0.0f;

  // Reset controller states
  float_vect_zero(u_cmd, ANDI_NUM_ACT);
  float_vect_zero(nu_obm, ANDI_OUTPUTS);

  // Initial control effectiveness matrix
  evaluate_obm_f_stb_u(ce_mat, &attitude_state_cf.att_d, &linear_state_cf.vel, ACTUATOR_PREF);

}

/**
 * @brief Main ANDI stabilization control loop.
 *
 * This function implements the core logic of the ANDI stabilization controller.
 * The following steps are performed each time the function is called:
 * 1. Fetch measurements:
 * - Linear state (velocity and acceleration)
 * - Attitude state (quaternion, angular rates, angular accelerations)
 * - Actuator states (from T4 feedback and/or on-board model estimates)
 *
 * 2. Filtering:
 * - Apply cascaded complementary filtering to obtain an undelayed estimates of attitude and linear states. (used for state dependent model contribution (and scheduling))
 * - Apply low-pass filtering to obtain synchronized (delayed) estimates of attitude and thrust states. (used for control error computation)
 *
 * 3. Reference generation:
 * - Generate reference models for attitude and thrust using desired setpoints and reference model gains.
 * - Apply low-pass filtering to the references to synchronize with measurement lag.
 *
 * 4. Pseudo-command computation:
 * - Compute error controller pseudo command for attitude and thrust reference and feedback using parallel error controller.
 * - Compute state dependent on-board model pseudo command contribution based on undelayed state estimates.
 * - Combine pseudo commands to form the overall virtual control input vector.
 *
 * 5. Actuator command allocation:
 * - Update the control effectiveness matrix based on the current state.
 * - Compute actuator commands using weighted least squares allocation with bounds and scaling.
 *
 * @param[in] use_rate_control  Flag indicating whether to use rate control mode.
 * @param[in] in_flight        Flag indicating whether the vehicle is in flight.
 * @param[in,out] stab_setpoint Pointer to the stabilization setpoint structure.
 * @param[in,out] thrust_setpoint Pointer to the thrust setpoint structure.
 * @param[out] cmd             Pointer to the output command array for actuators.
 *
 * @note The attitude stabilization mode is \c ATTITUDE_HEADING_MODE.
 * A heading estimate and setpoint is needed. Consider implementing
 * \c ATTITUDE_HEADING_RATE_MODE to avoid needing a heading estimate.
 *
 * FIXME: The function currently recomputes gains at each call. This could be optimized to only recompute when parameters change.
 * FIXME: The choice of actuator feedback source (T4 vs on-board model) should be configurable.
 */
void stabilization_andi_run(bool use_rate_control, bool in_flight, struct StabilizationSetpoint *stab_setpoint,
                            struct ThrustSetpoint *thrust_setpoint, int32_t *cmd)
{
  // Recompute gains
  // FIXME: Only recompute when parameters change, use parameter update handler callback.
  andi_k_rate_ec = compute_error_gains_order_2_vect_3(&andi_p_rate_ec);
  andi_k_rate_rm = compute_reference_gains_order_2_vect_3(&andi_p_rate_rm);
  andi_k_att_ec = compute_error_gains_order_3_vect_3(&andi_p_att_ec);
  andi_k_att_rm = compute_reference_gains_order_3_vect_3(&andi_p_att_rm);
  andi_k_thrust_ec = andi_p_thrust_ec;
  andi_k_thrust_rm = andi_p_thrust_rm;

  // Fetch linear measurements
  struct LinState lin_meas;
  float_quat_vmult(&lin_meas.vel, stateGetNedToBodyQuat_f(),
                   (struct FloatVect3 *)stateGetSpeedNed_f()); // From Kalman filter
  lin_meas.acc = *stateGetAccelBody_f(); // From accelerometer

  // Fetch attitude measurements
  struct AttStateQuat attitude_meas;
  attitude_meas.att = *stateGetNedToBodyQuat_f(); // From Kalman filter
  attitude_meas.att_d = *stateGetBodyRates_f();   // From gyroscope
  attitude_meas.att_2d.x = (attitude_meas.att_d.p - rates_prev.p) * PERIODIC_FREQUENCY;
  attitude_meas.att_2d.y = (attitude_meas.att_d.q - rates_prev.q) * PERIODIC_FREQUENCY;
  attitude_meas.att_2d.z = (attitude_meas.att_d.r - rates_prev.r) * PERIODIC_FREQUENCY;
  rates_prev = attitude_meas.att_d; // Store previous rates for next acceleration calculation

  // Propagate and get on board model actuator state estimates
  float actuator_obm[ANDI_NUM_ACT];
  update_first_order_zoh_low_pass_array(ANDI_NUM_ACT, actuator_obm_zohlpf, u_cmd);
  get_first_order_zoh_low_pass_array(ANDI_NUM_ACT, actuator_obm_zohlpf, actuator_obm);
  update_transport_delay_array(ANDI_NUM_ACT, actuator_obm_delay, actuator_obm);
  get_transport_delay_array(ANDI_NUM_ACT, actuator_obm_delay, actuator_obm);

  // Get actuator measurements from T4 feedback
  fetch_actuators_t4(actuator_meas, &actuators_t4_obs);

  // FIXME: Make configurable which source to use for each actuator.
  // Choose actuator feedback for elevon and on board model for esc (because lack of rpm control).
  actuator_state[0] = actuator_meas[0];
  actuator_state[1] = actuator_meas[1];
  actuator_state[2] = actuator_obm[2];
  actuator_state[3] = actuator_obm[3];

  // Evaluate On Board Model with previous filtered state estimates
  struct FloatVect3 angular_accel_obm = evaluate_obm_moments(&attitude_state_cf.att_d, &linear_state_cf.vel,
                                        actuator_state);
  struct FloatVect3 linear_accel_obm = evaluate_obm_forces(&attitude_state_cf.att_d, &linear_state_cf.vel,
                                       actuator_state);

  // FIXME: Integral before filtering can cause numerical issues. Update complementary filter to handle this case.
  // FIXME: Numerical differentiation to get accelerations can be noisy. Consider using a filter before differentiation.
  // Cascaded complementary filter for linear velocity and accelerations measurements
  update_butterworth_2_complementary_vect3(&linear_accel_cf, &linear_accel_obm, &lin_meas.acc);
  linear_state_cf.acc = get_butterworth_2_complementary_vect3(&linear_accel_cf);
  float_vect3_integrate_fi(&linear_velocity_obm, &linear_state_cf.acc, SAMPLE_TIME);
  update_first_order_complementary_vect3(&linear_vel_cf, &linear_velocity_obm, &lin_meas.vel);
  linear_state_cf.vel = get_first_order_complementary_vect3(&linear_vel_cf);

  // Cascaded complementary filter for angular rates and accelerations measurements
  update_butterworth_2_complementary_vect3(&attitude_accel_cf, &angular_accel_obm, &attitude_meas.att_2d);
  attitude_state_cf.att_2d = get_butterworth_2_complementary_vect3(&attitude_accel_cf);
  float_rates_vect3_integrate_fi(&angular_rates_obm, &attitude_state_cf.att_2d, SAMPLE_TIME);
  update_first_order_complementary_rates(&attitude_rates_cf, &angular_rates_obm, &attitude_meas.att_d);
  attitude_state_cf.att_d = get_first_order_complementary_rates(&attitude_rates_cf);
  attitude_state_cf.att = attitude_meas.att;

  // Get thrust measurement from on board model
  thrust_state = evaluate_obm_thrust_z(actuator_state);

  // Reconstructed nu is repurposed to hold modeled moments and specific thrust from the on board model (for obm validation).
  // This can be used to compare the on board model prediction to the actual measured acceleration to validate model accuracy.
  // Do not compare this to the complementary filter values since these are influenced by the model, use the raw measurements instead.
  // nu_reconstructed[0] = angular_accel_obm.x;
  // nu_reconstructed[1] = angular_accel_obm.y;
  // nu_reconstructed[2] = angular_accel_obm.z;
  // nu_reconstructed[3] = thrust_state;

  // Get setpoints
  if (use_rate_control) {
    rates_des = stab_sp_to_rates_f(stab_setpoint);
    if (in_flight)
      generate_reference_rate(SAMPLE_TIME, &rates_des, &andi_k_rate_rm, &attitude_bounds, &attitude_ref);
  } else {
    attitude_des = stab_sp_to_quat_f(stab_setpoint);
    if (in_flight)
      generate_reference_attitude(SAMPLE_TIME, &attitude_des, &andi_k_att_rm, &attitude_bounds, &attitude_ref);
  }

  // FIXME: Thrust setpoint cant be of type THRUST_INCR_SP, this is not enforced but will silently fail
  thrust_des = th_sp_to_thrust_f(thrust_setpoint, 0, THRUST_AXIS_Z) * THRUST_MAX;
  generate_reference_thrust(SAMPLE_TIME, thrust_des, andi_k_thrust_rm, &thrust_bounds_min, &thrust_bounds_max,
                            &thrust_ref);

  // Construct pseudo control
  struct FloatVect3 nu_attitude;
  if (use_rate_control) {
    nu_attitude = control_error_rate(&attitude_ref, &attitude_state_cf, &andi_k_rate_ec);
  } else {
    nu_attitude = control_error_attitude(&attitude_ref, &attitude_state_cf, &andi_k_att_ec);
  }
  float nu_thrust = control_error_thrust(&thrust_ref, thrust_state, andi_k_thrust_ec);

  nu_ec[0] = nu_attitude.x;
  nu_ec[1] = nu_attitude.y;
  nu_ec[2] = nu_attitude.z;
  nu_ec[3] = nu_thrust;

  // State feedback from on board model
  if (USE_STATE_DYNAMICS) evaluate_obm_f_stb_x(nu_obm, &attitude_state_cf.att_d, &linear_state_cf.vel,
        &attitude_state_cf.att_2d, &linear_state_cf.acc, actuator_state);
  else float_vect_zero(nu_obm, ANDI_OUTPUTS);

  if (in_flight) {
    nu_obj[0] = nu_ec[0] - (nu_obm[0] * ANDI_RELAX_OBM);
    nu_obj[1] = nu_ec[1] - (nu_obm[1] * ANDI_RELAX_OBM);
    nu_obj[2] = nu_ec[2] - (nu_obm[2] * ANDI_RELAX_OBM);
  } else {
    nu_obj[0] = 0.0f;
    nu_obj[1] = 0.0f;
    nu_obj[2] = 0.0f;
  }
  nu_obj[3] = nu_ec[3] - (nu_obm[3] * ANDI_RELAX_OBM);

  // Compute control effectiveness matrix based on current states
  if (SCHEDULE_EFF) evaluate_obm_f_stb_u(ce_mat, &attitude_state_cf.att_d, &linear_state_cf.vel, actuator_state);

  // Solve control allocation using weighted least squares
  compute_wls_lower_bounds(du_min, actuator_state, ACTUATOR_MIN, ACTUATOR_D_MIN, ACTUATOR_DYNAMICS);
  compute_wls_upper_bounds(du_max, actuator_state, ACTUATOR_MAX, ACTUATOR_D_MAX, ACTUATOR_DYNAMICS);
  float wls_u_scaler[ANDI_NUM_ACT];
  float wls_v_scaler[ANDI_OUTPUTS] = {[0 ... ANDI_OUTPUTS - 1] = 1.0f}; // Disable v scaling
  compute_wls_u_scaler(wls_u_scaler, du_min,
                       du_max); // FIXME: Compute only once in advance for fixed bounds (important for well defined WLS Wu weights)
  // compute_wls_v_scaler(wls_v_scaler, nu_obj);

  float ce_mat_scaled[ANDI_OUTPUTS][ANDI_NUM_ACT];
  float *bwls[ANDI_OUTPUTS];
  // Scale control effectiveness matrix
  for (uint8_t i = 0; i < ANDI_OUTPUTS; i++) {
    for (uint8_t j = 0; j < ANDI_NUM_ACT; j++) {
      ce_mat_scaled[i][j] = ce_mat[i * ANDI_NUM_ACT + j] * wls_v_scaler[i] / wls_u_scaler[j];
    }
    bwls[i] = ce_mat_scaled[i];
  }

  // Scale actuator bounds and weights
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++) {
    wls_stab_p.u_min[i] = du_min[i] * wls_u_scaler[i];
    wls_stab_p.u_max[i] = du_max[i] * wls_u_scaler[i];
    wls_stab_p.u_pref[i] = (wls_stab_p.u_min[i] + wls_stab_p.u_max[i]) / 2.0f; // Use mid-point as preferred command
    wls_stab_p.Wu[i] = WLS_WU[i];
  }

  // Scale pseudo control and weights
  for (uint8_t i = 0; i < ANDI_OUTPUTS; i++) {
    wls_stab_p.v[i] = nu_obj[i] * wls_v_scaler[i];
    wls_stab_p.Wv[i] = WLS_WV[i];
  }

  wls_alloc(&wls_stab_p, bwls, 0, 0, 10);

  // Scale back actuator commands
  for (uint8_t i = 0; i < ANDI_NUM_ACT; i++) {
    du_cmd[i] = (wls_stab_p.u[i] / wls_u_scaler[i]);
    u_cmd[i] = du_cmd[i] / ACTUATOR_DYNAMICS[i] + actuator_state[i];
    Bound(u_cmd[i], ACTUATOR_MIN[i], ACTUATOR_MAX[i]);
  }

  // Commit actuator commands
  // Resulting commands are in rad for servo and rad/s for motor
  // Paparazzi expects the commands in pprz units (-MAX_PPRZ to MAX_PPRZ for servo, 0 to MAX_PPRZ for motor).
  // FIXME: Do not hardcode actuator layout
  commands[0] = (pprz_t)(u_cmd[0] / ACTUATOR_MAX[0] * MAX_PPRZ);
  commands[1] = (pprz_t)(u_cmd[1] / ACTUATOR_MAX[1] * MAX_PPRZ);
  commands[2] = (pprz_t)(sqrt(u_cmd[2] / ACTUATOR_MAX[2]) * MAX_PPRZ);
  commands[3] = (pprz_t)(sqrt(u_cmd[3] / ACTUATOR_MAX[3]) * MAX_PPRZ);

  // Set Thrust command for compatibility with other modules
  // FIXME: Do not hardcode actuator layout
  cmd[COMMAND_THRUST] = 0;
  cmd[COMMAND_THRUST] += (pprz_t)(sqrt(u_cmd[2] / ACTUATOR_MAX[2]) * MAX_PPRZ);
  cmd[COMMAND_THRUST] += (pprz_t)(sqrt(u_cmd[3] / ACTUATOR_MAX[3]) * MAX_PPRZ);
  cmd[COMMAND_THRUST] /= 2;
}

// FIXME: The following functions are to integrate the controller in the existing stabilization framework, find a better way to do this or move.
void stabilization_attitude_enter(void)
{
  ;
}

void stabilization_rate_enter(void)
{
  ;
}

void stabilization_rate_run(bool in_flight __attribute__((unused)),
                            struct StabilizationSetpoint *rate_sp __attribute__((unused)), struct ThrustSetpoint *thrust __attribute__((unused)),
                            int32_t *cmd __attribute__((unused)))
{
  ;
}

void stabilization_attitude_run(bool in_flight __attribute__((unused)),
                                struct StabilizationSetpoint *sp __attribute__((unused)), struct ThrustSetpoint *thrust __attribute__((unused)),
                                int32_t *cmd __attribute__((unused)))
{
  ;
}

// FIXME: This function is duplicated in stabilization_rate.c, find a way to share it
struct StabilizationSetpoint stabilization_rate_read_rc(struct RadioControl *rc)
{
  struct FloatRates rate_sp;
  FLOAT_RATES_ZERO(rate_sp);
  if (ROLL_RATE_DEADBAND_EXCEEDED(rc)) {
    rate_sp.p = rc->values[RC_RATE_P] * RC_RATE_MAX[0] / MAX_PPRZ;
  }
  if (PITCH_RATE_DEADBAND_EXCEEDED(rc)) {
    rate_sp.q = rc->values[RC_RATE_Q] * RC_RATE_MAX[1] / MAX_PPRZ;
  }
  if (YAW_RATE_DEADBAND_EXCEEDED(rc)) {
    rate_sp.r = rc->values[RC_RATE_R] * RC_RATE_MAX[2] / MAX_PPRZ;
  }
  return stab_sp_from_rates_f(&rate_sp);
}

// FIXME: Autopilot end flight detection is not applicable to tailsitters.
bool autopilot_in_flight_end_detection(bool motors_on __attribute__((unused)))
{
  return false;
}

// DEBUG FUNCTION
// static int counter = 0;
// /**
//  * @brief Debug function for actuators.
//  *
//  * Send step inputs to actuators and observe responses.
//  * useful to estimate actuator dynamics.
//  */
// void actuator_debug(bool do_servo_step, bool do_motor_step, float step_rate)
// {
//   if (counter++ > PERIODIC_FREQUENCY * step_rate) {
//     counter = 0;
//   }

//   if (counter <= (PERIODIC_FREQUENCY * step_rate) / 2) {
//     commands[0] = (pprz_t)(do_servo_step ? MAX_PPRZ : 0);
//     commands[1] = (pprz_t)(do_servo_step ? MAX_PPRZ : 0);
//     commands[2] = (pprz_t)(do_motor_step ? MAX_PPRZ : 0);
//     commands[3] = (pprz_t)(do_motor_step ? MAX_PPRZ : 0);
//   } else {
//     commands[0] = (pprz_t)(do_servo_step ? MIN_PPRZ : 0);
//     commands[1] = (pprz_t)(do_servo_step ? MIN_PPRZ : 0);
//     commands[2] = (pprz_t)(do_motor_step ? 0 : 0);
//     commands[3] = (pprz_t)(do_motor_step ? 0 : 0);
//   }
// }
