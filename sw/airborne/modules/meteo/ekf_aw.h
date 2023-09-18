#ifndef EKF_AW_H
#define EKF_AW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"

// Settings
struct ekfAwParameters {
  // Q
  float Q_accel;     ///< accel process noise
  float Q_gyro;      ///< gyro process noise
  float Q_mu;        ///< wind process noise
  float Q_k;         ///< offset process noise

  // R
  float R_V_gnd;      ///< speed measurement noise
  float R_accel_filt[3]; ///< filtered accel measurement noise
  float R_V_pitot;      ///< airspeed measurement noise

  // Model Based Parameters
    float vehicle_mass;
    
    // X Axis
    float k_fx_drag[2]; // Temporary setting for fuselage + hover prop
    float k_fx_fuselage[4];
    float k_fx_hover[3];
    float k_fx_wing[5];
    float k_fx_push[3];
    float k_fx_elev[3];

    // Y Axis
    float k_fy_beta;
    float k_fy_v;
    float k_fy_wing[5];

    // Z Axis
    float k_fz_fuselage[4];
    float k_fz_wing[4];
    float k_fz_hover[5];
    float k_fz_elev[2];

  // Other options
  bool use_model[3];
  bool use_pitot;
  bool propagate_offset;
  bool quick_convergence;
};

struct ekfHealth{
  bool healthy;
  uint16_t crashes_n;
};

extern struct ekfAwParameters ekf_aw_params;

// Init functions
extern void ekf_aw_init(void);
extern void ekf_aw_reset(void);

// Filtering functions
extern void ekf_aw_propagate(struct FloatVect3 *acc,struct FloatRates *gyro, struct FloatEulers *euler, float *pusher_RPM,float *hover_RPM_array, float *skew, float *elevator_angle, struct FloatVect3 * V_gnd, struct FloatVect3 *acc_filt, float *V_pitot,float dt);

// Getter/Setter functions
extern struct NedCoor_f ekf_aw_get_speed_body(void);
extern struct NedCoor_f ekf_aw_get_wind_ned(void);
extern struct NedCoor_f ekf_aw_get_offset(void);
extern struct FloatVect3 ekf_aw_get_innov_V_gnd(void);
extern struct FloatVect3 ekf_aw_get_innov_accel_filt(void);
extern float ekf_aw_get_innov_V_pitot(void);
extern void ekf_aw_get_meas_cov(float meas_cov[7]);
extern void ekf_aw_get_state_cov(float state_cov[9]);
extern void ekf_aw_get_process_cov(float process_cov[9]);
extern void ekf_aw_get_fuselage_force(float force[3]);
extern void ekf_aw_get_wing_force(float force[3]);
extern void ekf_aw_get_elevator_force(float force[3]);
extern void ekf_aw_get_hover_force(float force[3]);
extern void ekf_aw_get_pusher_force(float force[3]);
extern struct ekfAwParameters *ekf_aw_get_param_handle(void);

extern void ekf_aw_set_speed_body(struct NedCoor_f *s);
extern void ekf_aw_set_wind(struct NedCoor_f *s);
extern void ekf_aw_set_offset(struct NedCoor_f *s);
extern struct ekfHealth ekf_aw_get_health(void);

// Settings handlers
extern void ekf_aw_update_params(void);
extern void ekf_aw_reset_health(void);


// Handlers
#define ekf_aw_update_Q_accel(_v) { \
  ekf_aw_params.Q_accel = _v; \
  ekf_aw_update_params(); \
}

#define ekf_aw_update_Q_gyro(_v) { \
  ekf_aw_params.Q_gyro = _v; \
  ekf_aw_update_params(); \
}

#define ekf_aw_update_Q_mu(_v) { \
  ekf_aw_params.Q_mu = _v; \
  ekf_aw_update_params(); \
}

#define ekf_aw_update_Q_k(_v) { \
  ekf_aw_params.Q_k = _v; \
  ekf_aw_update_params(); \
}

#define ekf_aw_update_R_V_gnd(_v) { \
  ekf_aw_params.R_V_gnd = _v; \
  ekf_aw_update_params(); \
}

#define ekf_aw_update_R_accel_filt_x(_v) { \
  ekf_aw_params.R_accel_filt[0] = _v; \
  ekf_aw_update_params(); \
}

#define ekf_aw_update_R_accel_filt_y(_v) { \
  ekf_aw_params.R_accel_filt[1] = _v; \
  ekf_aw_update_params(); \
}

#define ekf_aw_update_R_accel_filt_z(_v) { \
  ekf_aw_params.R_accel_filt[2] = _v; \
  ekf_aw_update_params(); \
}

#define ekf_aw_update_R_V_pitot(_v) { \
  ekf_aw_params.R_V_pitot = _v; \
  ekf_aw_update_params(); \
}

#ifdef __cplusplus
}
#endif

#endif /* EKF_AW_H */