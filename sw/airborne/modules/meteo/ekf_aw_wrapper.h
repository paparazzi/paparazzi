#ifndef EKF_AW_WRAPPER_H
#define EKF_AW_WRAPPER_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"
#include "modules/meteo/ekf_aw.h"


// EKF structure
struct ekfAw {

  // States
  struct NedCoor_f wind;
  struct NedCoor_f V_body;
  struct NedCoor_f offset;

  // Inputs
  struct FloatVect3 acc;  ///< Last accelerometer measurements
  struct FloatRates gyro;   ///< Last gyroscope measurements
  struct FloatEulers euler; /// Euler angles
  
  int32_t last_RPM_hover[4]; // Value obtained from ABI Callback
  int32_t last_RPM_pusher; // Value obtained from ABI Callback
  float RPM_hover[4]; /// Hover motor RPM
  float RPM_pusher; /// Pusher motor RPM
  float skew; /// Skew
  float elevator_angle;
  
  
  // Measurements
  struct FloatVect3 Vg_NED; /// Ground Speed
  struct FloatVect3 acc_filt;  ///< Last accelerometer measurements
  float V_pitot; /// Pitot tube airspeed

  // Innovation
  struct FloatVect3 innov_V_gnd;
  struct FloatVect3 innov_acc_filt;
  float innov_V_pitot;

  // Covariance
  float meas_cov[7];
  float state_cov[9];
  float process_cov[12];

  // Forces
  float fuselage_force[3];
  float wing_force[3];
  float elevator_force[3];
  float hover_force[3];
  float pusher_force[3];

  // Other
  bool reset;
  bool in_air;
  struct NedCoor_f wind_guess;
  struct NedCoor_f offset_guess;
  struct ekfHealth health; 
  uint64_t internal_clock;
  uint64_t time_last_on_gnd;
  uint64_t time_last_in_air;
  bool override_start;
  bool override_quick_convergence;

};

extern void ekf_aw_wrapper_init(void);
extern void ekf_aw_wrapper_periodic(void);
extern void ekf_aw_wrapper_fetch(void);

extern void set_in_air_status(bool);


extern float tau_filter_high;
extern float tau_filter_low;

extern struct ekfAw ekf_aw;

// Handlers
#define ekf_aw_wrapper_reset(_v) { \
  ekf_aw.reset = false;  \
  ekf_aw_reset();                  \
  ekf_aw_reset_health();                  \
}

#define ekf_aw_wrapper_set_wind_N(_v) { \
  ekf_aw.wind_guess.x = _v;  \
  ekf_aw_set_wind(&ekf_aw.wind_guess);                  \
}

#define ekf_aw_wrapper_set_wind_E(_v) { \
  ekf_aw.wind_guess.y = _v;  \
  ekf_aw_set_wind(&ekf_aw.wind_guess);                  \
}

#define ekf_aw_wrapper_set_wind_D(_v) { \
  ekf_aw.wind_guess.z = _v;  \
  ekf_aw_set_wind(&ekf_aw.wind_guess);                  \
}

#define ekf_aw_wrapper_set_offset_x(_v) { \
  ekf_aw.offset_guess.x = _v;  \
  ekf_aw_set_offset(&ekf_aw.offset_guess);                  \
}

#define ekf_aw_wrapper_set_offset_y(_v) { \
  ekf_aw.offset_guess.y = _v;  \
  ekf_aw_set_offset(&ekf_aw.offset_guess);                  \
}

#define ekf_aw_wrapper_set_offset_z(_v) { \
  ekf_aw.offset_guess.z = _v;  \
  ekf_aw_set_offset(&ekf_aw.offset_guess);                  \
}

#endif /* EKF_AW_WRAPPER_H */
