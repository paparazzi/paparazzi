#ifndef AHRS_ON_SYNTH_H
#define AHRS_ON_SYNTH_H

#include "math/pprz_algebra_float.h"

struct traj {
  char *name;
  char *desc;
  void (*init_fun)(void);
  void (*update_fun)(void);

  double ts;
  double te;
};

struct AhrsOnSynth {

  struct traj *traj;

  double time;
  double dt;

  /* sensors */
  struct FloatRates  gyro_bias;
  struct FloatRates  gyro_noise;
  struct FloatVect3  accel_noise;

  float heading_noise;
  float heading_meas;

  /* state */
  struct FloatEulers ltp_to_imu_euler;
  struct FloatQuat   ltp_to_imu_quat;
  struct FloatRates  imu_rates;

  struct FloatVect3  ltp_jerk;
  struct FloatVect3  ltp_accel;
  struct FloatVect3  ltp_vel;
  struct FloatVect3  ltp_pos;



};

extern struct AhrsOnSynth aos;

extern void aos_init(int traj_nb);
extern void aos_run(void);
extern void aos_compute_sensors(void);
extern void aos_compute_state(void);


#define AHRS_TYPE_ICE  0
#define AHRS_TYPE_ICQ  1
#define AHRS_TYPE_FLQ  2
#define AHRS_TYPE_FCR  3
#define AHRS_TYPE_FCR2 4
#define AHRS_TYPE_FCQ  5
#define AHRS_TYPE_NB   6

extern char *ahrs_type_str[AHRS_TYPE_NB];

#endif /* AHRS_ON_SYNTH_H */
