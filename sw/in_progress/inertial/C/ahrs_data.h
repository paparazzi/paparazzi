#ifndef AHRS_DATA_H
#define AHRS_DATA_H

struct ahrs_data {
  double dt;
  int nb_samples;
  double* t;

  /* true state */
  double* phi;
  double* theta;
  double* psi;

  double* bias_p;
  double* bias_q;
  double* bias_r;

  /* sensors    */
  double* gyro_p;
  double* gyro_q;
  double* gyro_r;

  double* accel_x;
  double* accel_y;
  double* accel_z;

  double* mag_x;
  double* mag_y;
  double* mag_z;

  /* measures */
  double* m_phi;
  double* m_theta;
  double* m_psi;

  /* estimated state */
  double* est_phi;
  double* est_theta;
  double* est_psi;

  double* est_bias_p;
  double* est_bias_q;
  double* est_bias_r;

  double* P11;
  double* P22;
  double* P33;
  double* P44;
  double* P55;
  double* P66;
  double* P77;
};

extern struct ahrs_data* ahrs_data_new(int len, double dt);
extern struct ahrs_data* ahrs_data_read_log(const char* filename);
extern void ahrs_data_save_state_euler(struct ahrs_data* ad, int idx, double* X, double* P);
extern void ahrs_data_save_state_quat(struct ahrs_data* ad, int idx, double* X, double* P);
extern void ahrs_data_save_measure(struct ahrs_data* ad, int idx);
extern void ahrs_data_save_state(struct ahrs_data* ad, int idx, double* X, double* P);

#endif /* AHRS_DATA_H */
