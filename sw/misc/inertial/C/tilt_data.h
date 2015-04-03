#ifndef TILT_DATA_H
#define TILT_DATA_H

struct tilt_data {
  double dt;
  int nb_samples;
  double* t;
  double* rate;
  double* angle;
  double* bias;

  double* gyro;
  double* ay;
  double* az;
  double* m_angle;

  double* est_angle;
  double* est_bias;

  double* P00;
  double* P01;
  double* P10;
  double* P11;

};

extern struct tilt_data* tilt_data_gen(void);
extern struct tilt_data* tilt_data_read_log(const char* filename);
extern struct tilt_data* tilt_data_new(int len, double dt);
extern void tilt_data_save_state(struct tilt_data* td, int idx, double* X, double* P);





#endif /* TILT_DATA_H */
