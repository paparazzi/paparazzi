#include "overo_file_logger.h"

#include "booz/booz_imu.h"

struct FileLogger file_logger;

void file_logger_init(char* filename) {

  file_logger.outfile = fopen(filename,"w+");

}



void file_logger_periodic(void) {
  static uint32_t foo = 0;
  foo++;
  fprintf(file_logger.outfile,"%f %d IMU_ACCEL_RAW %d %d %d\n",foo/512.,42,booz_imu.accel_unscaled.x,booz_imu.accel_unscaled.y,booz_imu.accel_unscaled.z);
  fprintf(file_logger.outfile,"%f %d IMU_GYRO_RAW %d %d %d\n",foo/512.,42,booz_imu.gyro_unscaled.p,booz_imu.gyro_unscaled.q,booz_imu.gyro_unscaled.r);

}

void file_logger_exit(void) {

  fclose(file_logger.outfile);

}




