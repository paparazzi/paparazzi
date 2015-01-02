#include "overo_file_logger.h"

#include "subsystems/imu.h"

struct FileLogger file_logger;

void file_logger_init(char *filename)
{

  file_logger.outfile = fopen(filename, "w+");

}



void file_logger_periodic(void)
{
  static uint32_t foo = 0;
  foo++;
  fprintf(file_logger.outfile, "%f %d IMU_ACCEL_RAW %d %d %d\n", foo / 512., 42, imu.accel_unscaled.x,
          imu.accel_unscaled.y, imu.accel_unscaled.z);
  fprintf(file_logger.outfile, "%f %d IMU_GYRO_RAW %d %d %d\n", foo / 512., 42, imu.gyro_unscaled.p, imu.gyro_unscaled.q,
          imu.gyro_unscaled.r);

}

void file_logger_exit(void)
{

  fclose(file_logger.outfile);

}




