

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>



#include "math/pprz_algebra_double.h"
#include "fms/fms_autopilot_msg.h"
#include "fms/libeknav/raw_log.h"

void print_raw_log_entry(struct raw_log_entry *);
//void build_fake_log(void);

#define PRT(a) printf("%f ", a);
#define VI_GPS_DATA_VALID      2
#define GPS_READY(data_valid) (data_valid & (1<<VI_GPS_DATA_VALID))

/*
struct raw_log_entry __attribute__ ((packed)){
  double time;
  struct AutopilotMessageVIUp message;
}
*/


int main(int argc, char **argv)
{

  //  build_fake_log();

  int raw_log_fd = open(argv[1], O_RDONLY);

  if (raw_log_fd == -1) {
    perror("opening log\n");
    return -1;
  }
  //printf("%i\n", sizeof(struct raw_log_entry));
  //return 0;

  while (1) {
    struct raw_log_entry entry;
    ssize_t nb_read = read(raw_log_fd, &entry, sizeof(entry));
    if (nb_read != sizeof(entry)) { break; }
    print_raw_log_entry(&entry);
    printf("\n");
    //printf("%f %f %f %f", e.time, e.gyro.p, e.gyro.q, e.gyro.r);
  }

  return 0;
}



void print_raw_log_entry(struct raw_log_entry *e)
{
  struct DoubleVect3 tempvect;
  struct DoubleRates temprates;
  printf("%f\t", e->time);
  printf("%i\t", e->message.valid_sensors);
  RATES_FLOAT_OF_BFP(temprates, e->message.gyro);
  printf("% f % f % f\t", temprates.p,     temprates.q,     temprates.r);
  ACCELS_FLOAT_OF_BFP(tempvect, e->message.accel);
  printf("% f % f % f\t", tempvect.x,    tempvect.y,    tempvect.z);
  MAGS_FLOAT_OF_BFP(tempvect, e->message.mag);
  printf("% f % f % f\t", tempvect.x,      tempvect.y,     tempvect.z);
  printf("% i % i % i\t", e->message.ecef_pos.x, e->message.ecef_pos.y, e->message.ecef_pos.z);
  printf("% i % i % i\t", e->message.ecef_vel.x, e->message.ecef_vel.y, e->message.ecef_vel.z);
  double baro_scaled = (double)(e->message.pressure_absolute) / 256;
  printf("%f", baro_scaled);
}

/*
void build_fake_log(void) {
  int raw_log_fd = open( "log_test3.bin", O_WRONLY|O_CREAT, 00644);
  for (int i=0; i<5000; i++) {
    struct raw_log_entry e;
    e.time = i;
    write(raw_log_fd, &e, sizeof(e));
  }
  close(raw_log_fd);
}
*/
