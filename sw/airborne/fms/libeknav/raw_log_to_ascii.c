

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>




#include "math/pprz_algebra_float.h"

struct raw_log_entry {
  float time;
  struct FloatRates   gyro;
  struct FloatVect3   accel;
  struct FloatVect3   mag;
};

void print_raw_log_entry(struct raw_log_entry);

#define PRT(a) printf("%f ", a);



int main(int argc, char** argv) {

  const char* filename = "log_test3.bin";
  int raw_log_fd = open(filename, O_RDONLY); 
  
  // if (fd==-1) blaaa
  
  while (1) {
    struct raw_log_entry e;
    ssize_t nb_read = read(raw_log_fd, &e, sizeof(e));
    if (nb_read != sizeof(e)) break;
    print_raw_log_entry(e);
    //printf("%f %f %f %f", e.time, e.gyro.p, e.gyro.q, e.gyro.r);
    printf("\n");
  }

  return 0;
}



void print_raw_log_entry(struct raw_log_entry entry){
	printf("%f\t", entry.time);
	printf("%+f %+f %+f\t", entry.gyro.p, entry.gyro.q, entry.gyro.r);
	printf("%+f %+f %+f\t", entry.accel.x, entry.accel.y, entry.accel.z);
	printf("%+f %+f %+f\t", entry.mag.x, entry.mag.y, entry.mag.z);
}
